#!/bin/bash
# =============================================================================
# EKF 快速诊断 — 无需额外安装, 纯 ros2 CLI 工具
# =============================================================================
# 用法:
#   ./ekf_quick_check.sh              # 一次性快照
#   ./ekf_quick_check.sh --watch 3    # 每 3 秒刷新 (Ctrl+C 退出)
# =============================================================================

WATCH_INTERVAL="${1:-0}"
if [[ "$1" == "--watch" ]]; then
    WATCH_INTERVAL="${2:-3}"
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

check_topic_hz() {
    local topic="$1"
    local expected_min="$2"
    # 用 ros2 topic hz 采样 3 秒
    local hz_output
    hz_output=$(timeout 4 ros2 topic hz "$topic" --window 20 2>/dev/null | tail -1)
    if [[ -z "$hz_output" ]]; then
        echo -e "${RED}❌ 无数据${NC}"
        return
    fi
    local hz
    hz=$(echo "$hz_output" | grep -oP 'average rate: \K[0-9.]+')
    if [[ -z "$hz" ]]; then
        echo -e "${YELLOW}⚠️ 采样中...${NC}"
        return
    fi
    local hz_int=${hz%.*}
    if (( $(echo "$hz < $expected_min * 0.5" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${RED}${hz}Hz (期望 >${expected_min}Hz)${NC}"
    elif (( $(echo "$hz < $expected_min * 0.8" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${YELLOW}${hz}Hz (期望 >${expected_min}Hz)${NC}"
    else
        echo -e "${GREEN}${hz}Hz${NC}"
    fi
}

check_tf() {
    local from="$1"
    local to="$2"
    local output
    output=$(timeout 2 ros2 run tf2_ros tf2_echo "$from" "$to" 2>/dev/null | head -3)
    if [[ -z "$output" ]]; then
        echo -e "${RED}❌ 缺失${NC}"
    else
        echo -e "${GREEN}✅ 存在${NC}"
    fi
}

do_check() {
    echo ""
    echo -e "${BOLD}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║     🔍 EKF 快速诊断 @ $(date '+%H:%M:%S')                          ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════╣${NC}"

    # 话题频率
    echo -e "${BOLD}║ 话题频率:${NC}"
    printf "║   %-22s " "/odom (期望>10Hz)"; check_topic_hz "/odom" 10
    printf "║   %-22s " "/imu/data_raw (>20Hz)"; check_topic_hz "/imu/data_raw" 20
    printf "║   %-22s " "/gps/fix (>1Hz)"; check_topic_hz "/gps/fix" 1
    printf "║   %-22s " "/odometry/gps (>2Hz)"; check_topic_hz "/odometry/gps" 2
    printf "║   %-22s " "/cmd_vel (>5Hz)"; check_topic_hz "/cmd_vel" 5

    # TF 检查
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║ TF 树检查:${NC}"
    for pair in "map odom" "odom base_footprint" "odom base_link" "base_link livox_frame"; do
        read -r from to <<< "$pair"
        printf "║   %-20s → %-20s " "$from" "$to"
        check_tf "$from" "$to"
    done

    # GPS 最新数据
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║ 最新 GPS:${NC}"
    local gps_data
    gps_data=$(timeout 2 ros2 topic echo /gps/fix --once --field latitude --field longitude --field altitude --field status 2>/dev/null)
    if [[ -z "$gps_data" ]]; then
        echo -e "║   ${RED}❌ 无 GPS 数据${NC}"
    else
        echo -e "║   $gps_data" | while read line; do
            echo -e "║   ${CYAN}$line${NC}"
        done
    fi

    # /odometry/gps vs /odom 位置对比
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║ 里程计 vs GPS 里程计 (位置对比):${NC}"
    local odom_pos
    odom_pos=$(timeout 2 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null)
    local gps_odom_pos
    gps_odom_pos=$(timeout 2 ros2 topic echo /odometry/gps --once --field pose.pose.position 2>/dev/null)

    if [[ -n "$odom_pos" ]]; then
        echo -e "║   /odom 位置:         ${CYAN}$odom_pos${NC}"
    else
        echo -e "║   /odom: ${RED}无数据${NC}"
    fi
    if [[ -n "$gps_odom_pos" ]]; then
        echo -e "║   /odometry/gps 位置: ${CYAN}$gps_odom_pos${NC}"
    else
        echo -e "║   /odometry/gps: ${RED}无数据${NC}"
    fi

    # 最新 cmd_vel
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║ 最新 /cmd_vel:${NC}"
    local cmd
    cmd=$(timeout 2 ros2 topic echo /cmd_vel --once --field linear --field angular 2>/dev/null)
    if [[ -z "$cmd" ]]; then
        echo -e "║   ${YELLOW}⚠️ 无 /cmd_vel 输出 (机器人可能未在导航)${NC}"
    else
        echo -e "║   ${CYAN}$cmd${NC}"
    fi

    echo -e "${BOLD}╚══════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

if [[ "$WATCH_INTERVAL" -gt 0 ]]; then
    echo -e "${CYAN}持续监控模式, 每 ${WATCH_INTERVAL}s 刷新, Ctrl+C 退出${NC}"
    while true; do
        clear 2>/dev/null || true
        do_check
        sleep "$WATCH_INTERVAL"
    done
else
    do_check
fi
