#!/bin/bash
# ============================================================================
# start.sh —— 一键启动（现场只需说清两件事：出发区号、本队安全区颜色）
# ============================================================================
#
# 用法：
#     ./start.sh <出发区 1-4> <本队颜色 red|blue> [车头朝向_度]
#
# 例：
#     ./start.sh 4 red          # 4 号区出发、本队是红队
#     ./start.sh 2 blue         # 2 号区出发、本队是蓝队
#     ./start.sh 1 红           # 中文也认
#     ./start.sh 4 red -45      # 摆位不是标准约定时，手工指定车头朝向
#
# 为什么要有这个脚本：
#   1. **车头朝向是出发区的纯函数**（= 从区中心指往外侧角的方向），
#      以前每次换区都要人算一遍这个角度、再手填进命令 —— 而**填错的代价是
#      整张地图旋转错**（位姿跑到场外、A* 起点不可通行、车一动不动）。
#      现在由 `field_elements.auto_start_heading_deg()` 单点推导，本脚本去问它，
#      **不在这里重抄一份表**（抄两份就一定会有一天对不上）。
#   2. 区号/颜色拼错时**当场报错退出**，而不是带着错值启动跑废一场。
#      颜色尤其重要：`TEAM_COLOR` 拼错会决定"哪个安全区是我方的"，
#      选错等于把物资投到对手家。
#
# 配套前提（摆位约定，务必核对）：
#   车身斜 45° 摆在出发区正方形对角线上，**车头朝出发区的外侧角**（背离场地中心
#   的那一角），开场动作是**后退**斜穿进场。所以：
#       1 号区车头 +135°   2 号区车头  +45°
#       3 号区车头 -135°   4 号区车头  -45°
#   若某次摆法不同（例如车头朝场地中心、或与边线平行），
#   **必须**用第三个参数显式指定实测角度。
# ============================================================================

set -euo pipefail
cd "$(dirname "$0")"

ZONE="${1:-}"
COLOR="${2:-}"
HEADING="${3:-}"

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
}

# ---------- 参数校验：宁可现在报错，也不要带着错值跑废一场 ----------
if [ -z "$ZONE" ] || [ -z "$COLOR" ]; then
    echo "❌ 参数不足：需要 <出发区 1-4> <本队颜色 red|blue> [车头朝向_度]" >&2
    echo >&2
    usage
fi

case "$ZONE" in
    1|2|3|4) ;;
    *) echo "❌ 出发区必须是 1~4，收到 '$ZONE'" >&2; exit 2 ;;
esac

# 颜色别名：与 main.py 的 _TEAM_RED_ALIASES / _TEAM_BLUE_ALIASES 保持一致
COLOR_LC=$(printf '%s' "$COLOR" | tr 'A-Z' 'a-z')
case "$COLOR_LC" in
    red|r|红|红队|hong|red_team|hongdui)   COLOR="red"  ;;
    blue|b|蓝|蓝队|lan|blue_team|landui)   COLOR="blue" ;;
    *)
        echo "❌ 颜色无法识别：'$COLOR'（合法值：red/blue，也接受 红/蓝 等写法）" >&2
        echo "   ⚠️ 这个值决定『哪个安全区是我方的』——拼错等于把物资投到对手家。" >&2
        exit 2
        ;;
esac

if [ -n "$HEADING" ]; then
    if ! printf '%s' "$HEADING" | grep -qE '^-?[0-9]+(\.[0-9]+)?$'; then
        echo "❌ 车头朝向必须是数字（度），收到 '$HEADING'" >&2; exit 2
    fi
fi

# ---------- 自动推导车头朝向（唯一来源 = field_elements，不在此重抄表） ----------
AUTO=$(PYTHONPATH=src python3 -c \
    "from rescue_robot.perception.field_elements import auto_start_heading_deg as a; \
     print(a($ZONE) if a($ZONE) is not None else '')" 2>/dev/null || true)

if [ -n "$HEADING" ]; then
    HEAD_TXT="${HEADING}°（手工指定，将覆盖自动推导）"
elif [ -n "$AUTO" ]; then
    HEAD_TXT="${AUTO}°（按 ${ZONE} 号区自动推导）"
else
    HEAD_TXT="未能自动推导（将回退到默认『朝场地内侧』——摆位不符时这是错的！）"
fi

# ---------- 交给 run.sh ----------
export TEAM_COLOR="$COLOR"
export START_ZONE="$ZONE"
if [ -n "$HEADING" ]; then
    export START_HEADING_DEG="$HEADING"
else
    unset START_HEADING_DEG || true     # 确保不会残留上次的值
fi

echo "════════════════════════════════════════════════════════"
echo "  出发区      : ${ZONE} 号"
echo "  本队安全区  : ${COLOR}"
echo "  车头朝向    : ${HEAD_TXT}"
echo "────────────────────────────────────────────────────────"
echo "  ⚠️ 请确认：车头朝出发区**外侧角**（背离场地中心的那一角），"
echo "     开场是**后退**斜穿进场。摆位不符请用第 3 个参数指定实测角度。"
echo "════════════════════════════════════════════════════════"
echo

exec ./run.sh "${@:4}"
