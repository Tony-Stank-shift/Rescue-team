#!/bin/bash
# ============================================================
# deploy.sh —— 快速部署脚本 (Linux/Raspberry Pi)
#
# 用法: bash scripts/deploy.sh [--target HOST] [--dry-run]
# 目标: <5 分钟从代码修改到机器人运行
# ============================================================

set -euo pipefail

TARGET="${TARGET:-raspberrypi.local}"
TARGET_USER="${TARGET_USER:-pi}"
TARGET_PATH="${TARGET_PATH:-/home/pi/rescue-robot}"
DRY_RUN=false
SKIP_TEST=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --target) TARGET="$2"; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --skip-test) SKIP_TEST=true; shift ;;
        -h|--help)
            echo "用法: $0 [--target HOST] [--dry-run] [--skip-test]"
            echo ""
            echo "快速部署救援机器人代码到树莓派。"
            echo ""
            echo "选项:"
            echo "  --target HOST   目标主机 (默认: raspberrypi.local)"
            echo "  --dry-run       模拟运行，不实际部署"
            echo "  --skip-test     跳过本地测试"
            exit 0
            ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "  救援机器人 - 快速部署"
echo "============================================"
echo "  目标: ${TARGET_USER}@${TARGET}:${TARGET_PATH}"
echo "  项目: ${PROJECT_DIR}"
echo ""

# ─── 步骤 1: 依赖检查 (10s) ───
echo "[1/5] 依赖检查..."
python3 -c "import sys; assert sys.version_info >= (3, 10), '需要 Python 3.10+'" 2>/dev/null || {
    echo "  ❌ Python 版本不满足要求 (需要 3.10+)"
    exit 1
}
echo "  ✅ Python $(python3 --version)"

if [ "$DRY_RUN" = true ]; then
    echo "  🔧 DRY RUN 模式 - 跳过实际部署"
    exit 0
fi

# ─── 步骤 2: 本地测试 (30s) ───
if [ "$SKIP_TEST" = false ]; then
    echo "[2/5] 本地冒烟测试..."
    cd "$PROJECT_DIR"
    PYTHONIOENCODING=utf-8 PYTHONPATH=src python3 -c "
from rescue_robot.innovation.config_loader import ConfigLoader
cl = ConfigLoader()
cfg = cl.load_yaml('config/robot.default.yaml')
print(f'  ✅ 配置加载成功: {len(cfg)} 个顶层键')
print(f'  ✅ 策略权重: {cfg[\"robot\"][\"strategy_weights\"]}')
" || {
        echo "  ❌ 冒烟测试失败"
        exit 1
    }
    echo "  ✅ 冒烟测试通过"
else
    echo "[2/5] 本地测试 (已跳过)"
fi

# ─── 步骤 3: 代码同步 (30s) ───
echo "[3/5] 代码同步..."
rsync -avz --delete \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude 'logs' \
    "$PROJECT_DIR/src/" "${TARGET_USER}@${TARGET}:${TARGET_PATH}/src/" 2>/dev/null || {
    echo "  ⚠️  rsync 失败，尝试 scp..."
    tar czf /tmp/rescue-deploy.tar.gz -C "$PROJECT_DIR" src/ config/ scripts/
    scp /tmp/rescue-deploy.tar.gz "${TARGET_USER}@${TARGET}:/tmp/"
    ssh "${TARGET_USER}@${TARGET}" "cd ${TARGET_PATH} && tar xzf /tmp/rescue-deploy.tar.gz"
    rm /tmp/rescue-deploy.tar.gz
}
echo "  ✅ 代码已同步"

# ─── 步骤 4: 远程重启服务 (60s) ───
echo "[4/5] 远程重启服务..."
ssh "${TARGET_USER}@${TARGET}" "
    cd ${TARGET_PATH}
    # 检查依赖
    python3 -c 'import yaml' 2>/dev/null || pip3 install pyyaml --quiet
    # 重启服务
    sudo systemctl restart rescue-robot 2>/dev/null || {
        echo '  ⚠️  systemd 不可用，请手动启动: cd ${TARGET_PATH} && RUN_MODE=real python3 -m rescue_robot.main'
    }
"
echo "  ✅ 服务已重启"

# ─── 步骤 5: 健康检查 (10s) ───
echo "[5/5] 健康检查..."
sleep 3
ssh "${TARGET_USER}@${TARGET}" "
    pgrep -f 'rescue_robot.main' > /dev/null && echo '  ✅ 进程运行中' || echo '  ⚠️  进程未运行，请检查日志'
    # 检查 WebSocket 端口
    ss -tlnp 2>/dev/null | grep -q 8765 && echo '  ✅ WebSocket 端口 8765 已监听' || echo '  ⚠️  端口未监听'
" 2>/dev/null || echo "  ⚠️  无法连接到 ${TARGET}，请手动验证"

echo ""
echo "============================================"
echo "  部署完成! 🚀"
echo "============================================"
