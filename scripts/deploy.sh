#!/bin/bash
# ============================================================
# deploy.sh —— 快速部署脚本 (RDK X5 / Linux)
#
# 用法: bash scripts/deploy.sh [--target HOST] [--user USER] [--path PATH]
#                              [--dry-run] [--skip-test]
#
# 目标: <5 分钟从代码修改到机器人运行
#
# 【默认目标 = RDK X5 现场约定】
#     sunrise@192.168.50.2:/home/sunrise/rescue
#   · RDK X5 板载 40PIN UART1 的设备名已实测确认为 /dev/ttyS1 @115200，
#     上位机代码默认就是它，RDK 侧部署后无需再设 CHASSIS_PORT。
#   · 电脑 USB-TTL 直连下位机调试时，用 --target（必要时配合 --user/--path）
#     覆写目标，例如: bash scripts/deploy.sh --target 192.168.1.23 \
#                        --user sunrise --path /home/sunrise/rescue
#
# 【同步内容】rsync 路径与 tar 兜底路径**完全一致**（T2-19 / T2-20）：
#     src/     上位机代码
#     config/  现场 YAML（robot.default.yaml / field.default.yaml）
#     scripts/ 部署与运维脚本
#     tools/   现场自检程序（tools/hw_selftest.py + tools/hw_selftest/）
#   ⚠️ 以前 rsync 只同步 src/（config/ 与 tools/ 只有 tar 兜底才带），
#      导致"现场改了 config 没反应""自检程序上不了车"，现已统一。
# ============================================================

set -euo pipefail

# ─── 默认目标：RDK X5 现场约定（电脑 USB-TTL 调试时用 --target 等覆写）───
TARGET="${TARGET:-192.168.50.2}"
TARGET_USER="${TARGET_USER:-sunrise}"
TARGET_PATH="${TARGET_PATH:-/home/sunrise/rescue}"
DRY_RUN=false
SKIP_TEST=false

# 需要同步的相对目录（rsync 与 tar 兜底都用这个清单，保证两条路径一致）
SYNC_DIRS=(src config scripts tools)
# 允许 --delete 的目录：只对纯代码目录启用，避免误删车上特有的 config/scripts
DELETE_DIRS=(src tools)

usage() {
    cat <<EOF
用法: $0 [--target HOST] [--user USER] [--path PATH] [--dry-run] [--skip-test]

快速部署救援机器人代码到 RDK X5。

默认目标（RDK X5 现场约定）:
  ${TARGET_USER}@${TARGET}:${TARGET_PATH}
  · RDK X5 默认 sunrise@192.168.50.2:/home/sunrise/rescue；
  · 电脑 USB-TTL 调试时用 --target 覆盖（必要时再配 --user / --path）。

同步内容（rsync 与 tar 兜底一致）:
  src/  config/  scripts/  tools/
  (tools/ = 现场自检程序 hw_selftest.py + hw_selftest/；config/ = 现场 YAML)

选项:
  --target HOST   目标主机 (默认: \$TARGET 或 192.168.50.2)
  --user USER     目标用户 (默认: \$TARGET_USER 或 sunrise)
  --path PATH     目标路径 (默认: \$TARGET_PATH 或 /home/sunrise/rescue)
  --dry-run       模拟运行：只做依赖检查并打印同步计划，不实际部署、不连远端
  --skip-test     跳过本地测试
  -h, --help      显示本帮助

环境变量: TARGET / TARGET_USER / TARGET_PATH 可替代对应选项。
EOF
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)
            [[ $# -ge 2 ]] || { echo "❌ --target 缺少参数" >&2; exit 1; }
            TARGET="$2"; shift 2 ;;
        --user)
            [[ $# -ge 2 ]] || { echo "❌ --user 缺少参数" >&2; exit 1; }
            TARGET_USER="$2"; shift 2 ;;
        --path)
            [[ $# -ge 2 ]] || { echo "❌ --path 缺少参数" >&2; exit 1; }
            TARGET_PATH="$2"; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --skip-test) SKIP_TEST=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "  救援机器人 - 快速部署 (RDK X5)"
echo "============================================"
echo "  目标: ${TARGET_USER}@${TARGET}:${TARGET_PATH}"
echo "  项目: ${PROJECT_DIR}"
echo "  同步: ${SYNC_DIRS[*]}/"
echo ""

# ─── 步骤 1: 依赖检查 (10s) ───
echo "[1/5] 依赖检查..."
python3 -c "import sys; assert sys.version_info >= (3, 10), '需要 Python 3.10+'" 2>/dev/null || {
    echo "  ❌ Python 版本不满足要求 (需要 3.10+)"
    exit 1
}
echo "  ✅ Python $(python3 --version)"

# 源目录自检：缺了就直接说清楚，别等部署完才发现车上没有
for d in "${SYNC_DIRS[@]}"; do
    if [ -d "$PROJECT_DIR/$d" ]; then
        echo "  ✅ 源目录存在: $d/"
    else
        echo "  ❌ 源目录缺失: $PROJECT_DIR/$d （请确认在仓库内运行本脚本）"
        exit 1
    fi
done
[ -f "$PROJECT_DIR/tools/hw_selftest.py" ] \
    && echo "  ✅ 现场自检程序: tools/hw_selftest.py" \
    || echo "  ⚠️  未找到 tools/hw_selftest.py（自检程序将无法上车）"
[ -f "$PROJECT_DIR/config/robot.default.yaml" ] \
    && echo "  ✅ 现场配置: config/robot.default.yaml" \
    || echo "  ⚠️  未找到 config/robot.default.yaml"

if [ "$DRY_RUN" = true ]; then
    echo "  🔧 DRY RUN 模式 - 跳过实际部署（不连远端、不交互）"
    echo ""
    echo "  将同步以下内容到 ${TARGET_USER}@${TARGET}:${TARGET_PATH}/ :"
    for d in "${SYNC_DIRS[@]}"; do
        if [[ " ${DELETE_DIRS[*]} " == *" $d "* ]]; then
            echo "    - $d/   (rsync --delete)"
        else
            echo "    - $d/   (rsync，不删除目标机多余文件)"
        fi
    done
    echo "  远端路径不存在时先 mkdir -p；rsync 不可用时 tar 兜底打同样的目录集。"
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
# src/ config/ scripts/ tools/ 全部同步：rsync 路径与 tar 兜底路径行为一致
echo "[3/5] 代码同步 (src/ config/ scripts/ tools/)..."

# 目标目录可能还不存在（首次部署到 ~/rescue）→ 先建，否则 rsync/tar 都会失败
ssh "${TARGET_USER}@${TARGET}" "mkdir -p '${TARGET_PATH}'" 2>/dev/null || true

rsync_one() {
    local rel="$1"
    local extra=""
    [[ " ${DELETE_DIRS[*]} " == *" $rel "* ]] && extra="--delete"
    # shellcheck disable=SC2086
    rsync -avz $extra \
        --exclude '__pycache__' \
        --exclude '*.pyc' \
        --exclude '.git' \
        --exclude 'logs' \
        "$PROJECT_DIR/$rel/" "${TARGET_USER}@${TARGET}:${TARGET_PATH}/$rel/" 2>/dev/null
}

SYNC_MODE="rsync"
for d in "${SYNC_DIRS[@]}"; do
    if ! rsync_one "$d"; then
        SYNC_MODE="tar"
        echo "  ⚠️  rsync 同步 $d/ 失败，改用 tar 兜底..."
        break
    fi
    echo "  ✅ rsync: $d/"
done

if [ "$SYNC_MODE" = "tar" ]; then
    # 兜底：打的目录集与上面 SYNC_DIRS 完全一致（含 tools/ 与 config/）
    tar czf /tmp/rescue-deploy.tar.gz -C "$PROJECT_DIR" \
        --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
        "${SYNC_DIRS[@]}"
    scp /tmp/rescue-deploy.tar.gz "${TARGET_USER}@${TARGET}:/tmp/"
    ssh "${TARGET_USER}@${TARGET}" "mkdir -p '${TARGET_PATH}' && \
        tar xzf /tmp/rescue-deploy.tar.gz -C '${TARGET_PATH}'"
    rm -f /tmp/rescue-deploy.tar.gz
    echo "  ✅ tar 兜底: ${SYNC_DIRS[*]}/"
fi
echo "  ✅ 代码已同步"

# ─── 步骤 4: 远程重启服务 (60s) ───
echo "[4/5] 远程重启服务..."
ssh "${TARGET_USER}@${TARGET}" "
    cd ${TARGET_PATH}
    # 检查依赖
    python3 -c 'import yaml' 2>/dev/null || pip3 install pyyaml --quiet
    # 重启服务
    sudo systemctl restart rescue-robot 2>/dev/null || {
        echo '  ⚠️  systemd 不可用（本仓库无 rescue-robot.service），请手动启动:'
        echo '      cd ${TARGET_PATH} && PYTHONPATH=src python3 -m rescue_robot.main'
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
    # 检查两条同步路径的关键文件真的上车了（T2-19 / T2-20）
    [ -f tools/hw_selftest.py ] && echo '  ✅ 自检程序已上车: tools/hw_selftest.py' || echo '  ⚠️  tools/hw_selftest.py 缺失'
    [ -d tools/hw_selftest ] && echo '  ✅ 自检模块已上车: tools/hw_selftest/' || echo '  ⚠️  tools/hw_selftest/ 缺失'
    [ -f config/robot.default.yaml ] && echo '  ✅ 现场配置已上车: config/robot.default.yaml' || echo '  ⚠️  config/robot.default.yaml 缺失'
" 2>/dev/null || echo "  ⚠️  无法连接到 ${TARGET}，请手动验证"

echo ""
echo "============================================"
echo "  部署完成! 🚀"
echo "============================================"
