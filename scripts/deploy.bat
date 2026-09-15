@echo off
REM ============================================================
REM deploy.bat —— 快速部署脚本 (Windows)
REM
REM 用法: scripts\deploy.bat [--skip-test]
REM
REM 默认目标（RDK X5 现场约定，与 deploy.sh 保持一致）:
REM   192.168.50.2 / sunrise / /home/sunrise/rescue
REM   · 电脑 USB-TTL 调试或目标不是默认机时，用 TARGET / TARGET_USER / TARGET_PATH
REM     三个环境变量覆盖（见脚本头部与 RUNBOOK §1.4）。
REM
REM 同步内容（rsync 路径与 tar 兜底一致）: src\ config\ scripts\ tools\
REM   tools\ = 现场自检程序 hw_selftest.py + hw_selftest\
REM ============================================================

setlocal enabledelayedexpansion

set TARGET=%TARGET%
if "%TARGET%"=="" set TARGET=192.168.50.2
set TARGET_USER=%TARGET_USER%
if "%TARGET_USER%"=="" set TARGET_USER=sunrise
set TARGET_PATH=%TARGET_PATH%
if "%TARGET_PATH%"=="" set TARGET_PATH=/home/sunrise/rescue
set SKIP_TEST=%1

echo ============================================
echo   救援机器人 - 快速部署
echo ============================================
echo   目标: %TARGET_USER%@%TARGET%:%TARGET_PATH%
echo.

REM 步骤 1: 依赖检查
echo [1/5] 依赖检查...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo   ❌ Python 未安装
    exit /b 1
)
echo   ✅ Python 可用

REM 步骤 2: 本地测试
if "%SKIP_TEST%"=="--skip-test" (
    echo [2/5] 本地测试 (已跳过^)
) else (
    echo [2/5] 本地冒烟测试...
    cd /d "%~dp0\.."
    set PYTHONIOENCODING=utf-8
    set PYTHONPATH=src
    python -c "from rescue_robot.innovation.config_loader import ConfigLoader; cl=ConfigLoader(); cfg=cl.load_yaml('config/robot.default.yaml'); print(f'  ✅ 配置加载成功')" >nul 2>&1
    if %errorlevel% neq 0 (
        echo   ❌ 冒烟测试失败
        exit /b 1
    )
    echo   ✅ 冒烟测试通过
)

REM 步骤 3-5: 需要 SSH 客户端
where ssh >nul 2>&1
if %errorlevel% neq 0 (
    echo [3-5] ❌ 未找到 SSH 客户端, 无法远程部署
    echo   请安装 OpenSSH 客户端或在 WSL 中使用 deploy.sh
    exit /b 1
)

echo [3/5] 代码同步 (src config scripts tools)...
where rsync >nul 2>&1
if %errorlevel% equ 0 (
    REM src\ 与 tools\ 带 --delete（纯代码目录）；config\ 与 scripts\ 不带，避免误删车上特有文件
    rsync -avz --delete --exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude logs "%~dp0..\src\" "%TARGET_USER%@%TARGET%:%TARGET_PATH%/src/"
    rsync -avz --exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude logs "%~dp0..\config\" "%TARGET_USER%@%TARGET%:%TARGET_PATH%/config/"
    rsync -avz --exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude logs "%~dp0..\scripts\" "%TARGET_USER%@%TARGET%:%TARGET_PATH%/scripts/"
    rsync -avz --delete --exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude logs "%~dp0..\tools\" "%TARGET_USER%@%TARGET%:%TARGET_PATH%/tools/"
) else (
    echo   ⚠️  rsync 不可用, 尝试 scp...
    REM 兜底打的目录集与上面 rsync 完全一致（含 tools\ 与 config\）
    tar czf %TEMP%\rescue-deploy.tar.gz -C "%~dp0.." src config scripts tools
    scp %TEMP%\rescue-deploy.tar.gz %TARGET_USER%@%TARGET%:/tmp/
    ssh %TARGET_USER%@%TARGET% "mkdir -p %TARGET_PATH% && cd %TARGET_PATH% && tar xzf /tmp/rescue-deploy.tar.gz"
    del %TEMP%\rescue-deploy.tar.gz
)
echo   ✅ 代码已同步

echo [4/5] 远程重启服务...
ssh %TARGET_USER%@%TARGET% "cd %TARGET_PATH% && sudo systemctl restart rescue-robot 2>/dev/null || echo '请手动启动: cd %TARGET_PATH% && PYTHONPATH=src python3 -m rescue_robot.main'"
echo   ✅ 服务已重启

echo [5/5] 健康检查...
ssh %TARGET_USER%@%TARGET% "pgrep -f rescue_robot.main > /dev/null && echo '  ✅ 进程运行中' || echo '  ⚠️ 未运行'; [ -f %TARGET_PATH%/tools/hw_selftest.py ] && echo '  ✅ 自检程序已上车' || echo '  ⚠️  tools/hw_selftest.py 缺失'; [ -f %TARGET_PATH%/config/robot.default.yaml ] && echo '  ✅ 现场配置已上车' || echo '  ⚠️  config/robot.default.yaml 缺失'"
echo.
echo ============================================
echo   部署完成!
echo ============================================
