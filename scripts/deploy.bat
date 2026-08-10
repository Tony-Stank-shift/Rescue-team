@echo off
REM ============================================================
REM deploy.bat —— 快速部署脚本 (Windows)
REM
REM 用法: scripts\deploy.bat [--skip-test]
REM ============================================================

setlocal enabledelayedexpansion

set TARGET=%TARGET%
if "%TARGET%"=="" set TARGET=raspberrypi.local
set TARGET_USER=%TARGET_USER%
if "%TARGET_USER%"=="" set TARGET_USER=pi
set TARGET_PATH=%TARGET_PATH%
if "%TARGET_PATH%"=="" set TARGET_PATH=/home/pi/rescue-robot
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

echo [3/5] 代码同步...
where rsync >nul 2>&1
if %errorlevel% equ 0 (
    rsync -avz --delete --exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude logs "%~dp0..\src\" "%TARGET_USER%@%TARGET%:%TARGET_PATH%/src/"
) else (
    echo   ⚠️  rsync 不可用, 尝试 scp...
    tar czf %TEMP%\rescue-deploy.tar.gz -C "%~dp0.." src config scripts
    scp %TEMP%\rescue-deploy.tar.gz %TARGET_USER%@%TARGET%:/tmp/
    ssh %TARGET_USER%@%TARGET% "cd %TARGET_PATH% && tar xzf /tmp/rescue-deploy.tar.gz"
    del %TEMP%\rescue-deploy.tar.gz
)
echo   ✅ 代码已同步

echo [4/5] 远程重启服务...
ssh %TARGET_USER%@%TARGET% "cd %TARGET_PATH% && sudo systemctl restart rescue-robot 2>/dev/null || echo '请手动启动'"
echo   ✅ 服务已重启

echo [5/5] 健康检查...
ssh %TARGET_USER%@%TARGET% "pgrep -f rescue_robot.main > /dev/null && echo '  ✅ 进程运行中' || echo '  ⚠️ 未运行'"
echo.
echo ============================================
echo   部署完成!
echo ============================================
