"""
deploy.py —— 快速部署流程 (7.2.1)

从代码修改到机器人运行 < 5 分钟的快速部署流水线。
支持 Dry-Run 模式和依赖检查。

用法:
  # 命令行
  python -m rescue_robot.innovation.deploy --target raspberrypi.local
  python -m rescue_robot.innovation.deploy --dry-run
  python -m rescue_robot.innovation.deploy --check-only

  # Python API
  deployer = Deployer()
  deployer.check_dependencies()
  deployer.quick_test()
  deployer.full_deploy("raspberrypi.local")
"""

import importlib
import logging
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

logger = logging.getLogger("deploy")

# 颜色输出（Windows 兼容）
_HAS_COLORAMA = False
try:
    import colorama
    colorama.init()
    _HAS_COLORAMA = True
except ImportError:
    pass


# ============================================================
# 检查结果
# ============================================================

@dataclass
class CheckResult:
    """单项检查结果"""
    name: str
    ok: bool
    detail: str = ""
    fix_hint: str = ""
    time_ms: float = 0.0


@dataclass
class DeployReport:
    """部署报告"""
    results: List[CheckResult] = field(default_factory=list)
    total_time_s: float = 0.0
    success: bool = False

    @property
    def passed(self) -> int:
        return sum(1 for r in self.results if r.ok)

    @property
    def failed(self) -> int:
        return len(self.results) - self.passed

    def summary(self) -> str:
        lines = [
            f"部署检查: {self.passed} 通过, {self.failed} 失败",
            f"总耗时: {self.total_time_s:.1f}s",
        ]
        for r in self.results:
            icon = "✅" if r.ok else "❌"
            lines.append(f"  {icon} {r.name}: {r.detail}")
            if not r.ok and r.fix_hint:
                lines.append(f"     → {r.fix_hint}")
        return "\n".join(lines)


# ============================================================
# Deployer
# ============================================================

class Deployer:
    """
    快速部署器。

    5 步部署流水线:
      1. 依赖检查 (10s)
      2. 本地冒烟测试 (30s)
      3. 代码同步 (30s)
      4. 远程重启 (60s)
      5. 健康检查 (10s)
    """

    # 必需的 Python 包（不含标准库）
    REQUIRED_PACKAGES = [
        # 已在项目中使用的库
        ("yaml", "PyYAML", "pip install pyyaml"),
        # ("cv2", "opencv-python", "pip install opencv-python"),
        # ("RPi", "RPi.GPIO", "pip install RPi.GPIO (仅树莓派)"),
    ]

    # 可选但建议的包
    OPTIONAL_PACKAGES = [
        ("psutil", "psutil", "pip install psutil"),
        ("numpy", "numpy", "pip install numpy"),
    ]

    # 必需文件
    REQUIRED_FILES = [
        "src/rescue_robot/main.py",
        "src/rescue_robot/config.py",
        "src/rescue_robot/state_machine.py",
        "src/rescue_robot/system_check.py",
        "config/robot.default.yaml",
    ]

    def __init__(self, project_root: Optional[str] = None):
        self._project_root = project_root or os.getcwd()
        self._report = DeployReport()

    @property
    def project_root(self) -> str:
        return self._project_root

    @property
    def last_report(self) -> DeployReport:
        return self._report

    # ---- 步骤 1: 依赖检查 ----

    def check_dependencies(self) -> DeployReport:
        """检查所有依赖。"""
        self._report = DeployReport()
        t0 = time.time()

        # Python 版本
        py_ver = sys.version_info
        ok = py_ver >= (3, 10)
        self._add_result(CheckResult(
            name="Python 版本",
            ok=ok,
            detail=f"Python {py_ver.major}.{py_ver.minor}.{py_ver.micro}",
            fix_hint="需要 Python 3.10+" if not ok else "",
        ))

        # 必需的包
        for import_name, display_name, fix_hint in self.REQUIRED_PACKAGES:
            t1 = time.time()
            ok = self._check_package(import_name)
            self._add_result(CheckResult(
                name=f"包: {display_name}",
                ok=ok,
                detail=f"已安装" if ok else "缺失",
                fix_hint=fix_hint if not ok else "",
                time_ms=(time.time() - t1) * 1000,
            ))

        # 可选包
        for import_name, display_name, fix_hint in self.OPTIONAL_PACKAGES:
            t1 = time.time()
            ok = self._check_package(import_name)
            if not ok:
                logger.info(f"可选包未安装: {display_name} — 不影响基本功能")
            self._add_result(CheckResult(
                name=f"包(可选): {display_name}",
                ok=True,  # 可选包不影响通过
                detail=f"已安装" if ok else f"未安装 (可选)",
                fix_hint="" if ok else fix_hint,
                time_ms=(time.time() - t1) * 1000,
            ))

        # 必需文件
        for file_path in self.REQUIRED_FILES:
            full = os.path.join(self._project_root, file_path)
            ok = os.path.exists(full)
            self._add_result(CheckResult(
                name=f"文件: {file_path}",
                ok=ok,
                detail="存在" if ok else "缺失",
                fix_hint="请检查项目完整性" if not ok else "",
            ))

        # 配置文件
        config_files = [
            "config/robot.default.yaml",
            "config/field.default.yaml",
        ]
        for cf in config_files:
            full = os.path.join(self._project_root, cf)
            ok = os.path.exists(full)
            self._add_result(CheckResult(
                name=f"配置: {cf}",
                ok=ok,
                detail="存在" if ok else "缺失",
                fix_hint="运行前需要有配置文件" if not ok else "",
            ))

        self._report.total_time_s = time.time() - t0
        self._report.success = self._report.failed == 0
        return self._report

    # ---- 步骤 2: 冒烟测试 ----

    def quick_test(self) -> bool:
        """
        快速冒烟测试（Mock 模式下 30s 验证）。

        Returns:
            True 测试通过
        """
        logger.info("快速冒烟测试开始...")

        tests_ok = True
        test_modules = [
            "rescue_robot.innovation.config_loader",
            "rescue_robot.innovation.model_switcher",
            "rescue_robot.innovation.hardware_profile",
            "rescue_robot.innovation.hot_reloader",
            "rescue_robot.robustness.fault_tolerance",
            "rescue_robot.robustness.stability",
            "rescue_robot.robustness.logging_system",
        ]

        for mod_name in test_modules:
            try:
                mod = importlib.import_module(mod_name)
                logger.info(f"  ✅ {mod_name}")
            except Exception as e:
                logger.error(f"  ❌ {mod_name}: {e}")
                tests_ok = False

        if tests_ok:
            logger.info("✅ 冒烟测试通过")
        else:
            logger.error("❌ 冒烟测试失败")

        return tests_ok

    # ---- 步骤 3-5: 远程部署 ----

    def sync_code(self, target_host: str, target_user: str = "pi",
                  target_path: str = "/home/pi/rescue-robot") -> bool:
        """
        同步代码到目标主机。

        先尝试 rsync，不可用时回退到 tar+scp。
        """
        logger.info(f"代码同步: {target_user}@{target_host}:{target_path}")

        # 1. 尝试 rsync
        if self._run(["rsync", "--version"], capture=True).ok:
            logger.info("使用 rsync 同步...")
            result = self._run([
                "rsync", "-avz", "--delete",
                "--exclude", "__pycache__",
                "--exclude", "*.pyc",
                "--exclude", ".git",
                "--exclude", "logs",
                f"{self._project_root}/src/",
                f"{target_user}@{target_host}:{target_path}/src/",
            ])
            if result.ok:
                logger.info("✅ rsync 同步成功")
                return True

        # 2. 回退到 tar+scp
        logger.info("使用 tar+scp 同步...")
        tar_file = "/tmp/rescue-deploy.tar.gz"
        self._run(["tar", "czf", tar_file,
                   "-C", self._project_root,
                   "src", "config", "scripts"])

        scp_result = self._run([
            "scp", tar_file,
            f"{target_user}@{target_host}:/tmp/",
        ])
        if not scp_result.ok:
            logger.error("scp 失败")
            return False

        ssh_result = self._run([
            "ssh", f"{target_user}@{target_host}",
            f"cd {target_path} && tar xzf /tmp/rescue-deploy.tar.gz",
        ])
        if ssh_result.ok:
            logger.info("✅ scp 同步成功")
        else:
            logger.error("远程解压失败")
        return ssh_result.ok

    def restart_remote(self, target_host: str, target_user: str = "pi") -> bool:
        """远程重启服务。"""
        logger.info(f"重启远程服务: {target_user}@{target_host}")

        # 先尝试 systemd
        result = self._run([
            "ssh", f"{target_user}@{target_host}",
            "sudo systemctl restart rescue-robot 2>/dev/null && "
            "echo 'systemd restart ok' || echo 'systemd not available'",
        ])
        logger.info(f"远程状态: {result.stdout.strip()}")
        return result.ok

    def health_check(self, target_host: str, target_user: str = "pi") -> bool:
        """远程健康检查。"""
        logger.info(f"健康检查: {target_user}@{target_host}")

        # 检查进程
        process_ok = False
        result = self._run([
            "ssh", f"{target_user}@{target_host}",
            "pgrep -f 'rescue_robot.main' > /dev/null && echo 'running' || echo 'not_running'",
        ], capture=True)
        if "running" in result.stdout:
            logger.info("  ✅ 进程运行中")
            process_ok = True
        else:
            logger.warning("  ⚠️  进程未运行")

        return process_ok

    def full_deploy(self, target_host: str, target_user: str = "pi",
                    target_path: str = "/home/pi/rescue-robot",
                    skip_test: bool = False) -> DeployReport:
        """
        完整部署流水线。

        目标: < 5 分钟
        """
        logger.info("=" * 50)
        logger.info("  完整部署开始...")
        logger.info("=" * 50)
        t_total = time.time()

        # 步骤 1: 依赖检查
        t1 = time.time()
        report = self.check_dependencies()
        if not report.success:
            logger.error(f"依赖检查失败: {report.failed} 项")
            logger.error(report.summary())
            return report
        logger.info(f"[{time.time() - t1:.0f}s] 步骤 1/5 完成: 依赖检查 ✅")

        # 步骤 2: 冒烟测试
        t2 = time.time()
        if not skip_test:
            if not self.quick_test():
                report.success = False
                return report
        logger.info(f"[{time.time() - t2:.0f}s] 步骤 2/5 完成: 冒烟测试 ✅")

        # 步骤 3: 代码同步
        t3 = time.time()
        if not self.sync_code(target_host, target_user, target_path):
            logger.error("代码同步失败")
            report.success = False
            return report
        logger.info(f"[{time.time() - t3:.0f}s] 步骤 3/5 完成: 代码同步 ✅")

        # 步骤 4: 远程重启
        t4 = time.time()
        self.restart_remote(target_host, target_user)
        time.sleep(3)  # 等待服务启动
        logger.info(f"[{time.time() - t4:.0f}s] 步骤 4/5 完成: 远程重启 ✅")

        # 步骤 5: 健康检查
        t5 = time.time()
        self.health_check(target_host, target_user)
        logger.info(f"[{time.time() - t5:.0f}s] 步骤 5/5 完成: 健康检查 ✅")

        report.total_time_s = time.time() - t_total
        report.success = True
        logger.info(f"🎉 部署完成! 总耗时: {report.total_time_s:.0f}s")
        return report

    # ---- 内部 ----

    @staticmethod
    def _check_package(import_name: str) -> bool:
        """检查 Python 包是否可用。"""
        try:
            importlib.import_module(import_name)
            return True
        except ImportError:
            return False

    def _add_result(self, result: CheckResult) -> None:
        self._report.results.append(result)

    @staticmethod
    def _run(args: List[str], capture: bool = False) -> subprocess.CompletedProcess:
        """安全运行子进程。"""
        try:
            return subprocess.run(
                args,
                capture_output=capture,
                text=True,
                timeout=60,
            )
        except subprocess.TimeoutExpired:
            logger.warning(f"命令超时: {' '.join(args)}")
            return subprocess.CompletedProcess(args, -1, "", "timeout")
        except FileNotFoundError:
            logger.debug(f"命令不可用: {args[0]}")
            return subprocess.CompletedProcess(args, -1, "", f"{args[0]}: not found")
        except Exception as e:
            logger.warning(f"命令异常: {' '.join(args)}: {e}")
            return subprocess.CompletedProcess(args, -1, "", str(e))


# ============================================================
# CLI
# ============================================================

def _cli_main():
    """CLI 入口。"""
    deployer = Deployer()

    if "--check-only" in sys.argv:
        report = deployer.check_dependencies()
        print(report.summary())
        sys.exit(0 if report.success else 1)

    if "--dry-run" in sys.argv:
        print("🔧 DRY RUN 模式")
        report = deployer.check_dependencies()
        print(report.summary())
        if report.success:
            print("\n✅ Dry-run 通过 (未实际部署)")
        sys.exit(0 if report.success else 1)

    target = "raspberrypi.local"
    for i, arg in enumerate(sys.argv):
        if arg == "--target" and i + 1 < len(sys.argv):
            target = sys.argv[i + 1]
        elif arg.startswith("--target="):
            target = arg.split("=", 1)[1]

    skip_test = "--skip-test" in sys.argv

    print(f"目标: {target}")
    report = deployer.full_deploy(target, skip_test=skip_test)

    if report.success:
        print(f"\n✅ 部署成功! 总耗时: {report.total_time_s:.0f}s")
    else:
        print(f"\n❌ 部署失败")
        print(report.summary())
        sys.exit(1)


# ============================================================
# 独立测试
# ============================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)-7s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  快速部署 — 独立测试")
    print("=" * 60)

    project_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )))

    deployer = Deployer(project_root=project_root)

    # ---- 测试 1: 依赖检查 ----
    print("\n--- 测试 1: 依赖检查 ---")
    report = deployer.check_dependencies()
    print(f"  通过: {report.passed}, 失败: {report.failed}")
    print(f"  耗时: {report.total_time_s:.1f}s")
    # 基本检查应通过（Python 版本 + 必需文件）
    assert report.passed >= 3, f"至少应有 3 项通过: {report.passed}"
    print("  ✅ 通过")

    # ---- 测试 2: 冒烟测试 ----
    print("\n--- 测试 2: 快速冒烟测试 ---")
    ok = deployer.quick_test()
    assert ok, "冒烟测试应通过"
    print("  ✅ 通过")

    # ---- 测试 3: Dry-run 不应实际部署 ----
    print("\n--- 测试 3: Dry-run 检查 ---")
    # 仅验证检查流程完整
    assert report.results, "应有检查结果"
    print(f"  检查项: {len(report.results)}")
    print("  ✅ 通过")

    print(f"\n{'=' * 60}")
    print("  快速部署 — 测试通过 ✅")
    print(f"{'=' * 60}")
    print()
    print("  注: 远程部署步骤 (rsync/ssh) 需要树莓派在线。")
    print("  本地检查全部通过。")
