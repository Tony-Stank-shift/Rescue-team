"""
hw_selftest —— 智能救援机器人「分部实机自检」

用途：**比赛现场跑实机时，一次运行就能指出哪一个具体板块出了问题**。
每个模块独立一个文件 ``m_<name>.py``，暴露 ``run(ctx) -> Result``；
统一输出 ``[PASS]/[FAIL]/[SKIP]`` + 一句人话结论 + 关键证据。

入口::

    PYTHONPATH=src python3 tools/hw_selftest.py            # 跑全部（无硬件自动 SKIP）
    PYTHONPATH=src python3 tools/hw_selftest.py --list     # 列出模块
    PYTHONPATH=src python3 tools/hw_selftest.py --only servo
    PYTHONPATH=src python3 tools/hw_selftest.py --yes-motion   # 允许驱动电机（先架起轮子）
"""

__all__ = ["framework"]
