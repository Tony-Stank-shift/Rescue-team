"""
hardware —— 硬件抽象层

提供平台无关的硬件接口：
  - Button:   一键启动按钮（Mock / GPIO）
  - Indicator: LED 状态指示器（Mock / GPIO）—— 本车未装蜂鸣器
"""

from .button import MockButton, GPIOButton, ButtonEvent, AbstractButton
from .indicator import MockIndicator, LEDIndicator, IndicatorSignal, AbstractIndicator

__all__ = [
    "MockButton", "GPIOButton", "ButtonEvent", "AbstractButton",
    "MockIndicator", "LEDIndicator", "IndicatorSignal", "AbstractIndicator",
]
