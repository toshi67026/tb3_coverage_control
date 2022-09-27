#!/usr/bin/python3

from enum import Enum

# 各種描画に使う色を定義
color_list = ["r", "b", "y", "m", "c", "g"]


class GeometricPattern(Enum):
    """幾何的な重要度分布配置"""

    NONE = "none"
    LINE = "line"
    GAUSS = "gauss"
    CIRCLE = "circle"
