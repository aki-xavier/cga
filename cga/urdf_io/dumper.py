"""CRDF 的 YAML 输出风格 (短标量列表流式, 其余块式)。"""

import yaml


class Dumper(yaml.SafeDumper):
    """SafeDumper + 短标量列表 → 流式 [..] 的 representer (可读性)。"""

    @staticmethod
    def smart_seq(dumper: yaml.SafeDumper, data: list) -> yaml.Node:
        """短标量列表 → 流式 [..], 其余 (几何数组等) → 块式。"""
        flow = len(data) <= 4 and all(isinstance(x, (int, float, str)) for x in data)
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)


Dumper.add_representer(list, Dumper.smart_seq)
