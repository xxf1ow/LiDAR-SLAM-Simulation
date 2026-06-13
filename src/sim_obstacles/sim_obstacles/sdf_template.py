"""SDF 模板渲染（纯文本替换；模板唯一占位符为 @NAME@）。"""


def render_sdf(template: str, name: str) -> str:
    """把障碍名注入模板（model name 与 ROS namespace）。"""
    return template.replace("@NAME@", name)
