# Agent Note: Localization-independent Nav2 startup

Status: implemented

## Problem

GICP 没有全局重定位能力，机器人不在先验地图初始位姿时需要操作员通过 Web 页面发布 `/initialpose`。navigation 启动链曾把 `/localization` 作为 Nav2 的启动条件，而 GICP 只在首次接受配准后创建该 topic；未在 60 秒内完成人工定位会终止整个 launch。

## Decision

navigation 模式保留 FAST-LIO `/Odometry` 与 `/cloud_registered_body` 的数据就绪闸门。闸门通过后立即启动 GICP，并通过 12 秒固定 `TimerAction` 错峰启动 Nav2。Nav2 进程创建不依赖 `/localization` 或 `/base_controller/odom` 的 topic discovery。

GICP 仍只在首次接受配准后发布 `/localization`，fitness 门限保持不变。Web UI 在上层数据出现前启动，允许未定位时发布 `/initialpose`，并在定位 telemetry 可用前拒绝 Web 导航目标。

## Alternatives considered

**等待 `/localization` 后启动 Nav2。** 该顺序把人工定位放在限时启动闸门内；操作员未及时设置初始位姿会中止完整 launch。

**在 GICP 构造时创建 `/localization`。** topic 存在不能证明地图匹配有效，并会让依赖该信号的消费者把默认变换当作有效定位。

**降低 fitness 门限。** 门限不解决未知初始位姿，且可能接受错误配准；启动顺序不改变定位质量策略。

**同时启动 GICP 与 Nav2。** 固定错峰保留原有的资源启动节奏，同时不依赖定位完成时间。

## Consequences

操作员可以在完整 navigation 栈保持运行时设置初始位姿。首次有效定位前 Nav2 可能报告暂时缺少地图坐标变换，Web UI 显示定位不可用并拒绝导航目标；有效定位到达后无需重启节点。

静态启动链测试固定只有 FAST-LIO 数据 gate，并验证其后是 GICP 和延迟 12 秒的 Nav2。动态定位与导航行为仍需在相应安全验收范围内验证。
