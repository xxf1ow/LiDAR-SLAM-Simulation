# Agent Note: Voice assistant recovery state

Status: implemented

## Problem

`/api/assistant-state` 面向只监听统一 `recovering` 状态的语音助手，而底层导航状态区分旋转、等待、后退和清理代价地图等细粒度阶段。若助手接口直接暴露这些阶段，恢复行为不会触发恢复播报。

## Decision

`WebUiNode.assistant_state()` 将所有已识别的导航恢复阶段聚合为 `recovering`。`/api/navigation-state` 保留细粒度阶段，浏览器界面仍可显示具体恢复行为；语音助手接口只提供播报所需的稳定语义。

恢复阶段清单由行为树节点映射派生。新增恢复节点时，将节点加入现有恢复节点清单即可同时获得导航优先级和助手状态聚合。

## Alternatives considered

**由语音客户端识别每个细粒度阶段。** 这种方式会把 Nav2 行为树细节扩散到客户端，并且新增恢复行为时容易再次漏播。

**同时聚合 `/api/navigation-state`。** 这种方式会丢失浏览器状态栏用于解释当前恢复行为的细节。

## Consequences

语音客户端只需监听 `recovering`，恢复行为的内部切换不会改变助手状态。需要具体恢复阶段的消费者使用 `/api/navigation-state`；`/api/assistant-state` 不提供细粒度恢复值。参数化单元测试覆盖全部已识别的恢复阶段，并保留底层阶段优先级测试。
