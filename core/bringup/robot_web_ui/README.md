# Robot Web UI 外部 HTTP 接口

本参考只描述语音助手等同机客户端使用的稳定 HTTP 合同。浏览器内部路由由 Web UI 实现和测试拥有，不在此手工列出。客户端通过 `http://127.0.0.1:<port>` 访问，JSON 使用 UTF-8，失败响应为 `{"error":"..."}`。

## 助手状态

```http
GET /api/assistant-state
```

成功返回 `200` 且禁用缓存：

```json
{
  "mode": "automatic",
  "navigation": "following",
  "distance_m": 3.8,
  "issue": null
}
```

- `mode`：`automatic`、`manual` 或 `unknown`。
- `navigation`：`idle`、`sending`、`navigating`、`planning`、`following`、`canceling`、`succeeded`、`canceled`、`failed`，或恢复阶段 `clearing_global_plan`、`clearing_local_control`、`clearing_global_recovery`、`clearing_local_recovery`、`spinning`、`waiting`、`backing_up`。
- `distance_m`：剩余距离（米），不可用时为 `null`。
- `issue`：最重要的当前异常或 `null`；优先级依次为 `feedback_unavailable`、`map_unavailable`、`localization_unavailable`、`navigation_unavailable`。

## 停车点

`GET /api/parking-points` 成功返回 `200` 和有序 `points` 数组。`number` 是显示编号，`name` 是保存和导航请求使用的唯一标识。

保存当前位置：

```http
POST /api/parking-points/save
Content-Type: application/json

{"name":"充电区"}
```

成功返回 `201`，响应中的 `point.number` 和 `point.name` 标识保存后的停车点。名称或请求格式错误返回 `400`，名称重复或控制模式冲突返回 `409`，定位或存储不可用返回 `503`。

前往停车点：

```http
POST /api/parking-points/navigate
Content-Type: application/json

{"name":"充电区"}
```

成功返回 `202` 和 `{"name":"充电区","status":"accepted"}`。停车点不存在返回 `404`，任务冲突返回 `409`，地图、定位或导航不可用返回 `503`。
