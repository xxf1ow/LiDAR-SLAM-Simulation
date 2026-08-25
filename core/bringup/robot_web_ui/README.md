# Robot Web UI HTTP 接口

同机程序通过 `http://127.0.0.1:<port>` 访问接口。下述接口使用 UTF-8 JSON；失败响应格式为 `{"error":"..."}`。

## 语音助手状态

```http
GET /api/assistant-state
```

成功返回 `200`，并禁用缓存：

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
- `distance_m`：剩余距离，单位为米；不可用时为 `null`。
- `issue`：当前最重要的异常，或 `null`。优先级依次为 `feedback_unavailable`、`map_unavailable`、`localization_unavailable`、`navigation_unavailable`。

## 停车点

### 获取停车点

```http
GET /api/parking-points
```

成功返回 `200` 和有序的 `points` 数组。语音接入字段为：

- `number`：停车点显示编号。
- `name`：停车点名称，也是保存和导航请求使用的标识。

### 保存当前位置

```http
POST /api/parking-points/save
Content-Type: application/json

{"name":"充电区"}
```

成功返回 `201`；响应中的 `point.number` 和 `point.name` 标识保存后的停车点。名称或请求格式错误返回 `400`，名称重复或控制模式冲突返回 `409`，定位或存储服务不可用返回 `503`。

### 前往停车点

```http
POST /api/parking-points/navigate
Content-Type: application/json

{"name":"充电区"}
```

成功返回 `202`：

```json
{"name":"充电区","status":"accepted"}
```

停车点不存在返回 `404`，导航任务冲突返回 `409`，地图、定位或导航服务不可用返回 `503`。
