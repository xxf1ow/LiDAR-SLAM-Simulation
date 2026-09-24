# Robot Web UI 外部 HTTP 接口

本参考描述 C++ `robot_web_ui` 节点向语音助手等同机客户端提供的稳定 HTTP 合同。该节点与 cpp-httplib 服务运行在一个进程中；浏览器内部路由由 Web UI 实现和测试拥有，不在此手工列出。客户端通过 `http://127.0.0.1:<port>` 访问，JSON 使用 UTF-8，失败响应为 `{"error":"..."}`。

正式 bringup 从生成的参数文件启动 `lib/robot_web_ui/robot_web_ui`。`navigation_sources_enabled` 在 navigation 模式为 `true`，在 mapping 模式为 `false`；mapping 模式保留人工控制、接管和恢复自动模式接口，但导航数据、初始位姿、保存当前位置和发起导航操作不可用。静态页面安装在 `share/robot_web_ui/web/`。

浏览器显示手动控制、自动导航、目标跟踪三种页面模式。手动页的“选择模式”按钮打开只含“自动导航”和“目标跟踪”的抽屉；后两种页面的主按钮均返回手动控制。跟踪页显示 `base_footprint` 单环回波和参考跟踪器最后输出，可拖动、缩放、适配、居中；双击视图按当前视图坐标提交搜索中心。该页不显示手动方向、导航或停车控件，刷新页面不重置节点目标。跟踪页面保持 gate 为 `manual`，页面不发送导航目标或跟踪速度；自动导航返回手动控制会调用 gate 接管服务，但不会取消已有 Nav2 goal。

## 跟踪观测与选择

`GET /api/tracking-state` 返回最近一次 `/tracking/state` JSON；收到状态前返回 `{"available":false}`。有效状态包含 `frame_id`、`stamp`（`sec`、`nanosec`）、`active`、`target` 和 `points`；`points` 为 `[x,y]` 米坐标，`target` 在尚未选择时为 `null`，否则为参考跟踪器最后输出，即使当前帧无匹配回波也可能保留旧位置。状态无自动失联判定。

`POST /api/tracking-target` 接受且仅接受 `{"x":1.0,"y":0.0}` 形式的有限数值，单位米、坐标系为 `base_footprint`。格式或数值错误返回 `400`，无跟踪目标订阅者或发布失败返回 `503`，发布成功返回 `202` 与 `{"ok":true}`。`202` 表示消息已发布，不确认跟踪器收到或匹配目标。

## 助手状态

```http
GET /api/assistant-state
```

成功返回 `200` 且禁用缓存：

```json
{
  "mode": "automatic",
  "navigation": "navigating",
  "distance_m": 3.8,
  "issue": null
}
```

- `mode`：`automatic`、`manual` 或 `unknown`。
- `navigation`：`idle`、`sending`、`navigating`、`canceling`、`succeeded`、`canceled` 或 `failed`。
- `distance_m`：剩余距离（米），不可用时为 `null`。
- `issue`：最重要的当前异常或 `null`；优先级依次为 `map_unavailable`、`localization_unavailable`、`navigation_unavailable`。

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
