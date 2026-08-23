(function () {
  "use strict";

  const LAYERS = {
    static: "/api/map/static",
    global_costmap: "/api/map/global-costmap",
    local_costmap: "/api/map/local-costmap",
    path: "/api/navigation-path"
  };
  const UNKNOWN_COLOR = [112, 137, 134, 255];
  const GRID_ALPHA = {
    static: 1,
    global_costmap: 0.45,
    local_costmap: 0.70
  };
  const PALETTE = {
    halo: "#111827",
    path: "#55ffff",
    robot: "#ffd400",
    placement: "#ff8a00",
    parking: "#7c3aed"
  };
  const MIN_SCALE = 0.25;
  const MAX_SCALE = 128;
  const PLACEMENT_ALPHA = 0.90;
  const PLACEMENT_ARROW_LENGTH = 36;
  const PLACEMENT_TIP_HIT_RADIUS = 22;
  const ACTIVE_GOAL_STATUSES = new Set(["sending", "navigating", "canceling"]);
  const NAVIGATION_PHASE_LABELS = Object.freeze({
    planning: "正在规划路径",
    following: "正在跟踪路径",
    clearing_global_plan: "路径规划受阻，正在清理全局代价地图",
    clearing_local_control: "路径跟踪受阻，正在清理局部代价地图",
    clearing_global_recovery: "导航恢复：正在清理全局代价地图",
    clearing_local_recovery: "导航恢复：正在清理局部代价地图",
    spinning: "导航恢复：正在原地旋转",
    waiting: "导航恢复：正在等待障碍消退",
    backing_up: "导航恢复：正在后退"
  });

  function gridToWorld(info, column, row) {
    const localX = (column + 0.5) * info.resolution;
    const localY = (row + 0.5) * info.resolution;
    const [originX, originY, yaw] = info.origin;
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    return {
      x: originX + localX * cosine - localY * sine,
      y: originY + localX * sine + localY * cosine
    };
  }

  function worldToGrid(info, x, y) {
    const [originX, originY, yaw] = info.origin;
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    const dx = x - originX;
    const dy = y - originY;
    const localX = dx * cosine + dy * sine;
    const localY = -dx * sine + dy * cosine;
    return {
      column: Math.floor(localX / info.resolution),
      row: Math.floor(localY / info.resolution)
    };
  }

  function validLocalAffine(info) {
    return info.transform_available === true
      && Array.isArray(info.map_from_source)
      && info.map_from_source.length === 3
      && info.map_from_source.every(Number.isFinite);
  }

  function create(options) {
    const canvas = options.canvas;
    const context = canvas.getContext("2d");
    const buttons = options.buttons || {};
    const navigationButtons = options.navigationButtons || {};
    const cache = {};
    let latestState = {layers: {}};
    let inFlight = false;
    let timer = null;
    let dragging = null;
    let placement = null;
    let parkingPoints = [];
    let parkingPointsVisible = false;
    let parkingActionStates = null;
    let navigationRequestMessage = "";
    let cancelRequestPending = false;
    const transform = {scale: 16, x: 100, y: 50};
    let autoFit = true;
    let hasFitted = false;
    const controlDock = options.controlDock || null;

    function bounds() {
      const rect = canvas.getBoundingClientRect();
      return {width: rect.width || 1, height: rect.height || 1};
    }

    function viewportBounds() {
      const canvasRect = canvas.getBoundingClientRect();
      const full = {
        left: 0,
        top: 0,
        width: canvasRect.width || 1,
        height: canvasRect.height || 1
      };
      if (!controlDock) return full;
      const dockRect = controlDock.getBoundingClientRect();
      const height = dockRect.top - canvasRect.top;
      if (!Number.isFinite(height) || height <= 0 || height > full.height) {
        return full;
      }
      return {...full, height};
    }

    function resize() {
      const rect = bounds();
      const ratio = globalThis.devicePixelRatio || 1;
      const width = Math.round(rect.width * ratio);
      const height = Math.round(rect.height * ratio);
      if (canvas.width !== width) canvas.width = width;
      if (canvas.height !== height) canvas.height = height;
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      return rect;
    }

    function cellRgba(name, cell) {
      if (cell === 255) return UNKNOWN_COLOR;
      if (name === "static") {
        const occupancy = Math.max(0, Math.min(100, cell));
        const value = 255 - Math.floor(255 * occupancy / 100);
        return [value, value, value, 255];
      }
      if (cell === 0) return [0, 0, 0, 0];
      if (cell === 99) return [0, 255, 255, 255];
      if (cell === 100) return [255, 0, 255, 255];
      const cost = Math.max(1, Math.min(98, cell));
      const red = Math.floor(255 * cost / 100);
      return [red, 0, 255 - red, 255];
    }

    function buildGridRaster(name, info, cells) {
      const raster = document.createElement("canvas");
      raster.width = info.width;
      raster.height = info.height;
      const rasterContext = raster.getContext("2d");
      const imageData = rasterContext.createImageData(info.width, info.height);
      for (let index = 0; index < cells.length; index += 1) {
        imageData.data.set(cellRgba(name, cells[index]), index * 4);
      }
      rasterContext.putImageData(imageData, 0, 0);
      return raster;
    }

    function effectiveGridInfo(name, info) {
      const layer = cache[name];
      if (!layer) return null;
      if (
        info
        && layer.revision === info.revision
        && layer.etag === info.etag
      ) return info;
      return layer.info;
    }

    function drawGrid(name, info) {
      const layer = cache[name];
      const drawInfo = effectiveGridInfo(name, info);
      if (!layer || !drawInfo || !layer.raster) return;
      if (name === "local_costmap" && !validLocalAffine(drawInfo)) return;
      context.save();
      if (name === "local_costmap") {
        context.translate(
          drawInfo.map_from_source[0], drawInfo.map_from_source[1]
        );
        context.rotate(drawInfo.map_from_source[2]);
      }
      context.translate(drawInfo.origin[0], drawInfo.origin[1]);
      context.rotate(drawInfo.origin[2]);
      context.scale(drawInfo.resolution, drawInfo.resolution);
      context.globalAlpha = GRID_ALPHA[name];
      context.drawImage(
        layer.raster, 0, 0, drawInfo.width, drawInfo.height
      );
      context.restore();
    }

    function drawPath(info) {
      const layer = cache.path;
      if (!layer || !info || !layer.cells || layer.cells.byteLength < 8) return;
      const data = new DataView(
        layer.cells.buffer,
        layer.cells.byteOffset,
        layer.cells.byteLength
      );
      context.beginPath();
      for (let offset = 0; offset + 7 < data.byteLength; offset += 8) {
        const x = data.getFloat32(offset, true);
        const y = data.getFloat32(offset + 4, true);
        if (offset === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      }
      context.strokeStyle = PALETTE.halo;
      context.lineWidth = 5 / transform.scale;
      context.stroke();
      context.strokeStyle = PALETTE.path;
      context.lineWidth = 2.5 / transform.scale;
      context.stroke();
    }

    function drawRobot() {
      const pose = latestState.localization;
      if (!pose || !Number.isFinite(pose.x) || !Number.isFinite(pose.y)) return;
      context.save();
      context.translate(pose.x, pose.y);
      context.rotate(pose.yaw || 0);
      context.fillStyle = PALETTE.robot;
      context.beginPath();
      context.moveTo(0.32, 0);
      context.lineTo(-0.2, 0.2);
      context.lineTo(-0.2, -0.2);
      context.closePath();
      context.fill();
      context.strokeStyle = PALETTE.halo;
      context.lineWidth = 2 / transform.scale;
      context.stroke();
      context.restore();
    }

    function placementAnchor() {
      const canvasRect = canvas.getBoundingClientRect();
      const statusRect = options.statusStrip
        ? options.statusStrip.getBoundingClientRect()
        : canvasRect;
      const lowerBoundary = controlDock
        ? controlDock.getBoundingClientRect().top
        : canvasRect.bottom;
      return {
        x: canvasRect.width / 2,
        y: (statusRect.bottom + lowerBoundary) / 2 - canvasRect.top
      };
    }

    function drawPlacement() {
      if (!placement) return;
      const anchor = placementAnchor();
      const tip = {
        x: anchor.x + Math.cos(placement.yaw) * PLACEMENT_ARROW_LENGTH,
        y: anchor.y - Math.sin(placement.yaw) * PLACEMENT_ARROW_LENGTH
      };
      const wingLength = 10;
      const wingAngle = Math.PI / 7;
      context.save();
      context.globalAlpha = PLACEMENT_ALPHA;
      context.beginPath();
      context.moveTo(anchor.x - 12, anchor.y);
      context.lineTo(anchor.x + 12, anchor.y);
      context.moveTo(anchor.x, anchor.y - 12);
      context.lineTo(anchor.x, anchor.y + 12);
      context.moveTo(anchor.x, anchor.y);
      context.lineTo(tip.x, tip.y);
      context.moveTo(tip.x, tip.y);
      context.lineTo(
        tip.x - Math.cos(placement.yaw - wingAngle) * wingLength,
        tip.y + Math.sin(placement.yaw - wingAngle) * wingLength
      );
      context.moveTo(tip.x, tip.y);
      context.lineTo(
        tip.x - Math.cos(placement.yaw + wingAngle) * wingLength,
        tip.y + Math.sin(placement.yaw + wingAngle) * wingLength
      );
      context.strokeStyle = PALETTE.halo;
      context.lineWidth = 5;
      context.stroke();
      context.strokeStyle = PALETTE.placement;
      context.lineWidth = 2.5;
      context.stroke();
      context.restore();
    }

    function drawParkingPoints() {
      if (!parkingPointsVisible) return;
      context.save();
      context.fillStyle = PALETTE.parking;
      context.strokeStyle = PALETTE.halo;
      context.lineWidth = 3;
      context.textAlign = "center";
      context.textBaseline = "middle";
      parkingPoints.forEach((point) => {
        const screenX = transform.x + point.x * transform.scale;
        const screenY = transform.y - point.y * transform.scale;
        context.beginPath();
        context.arc(screenX, screenY, 14, 0, Math.PI * 2);
        context.fill();
        context.fillStyle = PALETTE.halo;
        context.fillText(String(point.number), screenX, screenY);
        context.fillStyle = PALETTE.parking;
        context.beginPath();
        context.moveTo(screenX, screenY);
        context.lineTo(
          screenX + Math.cos(point.yaw) * 20,
          screenY - Math.sin(point.yaw) * 20
        );
        context.stroke();
      });
      context.restore();
    }

    function render() {
      const rect = resize();
      context.clearRect(0, 0, rect.width, rect.height);
      context.save();
      context.translate(transform.x, transform.y);
      context.scale(transform.scale, -transform.scale);
      const layers = latestState.layers || {};
      drawGrid("static", layers.static);
      drawGrid("global_costmap", layers.global_costmap);
      drawGrid("local_costmap", layers.local_costmap);
      drawPath(layers.path);
      drawRobot();
      context.restore();
      drawParkingPoints();
      drawPlacement();
    }

    async function updateLayer(name, info) {
      if (!info) {
        delete cache[name];
        return;
      }
      if (name === "local_costmap" && !validLocalAffine(info)) return;
      const current = cache[name];
      if (
        current
        && current.revision === info.revision
        && current.etag === info.etag
      ) return;
      const headers = current && current.etag ? {"If-None-Match": current.etag} : {};
      const response = await fetch(LAYERS[name], {headers});
      const responseEtag = response.headers.get("ETag");
      if (responseEtag !== info.etag) return;
      if (response.status === 304) {
        if (current && current.etag === responseEtag) {
          cache[name] = {
            ...current,
            revision: info.revision,
            info
          };
        }
        return;
      }
      if (!response.ok) return;
      const cells = new Uint8Array(await response.arrayBuffer());
      if (name === "path") {
        if (cells.byteLength % 8 !== 0) return;
      } else if (
        !Number.isInteger(info.width)
        || !Number.isInteger(info.height)
        || info.width <= 0
        || info.height <= 0
        || cells.byteLength !== info.width * info.height
      ) return;
      cache[name] = {
        revision: info.revision,
        etag: responseEtag,
        info,
        cells,
        raster: name === "path" ? null : buildGridRaster(name, info, cells)
      };
    }

    function mapUnavailable() {
      const layers = latestState.layers || {};
      return Boolean(latestState.map_error) || !layers.static;
    }

    function placementAvailable(mode) {
      const navigation = latestState.navigation || {};
      if (mapUnavailable()) return false;
      if (mode === "initial_pose") {
        return navigation.initial_pose_ready === true
          && !ACTIVE_GOAL_STATUSES.has(navigation.goal_status);
      }
      return mode === "navigation_goal"
        && navigation.action_server_ready === true
        && latestState.localized === true
        && latestState.gate_mode === "automatic"
        && !ACTIVE_GOAL_STATUSES.has(navigation.goal_status);
    }

    function navigationStateMessage() {
      const navigation = latestState.navigation || {};
      const message = typeof navigation.message === "string"
        ? navigation.message
        : "";
      let status = "";
      if (navigation.goal_status === "sending") status = "导航目标发送中";
      if (navigation.goal_status === "navigating") {
        status = Object.prototype.hasOwnProperty.call(
          NAVIGATION_PHASE_LABELS, navigation.phase
        )
          ? NAVIGATION_PHASE_LABELS[navigation.phase]
          : "导航中";
        if (Number.isFinite(navigation.distance_remaining)) {
          status += `，剩余 ${navigation.distance_remaining.toFixed(1)} 米`;
        }
      }
      if (navigation.goal_status === "canceling") status = "导航取消中";
      if (navigation.goal_status === "succeeded") status = "导航已到达目标";
      if (navigation.goal_status === "canceled") status = "导航已取消";
      if (navigation.goal_status === "failed") status = "导航失败";
      if (message && navigation.goal_status !== "idle") {
        status += `${status ? "：" : ""}${message}`;
      }
      return status;
    }

    function updateNavigationButtons() {
      const placing = placement !== null;
      if (Array.isArray(navigationButtons.parkingActions)) {
        if (placing) {
          navigationButtons.parkingActions.forEach((action) => {
            if (!action) return;
            action.hidden = true;
            action.disabled = true;
          });
        } else if (parkingActionStates !== null) {
          parkingActionStates.forEach((state) => {
            state.action.hidden = state.hidden;
            state.action.disabled = state.disabled;
          });
          parkingActionStates = null;
        }
      }
      const navigation = latestState.navigation || {};
      const requestPending = Boolean(
        navigationButtons.placementConfirm
        && navigationButtons.placementConfirm.dataset
        && navigationButtons.placementConfirm.dataset.requestPending === "true"
      );
      if (navigationButtons.initialPose) {
        navigationButtons.initialPose.hidden = placing;
        navigationButtons.initialPose.disabled = !placementAvailable("initial_pose");
      }
      if (navigationButtons.navigationAction) {
        const action = navigationButtons.navigationAction;
        action.hidden = placing;
        if (navigation.goal_status === "sending") {
          action.textContent = "目标发送中";
          action.disabled = true;
        } else if (navigation.goal_status === "navigating") {
          if (cancelRequestPending) {
            action.textContent = "取消中";
            action.disabled = true;
          } else if (navigation.cancel_available === true) {
            action.textContent = "取消导航";
            action.disabled = false;
          } else {
            action.textContent = "导航中";
            action.disabled = true;
          }
        } else if (navigation.goal_status === "canceling") {
          action.textContent = "取消中";
          action.disabled = true;
        } else {
          action.textContent = "设置导航目标";
          action.disabled = !placementAvailable("navigation_goal");
        }
        if (placing) action.disabled = true;
      }
      if (navigationButtons.placementConfirm) {
        navigationButtons.placementConfirm.hidden = !placing;
        navigationButtons.placementConfirm.disabled = placing
          ? requestPending || !placementAvailable(placement.mode)
          : true;
      }
      if (navigationButtons.placementCancel) {
        navigationButtons.placementCancel.hidden = !placing;
        navigationButtons.placementCancel.disabled = !placing || requestPending;
      }
    }

    function updateNavigationStatus() {
      const layers = latestState.layers || {};
      const unavailable = mapUnavailable();
      const messages = [];
      if (unavailable) {
        messages.push(latestState.map_error
          ? `地图不可用：${latestState.map_error}`
          : "地图不可用");
      }
      if (latestState.localization_error) {
        messages.push(`定位异常：${latestState.localization_error}`);
      } else if (!latestState.localization) {
        messages.push("等待定位");
      }
      if (latestState.path_error) {
        messages.push(`路径异常：${latestState.path_error}`);
      }
      const local = layers.local_costmap;
      if (local && local.transform_available === false) {
        messages.push(local.transform_error
          ? `局部代价地图不可用：${local.transform_error}`
          : "局部代价地图等待坐标变换");
      }
      const stateMessage = navigationStateMessage();
      if (stateMessage) messages.push(stateMessage);
      if (navigationRequestMessage) messages.push(navigationRequestMessage);
      if (options.navigationStatus) {
        options.navigationStatus.textContent = messages.join("；");
      }
      Object.values(buttons).forEach((button) => {
        if (button) button.disabled = unavailable;
      });
      updateNavigationButtons();
    }

    async function applyState(state) {
      latestState = state || {layers: {}};
      const goalStatus = (latestState.navigation || {}).goal_status;
      if (cancelRequestPending && goalStatus !== "navigating") {
        cancelRequestPending = false;
      }
      const layers = latestState.layers || {};
      for (const name of Object.keys(LAYERS)) await updateLayer(name, layers[name]);
      updateNavigationStatus();
      if (!hasFitted) fitToMap();
      render();
    }

    async function poll() {
      if (inFlight) return;
      inFlight = true;
      try {
        const response = await fetch("/api/navigation-state");
        if (response.ok) await applyState(await response.json());
      } finally {
        inFlight = false;
        if (options.poll !== false) {
          timer = setTimeout(poll, 200);
        }
      }
    }

    function zoom(multiplier) {
      autoFit = false;
      transform.scale = Math.max(
        MIN_SCALE,
        Math.min(MAX_SCALE, transform.scale * multiplier)
      );
      render();
    }

    function fitToMap() {
      const stateInfo = latestState.layers && latestState.layers.static;
      const info = effectiveGridInfo("static", stateInfo);
      if (!info) return false;
      const [originX, originY, yaw] = info.origin;
      const cosine = Math.cos(yaw);
      const sine = Math.sin(yaw);
      const mapWidth = info.width * info.resolution;
      const mapHeight = info.height * info.resolution;
      const corners = [
        [0, 0],
        [mapWidth, 0],
        [0, mapHeight],
        [mapWidth, mapHeight]
      ].map(([localX, localY]) => ({
        x: originX + localX * cosine - localY * sine,
        y: originY + localX * sine + localY * cosine
      }));
      const minX = Math.min(...corners.map((point) => point.x));
      const maxX = Math.max(...corners.map((point) => point.x));
      const minY = Math.min(...corners.map((point) => point.y));
      const maxY = Math.max(...corners.map((point) => point.y));
      const viewport = viewportBounds();
      transform.scale = Math.max(
        MIN_SCALE,
        Math.min(
          MAX_SCALE,
          0.9 * Math.min(
            viewport.width / (maxX - minX),
            viewport.height / (maxY - minY)
          )
        )
      );
      transform.x = viewport.left + viewport.width / 2
        - transform.scale * (minX + maxX) / 2;
      transform.y = viewport.top + viewport.height / 2
        + transform.scale * (minY + maxY) / 2;
      hasFitted = true;
      return true;
    }

    function fit() {
      if (!fitToMap()) return;
      autoFit = true;
      render();
    }

    function refreshViewport() {
      if (autoFit && hasFitted) fitToMap();
      render();
    }

    function centerRobot() {
      const pose = latestState.localization;
      if (!pose) return;
      const viewport = viewportBounds();
      transform.x = viewport.left + viewport.width / 2 - pose.x * transform.scale;
      transform.y = viewport.top + viewport.height / 2 + pose.y * transform.scale;
      autoFit = false;
      render();
    }

    function clearNavigationRequestMessage() {
      navigationRequestMessage = "";
    }

    function showNavigationRequestFailure(error) {
      const message = error && error.message ? error.message : String(error);
      navigationRequestMessage = `请求失败：${message}`;
      updateNavigationStatus();
    }

    function setParkingPoints(points) {
      parkingPoints = Array.isArray(points)
        ? points.map((point) => ({
          number: point.number,
          name: point.name,
          x: point.x,
          y: point.y,
          yaw: point.yaw
        }))
        : [];
      render();
    }

    function setParkingPointsVisible(visible) {
      parkingPointsVisible = visible === true;
      render();
    }

    function startPlacement(mode) {
      if (mode !== "initial_pose" && mode !== "navigation_goal") return;
      clearNavigationRequestMessage();
      if (
        parkingActionStates === null
        && Array.isArray(navigationButtons.parkingActions)
      ) {
        parkingActionStates = navigationButtons.parkingActions
          .filter((action) => action)
          .map((action) => ({
            action,
            hidden: action.hidden,
            disabled: action.disabled
          }));
      }
      const pose = latestState.localization;
      placement = {
        mode,
        yaw: pose && Number.isFinite(pose.yaw) ? pose.yaw : 0,
        rotatingPointerId: null
      };
      updateNavigationStatus();
      render();
    }

    function cancelPlacement() {
      clearNavigationRequestMessage();
      placement = null;
      updateNavigationStatus();
      render();
    }

    function getPlacementPreview() {
      if (!placement) return null;
      const anchor = placementAnchor();
      const staticLayer = latestState.layers && latestState.layers.static;
      return {
        x: (anchor.x - transform.x) / transform.scale,
        y: (transform.y - anchor.y) / transform.scale,
        yaw: placement.yaw,
        map_revision: staticLayer ? staticLayer.revision : null
      };
    }

    async function confirmPlacement() {
      if (!placement) return;
      const available = placementAvailable(placement.mode);
      const confirmButton = navigationButtons.placementConfirm;
      if (
        confirmButton
        && confirmButton.dataset
        && confirmButton.dataset.requestPending === "true"
      ) return;
      if (!available || typeof options.request !== "function") {
        navigationRequestMessage = "请求失败：当前导航操作不可用";
        updateNavigationStatus();
        return;
      }
      const mode = placement.mode;
      const preview = getPlacementPreview();
      const path = mode === "initial_pose"
        ? "/api/initial-pose"
        : "/api/navigation-goal";
      clearNavigationRequestMessage();
      updateNavigationStatus();
      if (confirmButton) {
        confirmButton.dataset.requestPending = "true";
        confirmButton.disabled = true;
      }
      if (navigationButtons.placementCancel) {
        navigationButtons.placementCancel.disabled = true;
      }
      try {
        await options.request(path, preview);
        if (confirmButton) delete confirmButton.dataset.requestPending;
        placement = null;
        clearNavigationRequestMessage();
        updateNavigationStatus();
        render();
      } catch (error) {
        if (confirmButton) delete confirmButton.dataset.requestPending;
        showNavigationRequestFailure(error);
      }
    }

    async function cancelNavigation() {
      if (cancelRequestPending) return;
      clearNavigationRequestMessage();
      updateNavigationStatus();
      if (typeof options.request !== "function") {
        navigationRequestMessage = "请求失败：当前导航操作不可用";
        updateNavigationStatus();
        return;
      }
      cancelRequestPending = true;
      updateNavigationButtons();
      try {
        await options.request("/api/navigation-cancel", {});
        clearNavigationRequestMessage();
        updateNavigationStatus();
      } catch (error) {
        cancelRequestPending = false;
        showNavigationRequestFailure(error);
      }
    }

    canvas.addEventListener("pointerdown", (event) => {
      if (placement) {
        if (
          placement.rotatingPointerId !== null
          && event.pointerId !== placement.rotatingPointerId
        ) return;
        const rect = canvas.getBoundingClientRect();
        const anchor = placementAnchor();
        const tipX = anchor.x + Math.cos(placement.yaw) * PLACEMENT_ARROW_LENGTH;
        const tipY = anchor.y - Math.sin(placement.yaw) * PLACEMENT_ARROW_LENGTH;
        const pointerX = event.clientX - rect.left;
        const pointerY = event.clientY - rect.top;
        if (Math.hypot(pointerX - tipX, pointerY - tipY) <= PLACEMENT_TIP_HIT_RADIUS) {
          placement.rotatingPointerId = event.pointerId;
          dragging = null;
          canvas.setPointerCapture(event.pointerId);
          return;
        }
        placement.rotatingPointerId = null;
      }
      dragging = {pointerId: event.pointerId, x: event.clientX, y: event.clientY};
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove", (event) => {
      if (placement && placement.rotatingPointerId !== null) {
        if (event.pointerId !== placement.rotatingPointerId) return;
        const rect = canvas.getBoundingClientRect();
        const anchor = placementAnchor();
        const pointerX = event.clientX - rect.left;
        const pointerY = event.clientY - rect.top;
        placement.yaw = Math.atan2(anchor.y - pointerY, pointerX - anchor.x);
        render();
        return;
      }
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      autoFit = false;
      transform.x += event.clientX - dragging.x;
      transform.y += event.clientY - dragging.y;
      dragging.x = event.clientX;
      dragging.y = event.clientY;
      render();
    });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => {
      canvas.addEventListener(name, (event) => {
        if (placement && event.pointerId === placement.rotatingPointerId) {
          placement.rotatingPointerId = null;
        }
        if (dragging && event.pointerId === dragging.pointerId) dragging = null;
      });
    });
    if (buttons.zoomIn) buttons.zoomIn.addEventListener("click", () => zoom(1.25));
    if (buttons.zoomOut) buttons.zoomOut.addEventListener("click", () => zoom(0.8));
    if (buttons.fit) buttons.fit.addEventListener("click", fit);
    if (buttons.centerRobot) buttons.centerRobot.addEventListener("click", centerRobot);
    if (navigationButtons.initialPose) {
      navigationButtons.initialPose.addEventListener("click", () => startPlacement("initial_pose"));
    }
    if (navigationButtons.navigationAction) {
      navigationButtons.navigationAction.addEventListener("click", () => {
        const navigation = latestState.navigation || {};
        if (
          navigation.goal_status === "navigating"
          && navigation.cancel_available === true
        ) {
          cancelNavigation();
        } else if (
          !ACTIVE_GOAL_STATUSES.has(navigation.goal_status)
          && placementAvailable("navigation_goal")
        ) {
          startPlacement("navigation_goal");
        }
      });
    }
    if (navigationButtons.placementConfirm) {
      navigationButtons.placementConfirm.addEventListener("click", confirmPlacement);
    }
    if (navigationButtons.placementCancel) {
      navigationButtons.placementCancel.addEventListener("click", cancelPlacement);
    }
    if (globalThis.addEventListener) globalThis.addEventListener("resize", refreshViewport);
    updateNavigationStatus();
    render();
    if (options.poll !== false) poll();

    return {
      poll,
      applyState,
      render,
      zoomIn: () => zoom(1.25),
      zoomOut: () => zoom(0.8),
      fit,
      refreshViewport,
      centerRobot,
      startPlacement,
      cancelPlacement,
      getPlacementPreview,
      setParkingPoints,
      setParkingPointsVisible,
      showNavigationRequestFailure,
      getTransform: () => ({...transform}),
      stop: () => { if (timer !== null) clearTimeout(timer); }
    };
  }

  globalThis.RobotMapView = {create, gridToWorld, worldToGrid};
}());
