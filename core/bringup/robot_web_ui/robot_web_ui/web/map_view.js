(function () {
  "use strict";

  const LAYERS = {
    static: "/api/map/static",
    global_costmap: "/api/map/global-costmap",
    local_costmap: "/api/map/local-costmap",
    path: "/api/navigation-path"
  };
  const STATIC_PALETTE = {
    free: [229, 231, 235, 255],
    unknown: [102, 112, 133, 255],
    occupied: [217, 79, 79, 255]
  };
  const COSTMAP_PALETTE = {
    free: [0, 0, 0, 0],
    unknown: [102, 112, 133, 128],
    inflated: [240, 191, 104, 160],
    lethal: [217, 79, 79, 200]
  };
  const PALETTE = {
    path: "#4db7d6",
    robot: "#ffffff"
  };
  const MIN_SCALE = 0.25;
  const MAX_SCALE = 128;

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
    const cache = {};
    let latestState = {layers: {}};
    let inFlight = false;
    let timer = null;
    let dragging = null;
    const transform = {scale: 16, x: 100, y: 50};

    function bounds() {
      const rect = canvas.getBoundingClientRect();
      return {width: rect.width || 1, height: rect.height || 1};
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
      if (name === "static") {
        if (cell === 255) return STATIC_PALETTE.unknown;
        if (cell >= 100) return STATIC_PALETTE.occupied;
        return STATIC_PALETTE.free;
      }
      if (cell === 255) return COSTMAP_PALETTE.unknown;
      if (cell >= 99) return COSTMAP_PALETTE.lethal;
      if (cell >= 1) return COSTMAP_PALETTE.inflated;
      return COSTMAP_PALETTE.free;
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
      context.strokeStyle = PALETTE.path;
      context.lineWidth = 2 / transform.scale;
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
      context.fill();
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
        if (cells.byteLength === 0 || cells.byteLength % 8 !== 0) return;
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

    function updateNavigationStatus() {
      const layers = latestState.layers || {};
      const mapUnavailable = Boolean(latestState.map_error) || !layers.static;
      const messages = [];
      if (mapUnavailable) {
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
      if (options.navigationStatus) {
        options.navigationStatus.textContent = messages.join("；");
      }
      Object.values(buttons).forEach((button) => {
        if (button) button.disabled = mapUnavailable;
      });
    }

    async function applyState(state) {
      latestState = state || {layers: {}};
      const layers = latestState.layers || {};
      for (const name of Object.keys(LAYERS)) await updateLayer(name, layers[name]);
      updateNavigationStatus();
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
      transform.scale = Math.max(
        MIN_SCALE,
        Math.min(MAX_SCALE, transform.scale * multiplier)
      );
      render();
    }

    function fit() {
      const stateInfo = latestState.layers && latestState.layers.static;
      const info = effectiveGridInfo("static", stateInfo);
      if (!info) return;
      const corners = [
        gridToWorld(info, 0, 0),
        gridToWorld(info, info.width - 1, 0),
        gridToWorld(info, 0, info.height - 1),
        gridToWorld(info, info.width - 1, info.height - 1)
      ];
      const minX = Math.min(...corners.map((point) => point.x));
      const maxX = Math.max(...corners.map((point) => point.x));
      const minY = Math.min(...corners.map((point) => point.y));
      const maxY = Math.max(...corners.map((point) => point.y));
      const rect = bounds();
      transform.scale = Math.max(
        MIN_SCALE,
        Math.min(MAX_SCALE, 0.9 * Math.min(rect.width / (maxX - minX + info.resolution), rect.height / (maxY - minY + info.resolution)))
      );
      transform.x = rect.width / 2 - transform.scale * (minX + maxX) / 2;
      transform.y = rect.height / 2 + transform.scale * (minY + maxY) / 2;
      render();
    }

    function centerRobot() {
      const pose = latestState.localization;
      if (!pose) return;
      const rect = bounds();
      transform.x = rect.width / 2 - pose.x * transform.scale;
      transform.y = rect.height / 2 + pose.y * transform.scale;
      render();
    }

    canvas.addEventListener("pointerdown", (event) => {
      dragging = {pointerId: event.pointerId, x: event.clientX, y: event.clientY};
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      transform.x += event.clientX - dragging.x;
      transform.y += event.clientY - dragging.y;
      dragging.x = event.clientX;
      dragging.y = event.clientY;
      render();
    });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => {
      canvas.addEventListener(name, (event) => {
        if (dragging && event.pointerId === dragging.pointerId) dragging = null;
      });
    });
    if (buttons.zoomIn) buttons.zoomIn.addEventListener("click", () => zoom(1.25));
    if (buttons.zoomOut) buttons.zoomOut.addEventListener("click", () => zoom(0.8));
    if (buttons.fit) buttons.fit.addEventListener("click", fit);
    if (buttons.centerRobot) buttons.centerRobot.addEventListener("click", centerRobot);
    if (globalThis.addEventListener) globalThis.addEventListener("resize", render);
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
      centerRobot,
      getTransform: () => ({...transform}),
      stop: () => { if (timer !== null) clearTimeout(timer); }
    };
  }

  globalThis.RobotMapView = {create, gridToWorld, worldToGrid};
}());
