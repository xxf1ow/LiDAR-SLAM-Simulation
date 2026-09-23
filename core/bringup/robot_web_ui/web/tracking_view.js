(function () {
  "use strict";

  const PIXELS_PER_METER = 100;

  function create({canvas, request, onSelectionError, onSelectionSuccess}) {
    const context = canvas.getContext("2d");
    let state = null;
    let scale = PIXELS_PER_METER;
    let offsetX = 0;
    let offsetY = 0;
    let dragging = null;

    function drawPoint(x, y, radius, color) {
      context.beginPath();
      context.arc(x, y, radius, 0, 2 * Math.PI);
      context.fillStyle = color;
      context.fill();
    }

    function render() {
      const rect = canvas.getBoundingClientRect();
      const ratio = globalThis.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * ratio);
      canvas.height = Math.round(rect.height * ratio);
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, rect.width, rect.height);
      const centerX = rect.width / 2 + offsetX;
      const centerY = rect.height / 2 + offsetY;
      if (state && state.frame_id === "base_footprint") {
        for (const point of state.points || []) {
          drawPoint(centerX - point[1] * scale,
            centerY - point[0] * scale, 2, "#4db7d6");
        }
        if (state.active && state.target) {
          const targetX = centerX - state.target.y * scale;
          const targetY = centerY - state.target.x * scale;
          context.beginPath();
          context.arc(targetX, targetY, 0.3 * scale, 0, 2 * Math.PI);
          context.strokeStyle = "#d49a32";
          context.stroke();
          drawPoint(targetX, targetY, 6, "#ff8a00");
        }
      }
      drawPoint(centerX, centerY, 5, "#ffd400");
    }

    function setState(nextState) {
      state = nextState;
      render();
    }

    function zoom(multiplier) {
      scale = Math.max(10, Math.min(400, scale * multiplier));
      render();
    }

    function centerRobot() {
      offsetX = 0;
      offsetY = 0;
      render();
    }

    function fit() {
      if (!state || state.frame_id !== "base_footprint") return;
      if (!(state.points || []).length && !(state.active && state.target)) return;
      const points = [[0, 0]];
      points.push(...(state.points || []));
      if (state.active && state.target) {
        const {x, y} = state.target;
        points.push([x - 0.3, y - 0.3], [x + 0.3, y + 0.3]);
      }
      const xs = points.map((point) => point[0]);
      const ys = points.map((point) => point[1]);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const rect = canvas.getBoundingClientRect();
      scale = Math.max(10, Math.min(400, 0.9 * Math.min(
        rect.height / Math.max(maxX - minX, 0.5),
        rect.width / Math.max(maxY - minY, 0.5)
      )));
      offsetX = (minY + maxY) * scale / 2;
      offsetY = (minX + maxX) * scale / 2;
      render();
    }

    canvas.addEventListener("pointerdown", (event) => {
      dragging = {pointerId: event.pointerId, x: event.clientX, y: event.clientY};
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      offsetX += event.clientX - dragging.x;
      offsetY += event.clientY - dragging.y;
      dragging.x = event.clientX;
      dragging.y = event.clientY;
      render();
    });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => {
      canvas.addEventListener(name, (event) => {
        if (dragging && event.pointerId === dragging.pointerId) dragging = null;
      });
    });

    canvas.addEventListener("dblclick", async (event) => {
      const rect = canvas.getBoundingClientRect();
      const x = (rect.height / 2 + offsetY - (event.clientY - rect.top)) / scale;
      const y = (rect.width / 2 + offsetX - (event.clientX - rect.left)) / scale;
      try {
        await request("/api/tracking-target", {x, y});
        onSelectionSuccess();
      } catch (error) {
        onSelectionError(error);
      }
    });

    if (globalThis.addEventListener) globalThis.addEventListener("resize", render);
    return {
      setState, zoomIn: () => zoom(1.25), zoomOut: () => zoom(0.8),
      fit, centerRobot, refreshViewport: render
    };
  }

  globalThis.RobotTrackingView = {create};
})();
