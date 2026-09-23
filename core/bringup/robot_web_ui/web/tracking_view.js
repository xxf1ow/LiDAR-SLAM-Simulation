(function () {
  "use strict";

  const PIXELS_PER_METER = 100;

  function create({canvas, request}) {
    const context = canvas.getContext("2d");

    function drawPoint(x, y, radius, color) {
      context.beginPath();
      context.arc(x, y, radius, 0, 2 * Math.PI);
      context.fillStyle = color;
      context.fill();
    }

    function setState(state) {
      const rect = canvas.getBoundingClientRect();
      const ratio = globalThis.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * ratio);
      canvas.height = Math.round(rect.height * ratio);
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, rect.width, rect.height);
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      if (state && state.frame_id === "base_footprint") {
        for (const point of state.points || []) {
          drawPoint(centerX - point[1] * PIXELS_PER_METER,
            centerY - point[0] * PIXELS_PER_METER, 2, "#4db7d6");
        }
        if (state.active && state.target) {
          const targetX = centerX - state.target.y * PIXELS_PER_METER;
          const targetY = centerY - state.target.x * PIXELS_PER_METER;
          context.beginPath();
          context.arc(targetX, targetY, 0.3 * PIXELS_PER_METER, 0, 2 * Math.PI);
          context.strokeStyle = "#d49a32";
          context.stroke();
          drawPoint(targetX, targetY, 6, "#ff8a00");
        }
      }
      drawPoint(centerX, centerY, 5, "#ffd400");
    }

    canvas.addEventListener("dblclick", async (event) => {
      const rect = canvas.getBoundingClientRect();
      const x = (rect.height / 2 - (event.clientY - rect.top)) / PIXELS_PER_METER;
      const y = (rect.width / 2 - (event.clientX - rect.left)) / PIXELS_PER_METER;
      await request("/api/tracking-target", {x, y});
    });

    return {setState};
  }

  globalThis.RobotTrackingView = {create};
})();
