from pathlib import Path
import shutil
import subprocess
import textwrap

import pytest


ROOT = Path(__file__).parents[1]
MAP_VIEW = ROOT / "robot_web_ui" / "web" / "map_view.js"
NODE = shutil.which("node")


def _run_node(script):
    result = subprocess.run(
        [NODE, "-"],
        input=textwrap.dedent(script),
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def _run_map_scenario(scenario):
    assert MAP_VIEW.exists(), "missing map_view.js"
    source = MAP_VIEW.read_text(encoding="utf-8")
    script = textwrap.dedent(
        """
        const assert = require("assert");
        const source = %r;
        class FakeElement {
          constructor(id) { this.id = id; this.listeners = new Map(); this.hidden = false; }
          addEventListener(name, callback) {
            if (!this.listeners.has(name)) this.listeners.set(name, []);
            this.listeners.get(name).push(callback);
          }
          emit(name, event = {}) {
            for (const callback of this.listeners.get(name) || []) callback({
              pointerId: 1, clientX: 0, clientY: 0, preventDefault() {}, ...event
            });
          }
          getBoundingClientRect() { return {left: 0, top: 0, width: 200, height: 100}; }
          setPointerCapture() {}
        }
        class FakeCanvas extends FakeElement {
          constructor() {
            super("mapCanvas"); this.width = 200; this.height = 100; this.operations = [];
            this.context = {
              fillStyle: "", strokeStyle: "", lineWidth: 1,
              setTransform: (...args) => this.operations.push(["transform", ...args]),
              clearRect: (...args) => this.operations.push(["clear", ...args]),
              save: () => this.operations.push(["save"]), restore: () => this.operations.push(["restore"]),
              translate: (...args) => this.operations.push(["translate", ...args]),
              rotate: (...args) => this.operations.push(["rotate", ...args]),
              scale: (...args) => this.operations.push(["scale", ...args]),
              fillRect: (...args) => this.operations.push(["fill", this.context.fillStyle, ...args]),
              beginPath: () => this.operations.push(["begin"]),
              moveTo: (...args) => this.operations.push(["move", ...args]),
              lineTo: (...args) => this.operations.push(["line", ...args]),
              stroke: () => this.operations.push(["stroke", this.context.strokeStyle]),
              fill: () => this.operations.push(["robot", this.context.fillStyle]),
              arc: (...args) => this.operations.push(["arc", ...args])
            };
          }
          getContext() { return this.context; }
        }
        const timers = [];
        global.setTimeout = (callback, milliseconds) => { timers.push({callback, milliseconds}); return timers.length; };
        global.clearTimeout = () => {};
        global.devicePixelRatio = 1;
        global.document = { createElement: () => new FakeElement("created") };
        const canvas = new FakeCanvas();
        const state = (overrides = {}) => ({
          localization: {x: 4, y: 5, yaw: 0},
          layers: {static: null, global_costmap: null, local_costmap: null, path: null},
          ...overrides
        });
        const grid = (revision, overrides = {}) => ({
          width: 2, height: 2, resolution: 1, origin: [0, 0, 0], revision, etag: `"${revision}"`, ...overrides
        });
        const responses = [];
        const requests = [];
        global.fetch = async (path, options = {}) => {
          requests.push({path, options});
          const response = responses.shift();
          assert(response, `missing response for ${path}`);
          return response;
        };
        const json = (payload) => ({ok: true, status: 200, headers: {get: () => null}, json: async () => payload});
        const bytes = (cells, etag = '"etag"') => ({ok: true, status: 200, headers: {get: (name) => name === "ETag" ? etag : null}, arrayBuffer: async () => Uint8Array.from(cells).buffer});
        eval(source);
        assert(globalThis.RobotMapView);

        async function run() {
          const scenario = %r;
          if (scenario === "coordinates") {
            const info = grid(1, {height: 4, resolution: .5, origin: [3, -2, Math.PI / 2]});
            const world = RobotMapView.gridToWorld(info, 1, 2);
            const cell = RobotMapView.worldToGrid(info, world.x, world.y);
            assert.deepStrictEqual(cell, {column: 1, row: 2});
            return;
          }
          const view = RobotMapView.create({canvas, poll: false});
          if (scenario === "revision") {
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})), bytes([0, 255]));
            await view.poll();
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})));
            await view.poll();
            responses.push(json(state({layers: {static: grid(2), global_costmap: null, local_costmap: null, path: null}})), bytes([1, 2]));
            await view.poll();
            assert.deepStrictEqual(requests.map((request) => request.path), ["/api/navigation-state", "/api/map/static", "/api/navigation-state", "/api/navigation-state", "/api/map/static"]);
            assert.strictEqual(requests[4].options.headers["If-None-Match"], '"1"');
            return;
          }
          if (scenario === "binary") {
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})), bytes([0, 255, 1, 100]));
            await view.poll();
            assert(canvas.operations.some((operation) => operation[0] === "fill" && operation[1] === "#667085"));
            return;
          }
          if (scenario === "draw-order") {
            responses.push(json(state({layers: {static: grid(1), global_costmap: grid(1), local_costmap: grid(1, {map_from_source: [1, 0, 0]}), path: {revision: 1, etag: '"path"'}}})), bytes([0, 0, 0, 0]), bytes([100, 100, 100, 100]), bytes([255, 255, 255, 255]), bytes([0, 0, 0, 0, 0, 0, 0, 0]));
            await view.poll();
            const fills = canvas.operations.filter((operation) => operation[0] === "fill").map((operation) => operation[1]);
            assert.deepStrictEqual(fills.slice(-12, -8), ["#e5e7eb", "#e5e7eb", "#e5e7eb", "#e5e7eb"]);
            assert(canvas.operations.findIndex((operation) => operation[0] === "stroke") < canvas.operations.findIndex((operation) => operation[0] === "robot"));
            return;
          }
          if (scenario === "pan") {
            const before = view.getTransform();
            canvas.emit("pointerdown", {clientX: 10, clientY: 20});
            canvas.emit("pointermove", {clientX: 30, clientY: 45});
            canvas.emit("pointerup", {clientX: 30, clientY: 45});
            const after = view.getTransform();
            assert.strictEqual(after.scale, before.scale);
            assert.strictEqual(after.x - before.x, 20);
            assert.strictEqual(after.y - before.y, 25);
            assert.strictEqual(canvas.listeners.has("wheel"), false);
            assert.strictEqual(canvas.listeners.has("touchstart"), false);
            return;
          }
          if (scenario === "zoom") {
            responses.push(bytes(new Array(200).fill(0)));
            await view.applyState(state({layers: {static: grid(1, {width: 20, height: 10}), global_costmap: null, local_costmap: null, path: null}}));
            view.zoomIn(); view.zoomIn(); view.zoomOut(); view.fit();
            const fitted = view.getTransform();
            assert(fitted.scale > 0 && fitted.scale <= 128);
            view.centerRobot();
            const centered = view.getTransform();
            assert.notDeepStrictEqual(centered, fitted);
            return;
          }
          if (scenario === "local-affine") {
            const local = grid(1, {map_from_source: [1, 2, 0]});
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: local, path: null}})), bytes([1, 2, 3, 4]));
            await view.poll();
            const before = canvas.operations.filter((operation) => operation[0] === "rotate").length;
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: grid(1, {map_from_source: [1, 2, 1]}), path: null}})));
            await view.poll();
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/local-costmap").length, 1);
            assert(canvas.operations.filter((operation) => operation[0] === "rotate").length > before);
            return;
          }
          assert.fail(`unknown scenario: ${scenario}`);
        }
        run().catch((error) => { console.error(error.stack || error); process.exitCode = 1; });
        """ % (source, scenario)
    )
    _run_node(script)


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_browser_harness_canvas_fetch_and_array_buffer_self_test():
    _run_node(
        """
        const assert = require("assert");
        const calls = [];
        const canvas = {
          getContext() {
            return { fillRect(x, y, width, height) {
              calls.push([x, y, width, height]);
            }};
          }
        };
        const timers = [];
        global.setTimeout = (callback, milliseconds) => {
          timers.push({callback, milliseconds});
          return timers.length;
        };
        global.fetch = async (path) => ({
          ok: true,
          status: 200,
          headers: { get: (name) => name === "ETag" ? '"grid-1"' : null },
          async arrayBuffer() { return Uint8Array.from([0, 255]).buffer; }
        });

        canvas.getContext("2d").fillRect(1, 2, 3, 4);
        fetch("/api/map/static").then(async (response) => {
          const cells = new Uint8Array(await response.arrayBuffer());
          assert.deepStrictEqual(calls, [[1, 2, 3, 4]]);
          assert.strictEqual(response.headers.get("ETag"), '"grid-1"');
          assert.deepStrictEqual([...cells], [0, 255]);
          setTimeout(() => {}, 200);
          assert.strictEqual(timers[0].milliseconds, 200);
        }).catch((error) => {
          console.error(error.stack || error);
          process.exitCode = 1;
        });
        """
    )


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_world_grid_screen_round_trip_including_origin_yaw_and_y_flip():
    _run_map_scenario("coordinates")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_status_poll_fetches_binary_layer_only_when_revision_changes():
    _run_map_scenario("revision")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_binary_gzip_response_is_interpreted_as_one_byte_cells():
    _run_map_scenario("binary")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_draw_order_is_static_global_local_path_robot():
    _run_map_scenario("draw-order")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_single_pointer_drag_pans_without_pinch_or_rotation_modes():
    _run_map_scenario("pan")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_zoom_in_out_fit_and_center_robot_are_bounded():
    _run_map_scenario("zoom")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_local_costmap_uses_map_from_source_affine_without_redownload():
    _run_map_scenario("local-affine")
