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
        encoding="utf-8",
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
          constructor(id, rect = {left: 0, top: 0, width: 200, height: 100}) {
            this.id = id; this.listeners = new Map(); this.hidden = false;
            this.disabled = false; this.textContent = ""; this.rect = rect;
            this.dataset = {};
          }
          addEventListener(name, callback) {
            if (!this.listeners.has(name)) this.listeners.set(name, []);
            this.listeners.get(name).push(callback);
          }
          emit(name, event = {}) {
            return Promise.all((this.listeners.get(name) || []).map((callback) => callback({
              currentTarget: this, pointerId: 1, clientX: 0, clientY: 0,
              preventDefault() {}, ...event
            })));
          }
          getBoundingClientRect() {
            return {
              ...this.rect,
              right: this.rect.left + this.rect.width,
              bottom: this.rect.top + this.rect.height
            };
          }
          setPointerCapture() {}
        }
        class FakeCanvas extends FakeElement {
          constructor() {
            super("mapCanvas"); this.width = 200; this.height = 100;
            this.operations = []; this.path = [];
            const contextStates = [];
            this.context = {
              fillStyle: "", strokeStyle: "", lineWidth: 1, globalAlpha: 1,
              setTransform: (...args) => this.operations.push(["transform", ...args]),
              clearRect: (...args) => this.operations.push(["clear", ...args]),
              save: () => {
                contextStates.push({
                  globalAlpha: this.context.globalAlpha,
                  strokeStyle: this.context.strokeStyle,
                  lineWidth: this.context.lineWidth
                });
                this.operations.push(["save"]);
              },
              restore: () => {
                Object.assign(this.context, contextStates.pop());
                this.operations.push(["restore"]);
              },
              translate: (...args) => this.operations.push(["translate", ...args]),
              rotate: (...args) => this.operations.push(["rotate", ...args]),
              scale: (...args) => this.operations.push(["scale", ...args]),
              drawImage: (...args) => this.operations.push(["drawImage", ...args, this.context.globalAlpha]),
              fillRect: (...args) => this.operations.push(["fill", this.context.fillStyle, ...args]),
              beginPath: () => { this.path = []; this.operations.push(["begin"]); },
              moveTo: (...args) => { this.path.push(["move", ...args]); this.operations.push(["move", ...args]); },
              lineTo: (...args) => { this.path.push(["line", ...args]); this.operations.push(["line", ...args]); },
              stroke: () => this.operations.push(["stroke", this.context.strokeStyle, this.context.globalAlpha, [...this.path], this.context.lineWidth]),
              fill: () => this.operations.push(["robot", this.context.fillStyle, this.context.globalAlpha]),
              arc: (...args) => this.operations.push(["arc", ...args]),
              closePath: () => this.operations.push(["close"])
            };
          }
          getContext() { return this.context; }
        }
        const offscreenCanvases = [];
        class FakeOffscreenCanvas {
          constructor() {
            this.width = 0; this.height = 0; this.operations = [];
            this.context = {
              createImageData: (width, height) => ({
                width, height, data: new Uint8ClampedArray(width * height * 4)
              }),
              putImageData: (imageData, x, y) => {
                this.imageData = imageData;
                this.operations.push(["putImageData", imageData, x, y]);
              }
            };
            offscreenCanvases.push(this);
          }
          getContext() { return this.context; }
        }
        const timers = [];
        global.setTimeout = (callback, milliseconds) => { timers.push({callback, milliseconds}); return timers.length; };
        global.clearTimeout = () => {};
        global.devicePixelRatio = 1;
        global.document = {
          createElement: (tagName) => tagName === "canvas"
            ? new FakeOffscreenCanvas()
            : new FakeElement("created")
        };
        const canvas = new FakeCanvas();
        const state = (overrides = {}) => ({
          localized: true,
          localization: {x: 4, y: 5, yaw: 0},
          gate_mode: "automatic",
          navigation: {
            initial_pose_ready: true,
            action_server_ready: true,
            goal_status: "idle",
            cancel_available: false,
            phase: null,
            distance_remaining: null,
            message: null
          },
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
        const bytes = (cells, etag = '"1"') => ({ok: true, status: 200, headers: {get: (name) => name === "ETag" ? etag : null}, arrayBuffer: async () => Uint8Array.from(cells).buffer});
        const pathBytes = (points, etag = '"path"') => {
          const buffer = new ArrayBuffer(points.length * 8);
          const view = new DataView(buffer);
          points.forEach(([x, y], index) => {
            view.setFloat32(index * 8, x, true);
            view.setFloat32(index * 8 + 4, y, true);
          });
          return {ok: true, status: 200, headers: {get: (name) => name === "ETag" ? etag : null}, arrayBuffer: async () => buffer};
        };
        eval(source);
        assert(globalThis.RobotMapView);

        async function run() {
          const scenario = %r;
          if (scenario === "coordinates") {
            const info = grid(1, {height: 4, resolution: .5, origin: [3, -2, Math.PI / 2]});
            const world = RobotMapView.gridToWorld(info, 1, 2);
            assert(Math.abs(world.x - 1.75) < 1e-12);
            assert(Math.abs(world.y + 1.25) < 1e-12);
            const cell = RobotMapView.worldToGrid(info, world.x, world.y);
            assert.deepStrictEqual(cell, {column: 1, row: 2});
            return;
          }
          const navigationStatus = new FakeElement("navigationStatus");
          const statusStrip = new FakeElement(
            "statusStrip", {left: 0, top: 0, width: 200, height: 20}
          );
          const controlDock = new FakeElement(
            "controlDock", {left: 0, top: 80, width: 200, height: 20}
          );
          const buttons = {
            zoomIn: new FakeElement("mapZoomIn"),
            zoomOut: new FakeElement("mapZoomOut"),
            fit: new FakeElement("mapFit"),
            centerRobot: new FakeElement("mapCenterRobot")
          };
          const navigationButtons = {
            initialPose: new FakeElement("setInitialPose"),
            navigationAction: new FakeElement("navigationAction"),
            placementConfirm: new FakeElement("confirmPlacement"),
            placementCancel: new FakeElement("cancelPlacement")
          };
          const navigationRequests = [];
          const navigationResults = [];
          const request = async (path, body) => {
            navigationRequests.push({path, body});
            const result = navigationResults.shift();
            if (result instanceof Error) throw result;
            return result || {ok: true};
          };
          const view = RobotMapView.create({
            canvas, buttons, navigationStatus, statusStrip, controlDock,
            navigationButtons, request, poll: false
          });
          if (scenario === "auto-fit-lifecycle") {
            const staticMap = grid(1, {width: 20, height: 10});
            responses.push(bytes(new Array(200).fill(0)));
            await view.applyState(state({
              layers: {
                static: staticMap, global_costmap: null,
                local_costmap: null, path: null
              }
            }));
            assert.deepStrictEqual(view.getTransform(), {scale: 7.2, x: 28, y: 76});

            const firstFit = view.getTransform();
            await view.applyState(state({
              layers: {
                static: staticMap, global_costmap: null,
                local_costmap: null, path: null
              }
            }));
            assert.deepStrictEqual(view.getTransform(), firstFit);

            controlDock.rect.top = 100;
            view.refreshViewport();
            assert.deepStrictEqual(view.getTransform(), {scale: 9, x: 10, y: 95});

            view.zoomIn();
            const userView = view.getTransform();
            controlDock.rect.top = 80;
            view.refreshViewport();
            assert.deepStrictEqual(view.getTransform(), userView);

            view.fit();
            assert.deepStrictEqual(view.getTransform(), {scale: 7.2, x: 28, y: 76});
            return;
          }
          if (scenario === "placement-preview-and-pointer") {
            assert.strictEqual(typeof view.startPlacement, "function");
            assert.strictEqual(typeof view.cancelPlacement, "function");
            assert.strictEqual(typeof view.getPlacementPreview, "function");
            assert.strictEqual(view.getPlacementPreview(), null);
            assert.strictEqual(
              canvas.operations.some((operation) =>
                operation[0] === "stroke" && operation[2] > 0 && operation[2] < 1
              ),
              false
            );

            responses.push(bytes([0, 0, 0, 0], '"7"'));
            await view.applyState(state({
              localization: {x: 4, y: 5, yaw: 0.25},
              layers: {
                static: grid(7), global_costmap: null,
                local_costmap: null, path: null
              }
            }));
            canvas.operations.length = 0;
            const initialWidth = canvas.width;
            const initialHeight = canvas.height;
            view.startPlacement("initial_pose");
            const preview = view.getPlacementPreview();
            assert.deepStrictEqual(preview, {
              x: 1, y: 13 / 18, yaw: 0.25, map_revision: 7
            });
            assert.notStrictEqual(view.getPlacementPreview(), preview);
            assert.strictEqual(navigationButtons.initialPose.hidden, true);
            assert.strictEqual(navigationButtons.navigationAction.hidden, true);
            assert.strictEqual(navigationButtons.placementConfirm.hidden, false);
            assert.strictEqual(navigationButtons.placementCancel.hidden, false);

            const placementStrokes = canvas.operations.filter((operation) =>
              operation[0] === "stroke" && operation[2] > 0 && operation[2] < 1
            );
            assert(placementStrokes.length >= 2, "crosshair and arrow must both draw");
            const arrowStroke = placementStrokes.at(-1);
            const tipSegment = arrowStroke[3].find((segment) =>
              segment[0] === "line" && Math.hypot(segment[1] - 100, segment[2] - 50) > 20
            );
            assert(tipSegment, "direction arrow needs a screen-space tip");
            const transformBeforeRotate = view.getTransform();
            await canvas.emit("pointerdown", {
              pointerId: 1, clientX: tipSegment[1], clientY: tipSegment[2]
            });
            const yawBeforeForeignPointer = view.getPlacementPreview().yaw;
            await canvas.emit("pointermove", {
              pointerId: 2, clientX: 100, clientY: 10
            });
            assert.strictEqual(
              view.getPlacementPreview().yaw,
              yawBeforeForeignPointer,
              "a second pointer must not rotate the owned arrow"
            );
            await canvas.emit("pointerup", {pointerId: 2});
            await canvas.emit("pointercancel", {pointerId: 2});
            await canvas.emit("pointermove", {
              pointerId: 1, clientX: 100, clientY: 10
            });
            assert(Math.abs(view.getPlacementPreview().yaw - Math.PI / 2) < 1e-12);
            await canvas.emit("pointercancel", {pointerId: 2});
            await canvas.emit("pointermove", {
              pointerId: 1, clientX: 60, clientY: 50
            });
            assert(Math.abs(Math.abs(view.getPlacementPreview().yaw) - Math.PI) < 1e-12);
            await canvas.emit("pointerup", {pointerId: 1});
            const releasedYaw = view.getPlacementPreview().yaw;
            await canvas.emit("pointermove", {
              pointerId: 1, clientX: 140, clientY: 50
            });
            assert.strictEqual(view.getPlacementPreview().yaw, releasedYaw);
            assert.deepStrictEqual(view.getTransform(), transformBeforeRotate);

            const yawBeforePan = view.getPlacementPreview().yaw;
            await canvas.emit("pointerdown", {clientX: 10, clientY: 20});
            await canvas.emit("pointermove", {clientX: 30, clientY: 45});
            await canvas.emit("pointerup", {clientX: 30, clientY: 45});
            assert.strictEqual(view.getPlacementPreview().yaw, yawBeforePan);
            const transformAfterPan = view.getTransform();
            assert.strictEqual(transformAfterPan.x - transformBeforeRotate.x, 20);
            assert.strictEqual(transformAfterPan.y - transformBeforeRotate.y, 25);

            const beforeZoom = view.getPlacementPreview();
            const beforeZoomStroke = canvas.operations.filter((operation) =>
              operation[0] === "stroke" && operation[2] > 0 && operation[2] < 1
            ).at(-1);
            const beforeZoomTip = beforeZoomStroke[3].find((segment) =>
              segment[0] === "line" && Math.hypot(segment[1] - 100, segment[2] - 50) > 20
            );
            view.zoomIn();
            const afterZoom = view.getPlacementPreview();
            const afterZoomStroke = canvas.operations.filter((operation) =>
              operation[0] === "stroke" && operation[2] > 0 && operation[2] < 1
            ).at(-1);
            const afterZoomTip = afterZoomStroke[3].find((segment) =>
              segment[0] === "line" && Math.hypot(segment[1] - 100, segment[2] - 50) > 20
            );
            assert.notDeepStrictEqual(afterZoom, beforeZoom);
            assert.strictEqual(
              Math.hypot(beforeZoomTip[1] - 100, beforeZoomTip[2] - 50),
              Math.hypot(afterZoomTip[1] - 100, afterZoomTip[2] - 50)
            );
            assert.strictEqual(canvas.width, initialWidth);
            assert.strictEqual(canvas.height, initialHeight);

            controlDock.rect.top = 100;
            view.refreshViewport();
            const resizedDockPreview = view.getPlacementPreview();
            assert.notStrictEqual(resizedDockPreview.y, afterZoom.y);
            const beforeFit = view.getPlacementPreview();
            view.fit();
            assert.notDeepStrictEqual(view.getPlacementPreview(), beforeFit);
            assert.strictEqual(canvas.width, initialWidth);
            assert.strictEqual(canvas.height, initialHeight);

            canvas.operations.length = 0;
            view.cancelPlacement();
            assert.strictEqual(view.getPlacementPreview(), null);
            assert.strictEqual(navigationButtons.placementConfirm.hidden, true);
            assert.strictEqual(navigationButtons.placementCancel.hidden, true);
            assert.strictEqual(navigationButtons.initialPose.hidden, false);
            assert.strictEqual(navigationButtons.navigationAction.hidden, false);
            assert.strictEqual(
              canvas.operations.some((operation) =>
                operation[0] === "stroke" && operation[2] > 0 && operation[2] < 1
              ),
              false
            );
            assert.deepStrictEqual(navigationRequests, []);
            return;
          }
          if (scenario === "placement-requests") {
            assert.strictEqual(typeof view.startPlacement, "function");
            responses.push(bytes([0, 0, 0, 0], '"3"'));
            await view.applyState(state({
              localization: {x: 4, y: 5, yaw: 0.5},
              layers: {
                static: grid(3), global_costmap: null,
                local_costmap: null, path: null
              }
            }));

            view.startPlacement("initial_pose");
            const initialPreview = view.getPlacementPreview();
            let resolveInitialRequest;
            navigationResults.push(new Promise((resolve) => {
              resolveInitialRequest = resolve;
            }));
            const firstConfirm = navigationButtons.placementConfirm.emit("click");
            await view.applyState(state({
              localization: {x: 4, y: 5, yaw: 0.5},
              layers: {
                static: grid(3), global_costmap: null,
                local_costmap: null, path: null
              }
            }));
            const duplicateConfirm = navigationButtons.placementConfirm.emit("click");
            assert.strictEqual(navigationRequests.length, 1);
            resolveInitialRequest({ok: true});
            await Promise.all([firstConfirm, duplicateConfirm]);
            assert.deepStrictEqual(navigationRequests[0], {
              path: "/api/initial-pose", body: initialPreview
            });
            assert.strictEqual(view.getPlacementPreview(), null);

            view.startPlacement("navigation_goal");
            const rejectedPreview = view.getPlacementPreview();
            navigationResults.push(new Error("action unavailable"));
            await navigationButtons.placementConfirm.emit("click");
            assert.deepStrictEqual(view.getPlacementPreview(), rejectedPreview);
            assert.strictEqual(
              (navigationStatus.textContent.match(/请求失败/g) || []).length,
              1
            );
            assert(navigationStatus.textContent.includes("action unavailable"));
            assert.deepStrictEqual(navigationRequests[1], {
              path: "/api/navigation-goal", body: rejectedPreview
            });

            view.startPlacement("initial_pose");
            assert(!navigationStatus.textContent.includes("action unavailable"));
            view.cancelPlacement();
            assert.strictEqual(navigationRequests.length, 2);

            view.startPlacement("navigation_goal");
            const retryPreview = view.getPlacementPreview();
            navigationResults.push(new Error("retry me"));
            await navigationButtons.placementConfirm.emit("click");
            assert.deepStrictEqual(view.getPlacementPreview(), retryPreview);
            navigationResults.push({goal_status: "sending"});
            await navigationButtons.placementConfirm.emit("click");
            assert.strictEqual(view.getPlacementPreview(), null);
            assert(!navigationStatus.textContent.includes("retry me"));
            assert.deepStrictEqual(navigationRequests.slice(2, 4), [
              {path: "/api/navigation-goal", body: retryPreview},
              {path: "/api/navigation-goal", body: retryPreview}
            ]);

            navigationResults.push({});
            await view.applyState(state({
              navigation: {
                initial_pose_ready: true,
                action_server_ready: true,
                goal_status: "navigating",
                cancel_available: true,
                phase: null,
                distance_remaining: null,
                message: null
              },
              layers: {
                static: grid(3), global_costmap: null,
                local_costmap: null, path: null
              }
            }));
            await navigationButtons.navigationAction.emit("click");
            assert.deepStrictEqual(navigationRequests[4], {
              path: "/api/navigation-cancel", body: {}
            });
            assert.strictEqual(
              navigationRequests.some((item) => item.path === "/api/resume-automatic"),
              false
            );
            return;
          }
          if (scenario === "cancel-request-pending") {
            const navigation = (overrides = {}) => ({
              initial_pose_ready: true,
              action_server_ready: true,
              goal_status: "navigating",
              cancel_available: true,
              distance_remaining: null,
              message: null,
              ...overrides
            });
            const activeState = (overrides = {}) => state({
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: null, path: null
              },
              navigation: navigation(overrides)
            });
            responses.push(bytes([0, 0, 0, 0]));
            await view.applyState(activeState());
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);

            let resolveCancel;
            navigationResults.push(new Promise((resolve) => {
              resolveCancel = resolve;
            }));
            const firstCancel = navigationButtons.navigationAction.emit("click");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            await view.applyState(activeState());
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            const duplicateCancel = navigationButtons.navigationAction.emit("click");
            resolveCancel({ok: true, goal_status: "canceling"});
            await Promise.all([firstCancel, duplicateCancel]);
            assert.strictEqual(
              navigationRequests.filter((item) =>
                item.path === "/api/navigation-cancel"
              ).length,
              1
            );
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);

            await view.applyState(activeState({
              goal_status: "canceling", cancel_available: false
            }));
            await view.applyState(activeState({
              goal_status: "canceled", cancel_available: false
            }));
            await view.applyState(activeState());
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);

            let rejectCancel;
            navigationResults.push(new Promise((_resolve, reject) => {
              rejectCancel = reject;
            }));
            const failingCancel = navigationButtons.navigationAction.emit("click");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            await view.applyState(activeState());
            const duplicateFailure = navigationButtons.navigationAction.emit("click");
            rejectCancel(new Error("cancel offline"));
            await Promise.all([failingCancel, duplicateFailure]);
            assert.strictEqual(
              navigationRequests.filter((item) =>
                item.path === "/api/navigation-cancel"
              ).length,
              2
            );
            assert(navigationStatus.textContent.includes("cancel offline"));
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);

            navigationResults.push({ok: true, goal_status: "canceling"});
            await navigationButtons.navigationAction.emit("click");
            assert.strictEqual(
              navigationRequests.filter((item) =>
                item.path === "/api/navigation-cancel"
              ).length,
              3
            );
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            assert(!navigationStatus.textContent.includes("cancel offline"));
            await view.applyState(activeState({
              goal_status: "canceling", cancel_available: false
            }));
            return;
          }
          if (scenario === "navigation-phase-labels") {
            const phaseLabels = {
              planning: "正在规划路径",
              following: "正在跟踪路径",
              clearing_global_plan: "路径规划受阻，正在清理全局代价地图",
              clearing_local_control: "路径跟踪受阻，正在清理局部代价地图",
              clearing_global_recovery: "导航恢复：正在清理全局代价地图",
              clearing_local_recovery: "导航恢复：正在清理局部代价地图",
              spinning: "导航恢复：正在原地旋转",
              waiting: "导航恢复：正在等待障碍消退",
              backing_up: "导航恢复：正在后退"
            };
            const navigation = (overrides = {}) => ({
              initial_pose_ready: true,
              action_server_ready: true,
              goal_status: "navigating",
              cancel_available: true,
              phase: null,
              distance_remaining: 3.5,
              message: null,
              ...overrides
            });
            const availableState = (overrides = {}) => state({
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: null, path: null
              },
              navigation: navigation(overrides)
            });
            responses.push(bytes([0, 0, 0, 0]));
            for (const [phase, label] of Object.entries(phaseLabels)) {
              await view.applyState(availableState({phase}));
              assert.strictEqual(
                navigationStatus.textContent,
                `${label}，剩余 3.5 米`,
                phase
              );
            }
            await view.applyState(availableState({phase: null}));
            assert.strictEqual(
              navigationStatus.textContent,
              "导航中，剩余 3.5 米"
            );
            await view.applyState(availableState({phase: "unknown_phase"}));
            assert.strictEqual(
              navigationStatus.textContent,
              "导航中，剩余 3.5 米"
            );
            await view.applyState(availableState({phase: "planning", distance_remaining: null}));
            assert.strictEqual(navigationStatus.textContent, "正在规划路径");

            for (const [goalStatus, expected] of [
              ["sending", "导航目标发送中"],
              ["canceling", "导航取消中"],
              ["succeeded", "导航已到达目标"],
              ["canceled", "导航已取消"],
              ["failed", "导航失败"]
            ]) {
              await view.applyState(availableState({
                goal_status: goalStatus,
                phase: "planning",
                distance_remaining: 3.5
              }));
              assert(navigationStatus.textContent.startsWith(expected));
              assert(!navigationStatus.textContent.includes("正在规划路径"));
              assert(!navigationStatus.textContent.includes("剩余"));
            }
            return;
          }
          if (scenario === "navigation-action-button") {
            const navigation = (overrides = {}) => ({
              initial_pose_ready: true,
              action_server_ready: true,
              goal_status: "idle",
              cancel_available: false,
              phase: null,
              distance_remaining: null,
              message: null,
              ...overrides
            });
            const availableState = (overrides = {}) => state({
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: null, path: null
              },
              navigation: navigation(overrides)
            });
            responses.push(bytes([0, 0, 0, 0]));
            await view.applyState(availableState());
            assert.strictEqual(navigationButtons.navigationAction.textContent, "设置导航目标");
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            await navigationButtons.navigationAction.emit("click");
            assert.strictEqual(view.getPlacementPreview() !== null, true);
            assert.strictEqual(navigationButtons.navigationAction.hidden, true);
            assert.strictEqual(navigationButtons.placementConfirm.hidden, false);
            assert.strictEqual(navigationButtons.placementCancel.hidden, false);
            view.cancelPlacement();

            for (const goalStatus of ["succeeded", "canceled", "failed"]) {
              await view.applyState(availableState({goal_status: goalStatus}));
              assert.strictEqual(navigationButtons.navigationAction.textContent, "设置导航目标");
              assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            }
            await view.applyState(availableState({action_server_ready: false}));
            assert.strictEqual(navigationButtons.navigationAction.textContent, "设置导航目标");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            await view.applyState(availableState({goal_status: "sending"}));
            assert.strictEqual(navigationButtons.navigationAction.textContent, "目标发送中");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            await view.applyState(availableState({
              goal_status: "navigating",
              cancel_available: false
            }));
            assert.strictEqual(navigationButtons.navigationAction.textContent, "导航中");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);

            await view.applyState(availableState({
              goal_status: "navigating",
              cancel_available: true
            }));
            assert.strictEqual(navigationButtons.navigationAction.textContent, "取消导航");
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            let resolveCancel;
            navigationResults.push(new Promise((resolve) => {
              resolveCancel = resolve;
            }));
            const firstCancel = navigationButtons.navigationAction.emit("click");
            assert.strictEqual(navigationButtons.navigationAction.textContent, "取消中");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            const duplicateCancel = navigationButtons.navigationAction.emit("click");
            resolveCancel({ok: true});
            await Promise.all([firstCancel, duplicateCancel]);
            assert.strictEqual(
              navigationRequests.filter((item) => item.path === "/api/navigation-cancel").length,
              1
            );
            await view.applyState(availableState({
              goal_status: "canceling",
              cancel_available: false
            }));
            assert.strictEqual(navigationButtons.navigationAction.textContent, "取消中");
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            return;
          }
          if (scenario === "navigation-click-availability-race") {
            const availableState = state({
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: null, path: null
              }
            });
            responses.push(bytes([0, 0, 0, 0], '"1"'));
            await view.applyState(availableState);
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);

            let resolveStatic;
            responses.push({
              ok: true,
              status: 200,
              headers: {get: (name) => name === "ETag" ? '"2"' : null},
              arrayBuffer: () => new Promise((resolve) => {
                resolveStatic = resolve;
              })
            });
            const unavailableState = state({
              localized: false,
              gate_mode: "manual",
              navigation: {
                initial_pose_ready: true,
                action_server_ready: false,
                goal_status: "idle",
                cancel_available: false,
                phase: null,
                distance_remaining: null,
                message: null
              },
              layers: {
                static: grid(2), global_costmap: null,
                local_costmap: null, path: null
              }
            });
            const statePromise = view.applyState(unavailableState);
            await Promise.resolve();
            await Promise.resolve();
            assert.strictEqual(typeof resolveStatic, "function");
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            await navigationButtons.navigationAction.emit("click");
            assert.strictEqual(view.getPlacementPreview(), null);
            assert.deepStrictEqual(navigationRequests, []);
            resolveStatic(Uint8Array.from([0, 0, 0, 0]).buffer);
            await statePromise;
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);
            return;
          }
          if (scenario === "navigation-availability-and-status") {
            assert.strictEqual(typeof view.startPlacement, "function");
            const navigation = (overrides = {}) => ({
              initial_pose_ready: true,
              action_server_ready: true,
              goal_status: "idle",
              cancel_available: false,
              distance_remaining: null,
              message: null,
              ...overrides
            });
            const availableState = (overrides = {}) => state({
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: null, path: null
              },
              ...overrides
            });
            responses.push(bytes([0, 0, 0, 0]));
            await view.applyState(availableState());
            assert.strictEqual(navigationButtons.initialPose.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.textContent, "设置导航目标");

            await view.applyState(availableState({gate_mode: "manual"}));
            assert.strictEqual(navigationButtons.initialPose.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);

            await view.applyState(availableState({
              localized: false, localization: null
            }));
            assert.strictEqual(navigationButtons.initialPose.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);

            await view.applyState(availableState({
              navigation: navigation({action_server_ready: false})
            }));
            assert.strictEqual(navigationButtons.navigationAction.disabled, true);

            await view.applyState(availableState({
              navigation: navigation({initial_pose_ready: false})
            }));
            assert.strictEqual(navigationButtons.initialPose.disabled, true);

            await view.applyState(availableState({
              navigation: navigation({
                initial_pose_ready: true,
                goal_status: "navigating",
                cancel_available: true,
                distance_remaining: 3.5,
                message: "controller active"
              })
            }));
            assert.strictEqual(navigationButtons.initialPose.disabled, true);
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.textContent, "取消导航");
            assert(navigationStatus.textContent.includes("3.5"));
            assert(!navigationStatus.textContent.includes("navigating"));
            assert(!/%%|ETA|预计/.test(navigationStatus.textContent));

            for (const [goalStatus, expected] of [
              ["sending", "发送"], ["canceling", "取消"],
              ["succeeded", "到达"], ["canceled", "已取消"],
              ["failed", "失败"]
            ]) {
              await view.applyState(availableState({
                navigation: navigation({
                  initial_pose_ready: true,
                  goal_status: goalStatus,
                  message: goalStatus === "failed" ? "planner stopped" : null
                })
              }));
              assert.strictEqual(
                navigationButtons.initialPose.disabled,
                ["sending", "navigating", "canceling"].includes(goalStatus),
                goalStatus
              );
              assert(navigationStatus.textContent.includes(expected), goalStatus);
              assert(!navigationStatus.textContent.includes(goalStatus), goalStatus);
              assert(!/%%|ETA|预计/.test(navigationStatus.textContent), goalStatus);
            }
            assert(navigationStatus.textContent.includes("planner stopped"));

            await view.applyState(availableState({
              map_error: "map offline",
              navigation: navigation({
                initial_pose_ready: true,
                goal_status: "navigating",
                cancel_available: true
              })
            }));
            assert.strictEqual(navigationButtons.initialPose.disabled, true);
            assert.strictEqual(navigationButtons.navigationAction.disabled, false);
            assert.strictEqual(navigationButtons.navigationAction.textContent, "取消导航");
            return;
          }
          if (scenario === "revision") {
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})), bytes([0, 255, 0, 0]));
            await view.poll();
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})));
            await view.poll();
            responses.push(json(state({layers: {static: grid(2), global_costmap: null, local_costmap: null, path: null}})), bytes([1, 2, 3, 4], '"2"'));
            await view.poll();
            assert.deepStrictEqual(requests.map((request) => request.path), ["/api/navigation-state", "/api/map/static", "/api/navigation-state", "/api/navigation-state", "/api/map/static"]);
            assert.strictEqual(requests[4].options.headers["If-None-Match"], '"1"');
            return;
          }
          if (scenario === "etag-interleaving") {
            const first = grid(1, {etag: '"A"'});
            const second = grid(2, {etag: '"B"', origin: [10, 20, 0]});
            responses.push(json(state({layers: {static: first, global_costmap: null, local_costmap: null, path: null}})), bytes([0, 0, 0, 0], '"A"'));
            await view.poll();
            const priorRaster = offscreenCanvases[0];
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: null}})), bytes([100, 100, 100, 100], '"C"'));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 1);
            assert(canvas.operations.some((operation) => operation[0] === "drawImage" && operation[1] === priorRaster));
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: null}})), bytes([100, 100, 100, 100], '"B"'));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 2);
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/static").length, 3);
            return;
          }
          if (scenario === "asset-lengths") {
            const first = grid(1, {etag: '"A"'});
            const second = grid(2, {etag: '"B"'});
            const firstPath = {revision: 1, etag: '"P"'};
            const secondPath = {revision: 2, etag: '"Q"'};
            responses.push(json(state({layers: {static: first, global_costmap: null, local_costmap: null, path: firstPath}})), bytes([0, 0, 0, 0], '"A"'), pathBytes([[1, 2]], '"P"'));
            await view.poll();
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: secondPath}})), bytes([100, 100, 100], '"B"'), bytes([0, 0, 0, 0], '"Q"'));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 1);
            assert.strictEqual(canvas.operations.filter((operation) => operation[0] === "move" && operation[1] === 1 && operation[2] === 2).length, 2);
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: secondPath}})), bytes([100, 100, 100, 100], '"B"'), pathBytes([[9, 10]], '"Q"'));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 2);
            assert(canvas.operations.some((operation) => operation[0] === "move" && operation[1] === 9 && operation[2] === 10));
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/static").length, 3);
            assert.strictEqual(requests.filter((request) => request.path === "/api/navigation-path").length, 3);
            return;
          }
          if (scenario === "empty-assets") {
            const first = grid(1, {etag: '"A"'});
            const second = grid(2, {etag: '"B"'});
            const firstPath = {revision: 1, etag: '"P"'};
            const secondPath = {revision: 2, etag: '"Q"'};
            responses.push(json(state({layers: {static: first, global_costmap: null, local_costmap: null, path: firstPath}})), bytes([0, 0, 0, 0], '"A"'), pathBytes([[1, 2]], '"P"'));
            await view.poll();
            assert(canvas.operations.some((operation) => operation[0] === "stroke" && operation[1] === "#55ffff"));
            canvas.operations.length = 0;
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: secondPath}})), bytes([], '"B"'), bytes([], '"Q"'));
            await view.poll();
            const emptyRevisionOperations = canvas.operations.slice();
            canvas.operations.length = 0;
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: secondPath}})), bytes([], '"B"'), bytes([], '"Q"'));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 1);
            assert.strictEqual(emptyRevisionOperations.some((operation) => operation[0] === "stroke" && operation[1] === "#55ffff"), false);
            assert.strictEqual(canvas.operations.some((operation) => operation[0] === "stroke" && operation[1] === "#55ffff"), false);
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/static").length, 3);
            assert.strictEqual(requests.filter((request) => request.path === "/api/navigation-path").length, 2);
            return;
          }
          if (scenario === "not-modified-etag-mismatch") {
            const first = grid(1, {etag: '"A"'});
            const second = grid(2, {etag: '"B"'});
            const mismatched304 = {ok: false, status: 304, headers: {get: () => '"C"'}};
            responses.push(json(state({layers: {static: first, global_costmap: null, local_costmap: null, path: null}})), bytes([0, 0, 0, 0], '"A"'));
            await view.poll();
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: null}})), mismatched304);
            await view.poll();
            responses.push(json(state({layers: {static: second, global_costmap: null, local_costmap: null, path: null}})), bytes([100, 100, 100, 100], '"B"'));
            await view.poll();
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/static").length, 3);
            assert.strictEqual(offscreenCanvases.length, 2);
            return;
          }
          if (scenario === "navigation-status") {
            const manualNotice = new FakeElement("notice");
            manualNotice.textContent = "人工控制请求失败";
            await view.applyState(state({
              map_error: "bad map yaml", localization: null,
              layers: {static: null, global_costmap: null, local_costmap: null, path: null}
            }));
            assert(navigationStatus.textContent.includes("bad map yaml"));
            assert(navigationStatus.textContent.includes("等待定位"));
            assert(Object.values(buttons).every((button) => button.disabled));
            assert.strictEqual(manualNotice.textContent, "人工控制请求失败");
            responses.push(bytes([0, 0, 0, 0]));
            await view.applyState(state({
              localization: null,
              layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}
            }));
            assert.strictEqual(navigationStatus.textContent, "等待定位");
            assert(Object.values(buttons).every((button) => !button.disabled));
            await view.applyState(state({
              localization: null,
              localization_error: "expected map pose",
              path_error: "expected map path",
              layers: {
                static: grid(1), global_costmap: null,
                local_costmap: {transform_available: false, transform_error: "missing transform"},
                path: null
              }
            }));
            assert(navigationStatus.textContent.includes("expected map pose"));
            assert(navigationStatus.textContent.includes("expected map path"));
            assert(navigationStatus.textContent.includes("missing transform"));
            assert(Object.values(buttons).every((button) => !button.disabled));
            assert.strictEqual(manualNotice.textContent, "人工控制请求失败");
            return;
          }
          if (scenario === "raster-semantics") {
            const info = grid(1, {width: 4, height: 1, origin: [3, -2, Math.PI / 2]});
            const costmap = grid(1, {width: 7, height: 1, origin: [3, -2, Math.PI / 2]});
            const bottom = RobotMapView.gridToWorld(info, 0, 0);
            const top = RobotMapView.gridToWorld(info, 0, 1);
            responses.push(
              json(state({
                localization: {...bottom, yaw: 0},
                layers: {
                  static: info,
                  global_costmap: costmap,
                  local_costmap: {...costmap, map_from_source: [0, 0, 0], transform_available: true},
                  path: {revision: 1, etag: '"path"'}
                }
              })),
              bytes([0, 50, 100, 255]),
              bytes([0, 1, 50, 98, 99, 100, 255]),
              bytes([0, 1, 50, 98, 99, 100, 255]),
              pathBytes([[top.x, top.y], [top.x + 1, top.y]])
            );
            await view.poll();
            assert.deepStrictEqual([...offscreenCanvases[0].imageData.data], [
              255, 255, 255, 255,
              128, 128, 128, 255,
              0, 0, 0, 255,
              112, 137, 134, 255
            ]);
            assert.deepStrictEqual([...offscreenCanvases[1].imageData.data], [
              0, 0, 0, 0,
              2, 0, 253, 255,
              127, 0, 128, 255,
              249, 0, 6, 255,
              0, 255, 255, 255,
              255, 0, 255, 255,
              112, 137, 134, 255
            ]);
            assert.deepStrictEqual([...offscreenCanvases[2].imageData.data], [
              0, 0, 0, 0,
              2, 0, 253, 255,
              127, 0, 128, 255,
              249, 0, 6, 255,
              0, 255, 255, 255,
              255, 0, 255, 255,
              112, 137, 134, 255
            ]);
            assert.deepStrictEqual(bottom, {x: 2.5, y: -1.5});
            assert.deepStrictEqual(top, {x: 1.5, y: -1.5});
            assert(canvas.operations.some((operation) => operation[0] === "move" && Math.abs(operation[1] - top.x) < 1e-6 && Math.abs(operation[2] - top.y) < 1e-6));
            assert(canvas.operations.some((operation) => operation[0] === "translate" && operation[1] === bottom.x && operation[2] === bottom.y));
            assert.deepStrictEqual(
              canvas.operations.filter((operation) => operation[0] === "drawImage").map((operation) => operation.at(-1)),
              [1, 0.45, 0.70]
            );
            const mapScale = view.getTransform().scale;
            assert.deepStrictEqual(
              canvas.operations.filter((operation) => operation[0] === "stroke").slice(0, 2).map((operation) => [operation[1], operation[2], operation[4]]),
              [["#111827", 1, 5 / mapScale], ["#55ffff", 1, 2.5 / mapScale]]
            );
            assert(canvas.operations.some((operation) => operation[0] === "robot" && operation[1] === "#ffd400"));
            const robotIndex = canvas.operations.findIndex((operation) => operation[0] === "robot");
            assert.strictEqual(canvas.operations[robotIndex - 1][0], "close");
            assert(canvas.operations.some((operation) => operation[0] === "stroke" && operation[1] === "#111827" && operation[4] === 2 / mapScale));
            const images = canvas.operations.filter((operation) => operation[0] === "drawImage").map((operation) => operation[1]);
            assert.deepStrictEqual(images, offscreenCanvases);
            assert.deepStrictEqual(
              canvas.operations.filter((operation) => ["drawImage", "stroke", "robot"].includes(operation[0])).map((operation) => operation[0]),
              ["drawImage", "drawImage", "drawImage", "stroke", "stroke", "robot", "stroke"]
            );
            const firstRobotRotation = canvas.operations.filter((operation) => operation[0] === "rotate").at(-1)[1];
            assert.strictEqual(firstRobotRotation, 0);
            canvas.operations.length = 0;
            await view.applyState(state({localization: {x: 4, y: 5, yaw: Math.PI / 2}}));
            assert.strictEqual(canvas.operations.filter((operation) => operation[0] === "rotate").at(-1)[1], Math.PI / 2);
            canvas.operations.length = 0;
            view.startPlacement("initial_pose");
            const placementStrokes = canvas.operations.filter((operation) => operation[0] === "stroke" && operation[2] === 0.90);
            assert.deepStrictEqual(
              placementStrokes.map((operation) => [operation[1], operation[2], operation[4]]),
              [["#111827", 0.90, 5], ["#ff8a00", 0.90, 2.5]]
            );
            return;
          }
          if (scenario === "raster-cache") {
            const local = grid(1, {map_from_source: [0, 0, 0], transform_available: true});
            responses.push(json(state({layers: {static: grid(1), global_costmap: grid(1), local_costmap: local, path: null}})), bytes([0, 0, 0, 0]), bytes([1, 2, 3, 4]), bytes([5, 6, 7, 8]));
            await view.poll();
            assert.strictEqual(offscreenCanvases.length, 3);
            const builds = offscreenCanvases.map((item) => item.operations.length);
            responses.push(json(state({layers: {static: grid(1), global_costmap: grid(1), local_costmap: {...local, map_from_source: [2, 3, 1]}, path: null}})));
            await view.poll();
            view.zoomIn(); view.zoomOut(); view.fit(); view.centerRobot();
            canvas.emit("pointerdown", {clientX: 1, clientY: 1});
            canvas.emit("pointermove", {clientX: 4, clientY: 5});
            canvas.emit("pointerup", {clientX: 4, clientY: 5});
            view.render();
            assert.strictEqual(offscreenCanvases.length, 3);
            assert.deepStrictEqual(offscreenCanvases.map((item) => item.operations.length), builds);
            assert(canvas.operations.filter((operation) => operation[0] === "drawImage").length > 3);
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
            const local = grid(1, {map_from_source: [1, 2, 0], transform_available: true});
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: local, path: null}})), bytes([1, 2, 3, 4]));
            await view.poll();
            const before = canvas.operations.filter((operation) => operation[0] === "rotate").length;
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: grid(1, {map_from_source: [1, 2, 1], transform_available: true}), path: null}})));
            await view.poll();
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/local-costmap").length, 1);
            assert(canvas.operations.filter((operation) => operation[0] === "rotate").length > before);
            return;
          }
          if (scenario === "invalid-local-affine") {
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: grid(1, {map_from_source: [1, 2, 0], transform_available: true}), path: null}})), bytes([1, 2, 3, 4]));
            await view.poll();
            canvas.operations.length = 0;
            responses.push(json(state({layers: {static: null, global_costmap: null, local_costmap: grid(1, {map_from_source: null, transform_available: false}), path: null}})));
            await view.poll();
            assert.strictEqual(canvas.operations.some((operation) => operation[0] === "fill"), false);
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/local-costmap").length, 1);
            return;
          }
          if (scenario === "not-modified") {
            const notModified = {ok: false, status: 304, headers: {get: () => '"1"'}};
            responses.push(json(state({layers: {static: grid(1), global_costmap: null, local_costmap: null, path: null}})), bytes([255, 0, 0, 0]));
            await view.poll();
            canvas.operations.length = 0;
            responses.push(json(state({layers: {static: grid(2, {etag: '"1"'}), global_costmap: null, local_costmap: null, path: null}})), notModified);
            await view.poll();
            assert(canvas.operations.some((operation) => operation[0] === "drawImage" && operation[1] === offscreenCanvases[0]));
            assert.deepStrictEqual([...offscreenCanvases[0].imageData.data.slice(0, 4)], [112, 137, 134, 255]);
            assert.strictEqual(offscreenCanvases.length, 1);
            responses.push(json(state({layers: {static: grid(2, {etag: '"1"'}), global_costmap: null, local_costmap: null, path: null}})));
            await view.poll();
            assert.strictEqual(requests.filter((request) => request.path === "/api/map/static").length, 2);
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
def test_state_etag_must_match_200_asset_before_cache_replacement():
    _run_map_scenario("etag-interleaving")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_malformed_grid_and_path_lengths_retain_cache_and_retry():
    _run_map_scenario("asset-lengths")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_empty_path_clears_route_while_empty_grid_retries():
    _run_map_scenario("empty-assets")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_state_etag_must_match_304_before_revision_advances():
    _run_map_scenario("not-modified-etag-mismatch")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_degradation_status_is_separate_and_scoped():
    _run_map_scenario("navigation-status")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_mobile_placement_preview_rendering_and_pointer_ownership():
    _run_map_scenario("placement-preview-and-pointer")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_mobile_placement_confirmation_retry_and_cancel_requests():
    _run_map_scenario("placement-requests")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_cancel_request_pending_owns_double_tap_and_poll_races():
    _run_map_scenario("cancel-request-pending")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_action_availability_and_human_status_labels():
    _run_map_scenario("navigation-availability-and-status")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_phase_labels_copy_and_distance_composition():
    _run_map_scenario("navigation-phase-labels")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_action_button_unifies_goal_and_cancel_feedback():
    _run_map_scenario("navigation-action-button")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_navigation_click_rechecks_latest_availability_during_layer_fetch():
    _run_map_scenario("navigation-click-availability-race")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_binary_gzip_response_is_interpreted_as_one_byte_cells():
    _run_map_scenario("raster-semantics")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_grid_rasters_are_built_once_and_composited_for_view_changes():
    _run_map_scenario("raster-cache")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_single_pointer_drag_pans_without_pinch_or_rotation_modes():
    _run_map_scenario("pan")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
@pytest.mark.parametrize("scenario", ["zoom", "auto-fit-lifecycle"])
def test_view_transform_controls_and_auto_fit_lifecycle(scenario):
    _run_map_scenario(scenario)


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_local_costmap_uses_map_from_source_affine_without_redownload():
    _run_map_scenario("local-affine")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_local_costmap_with_missing_map_affine_is_not_fetched_or_drawn():
    _run_map_scenario("invalid-local-affine")


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_not_modified_binary_layer_retains_cached_bytes_without_retries():
    _run_map_scenario("not-modified")
