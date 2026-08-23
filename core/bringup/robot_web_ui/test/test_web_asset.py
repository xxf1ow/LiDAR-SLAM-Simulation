import json
from pathlib import Path
import shutil
import subprocess
import textwrap

import pytest


HTML = (
    Path(__file__).parents[1]
    / "robot_web_ui"
    / "web"
    / "index.html"
)


def test_page_structure_matches_contextual_mobile_contract():
    source = HTML.read_text(encoding="utf-8")

    for element_id in (
        "mapCanvas", "statusStrip", "modeStatus", "motionStatus",
        "notice", "navigationStatus", "controlDock", "mapActions",
        "navigationActions", "manualControls", "modeToggle",
        "setInitialPose", "navigationAction", "confirmPlacement",
        "cancelPlacement", "speed", "linearVelocity",
        "angularVelocity", "feedbackState", "mapZoomIn",
        "mapZoomOut", "mapFit", "mapCenterRobot",
    ):
        assert source.count(f'id="{element_id}"') == 1
    assert source.count('data-direction="') == 4
    assert source.index('id="statusStrip"') < source.index('id="controlDock"')
    assert source.index('id="mapActions"') < source.index('id="modeToggle"')
    assert source.index('data-direction="forward"') < source.index(
        'data-direction="backward"'
    ) < source.index('data-direction="left"') < source.index(
        'data-direction="right"'
    )
    assert 'id="saveParkingPoint"' not in source
    assert 'id="goToParkingPoint"' not in source
    assert 'addEventListener("wheel"' not in source
    assert 'addEventListener("touch' not in source
    assert 'controlDock: document.getElementById("controlDock")' in source

    status_markup = source.split('id="statusStrip"', 1)[1].split(
        'id="controlDock"', 1
    )[0]
    manual_markup = source.split('id="manualControls"', 1)[1].split(
        'id="modeToggle"', 1
    )[0]
    assert "<button" not in status_markup
    status_css = source.split("#statusStrip {", 1)[1].split("}", 1)[0]
    assert 'id="speedValue"' not in status_markup
    assert 'id="speedValue"' in manual_markup
    assert "background: rgba(11, 18, 32, 0.68)" in status_css
    assert "color: #f2f6fb" in status_css
    assert '#feedbackState::before { content: "· "; }' in source
    assert "padding: 8px 10px" in status_css
    assert "border-radius: 10px" in status_css
    assert "text-shadow: -1px -1px 0 #fff" not in status_css
    assert "backdrop-filter" not in status_css
    assert "pointer-events: none" in status_css
    assert "safe-area-inset-bottom" in source
    assert "grid-template-columns: repeat(2, 1fr)" in source
    assert "min-height: 44px" in source
    assert '<input id="speed" type="range" min="0" max="100"' in source
    assert '<script src="/map_view.js"></script>' in source
    assert ":focus-visible" in source
    assert "prefers-reduced-motion: reduce" in source


def test_page_uses_neutral_api_without_vendor_surface():
    source = HTML.read_text(encoding="utf-8")

    assert 'post("/api/manual-command"' in source
    assert '"/api/takeover-manual"' in source
    assert '"/api/resume-automatic"' in source
    for forbidden in (
        "/api/status",
        "/api/driver",
        "RPM",
        "hardware feedback",
        "/driver",
        "/motor_speed",
        "/current_speed",
    ):
        assert forbidden.lower() not in source.lower()


NODE = shutil.which("node")


def _run_browser_scenario(scenario):
    source = HTML.read_text(encoding="utf-8")
    script = source.split("<script>", 1)[1].split("</script>", 1)[0]
    harness = textwrap.dedent(
        f"""
        const assert = require("assert");

        class FakeElement {{
          constructor(id = "") {{
            this.id = id;
            this.value = id === "speed" ? "20" : "";
                this.textContent = "";
                this.dataset = {{}};
                this.disabled = false;
                this.hidden = false;
                this.listeners = new Map();
            this.classList = {{
              values: new Set(),
              add: (...names) => names.forEach((name) => this.classList.values.add(name)),
              remove: (...names) => names.forEach((name) => this.classList.values.delete(name))
            }};
          }}
          addEventListener(name, callback) {{
            if (!this.listeners.has(name)) this.listeners.set(name, []);
            this.listeners.get(name).push(callback);
          }}
          emit(name, event = {{}}) {{
            const callbacks = this.listeners.get(name) || [];
            return Promise.all(callbacks.map((callback) => callback({{
              currentTarget: this,
              pointerId: 7,
              key: "",
              repeat: false,
              preventDefault() {{}},
              ...event
            }})));
          }}
            setPointerCapture() {{}}
            getBoundingClientRect() {{
              return {{left: 0, top: 0, width: 360, height: 640}};
            }}
        }}

        class FakeCanvas extends FakeElement {{
          constructor() {{
            super("mapCanvas");
            this.width = 360;
            this.height = 640;
            this.context = {{
              setTransform() {{}}, clearRect() {{}}, save() {{}}, restore() {{}},
              translate() {{}}, rotate() {{}}, scale() {{}}, fillRect() {{}},
              beginPath() {{}}, moveTo() {{}}, lineTo() {{}}, stroke() {{}}, fill() {{}}
            }};
          }}
          getContext() {{ return this.context; }}
        }}

        class FakeAbortController {{
          constructor() {{
            this.abortCount = 0;
            this.signal = {{aborted: false}};
          }}
          abort() {{
            this.abortCount += 1;
            this.signal.aborted = true;
          }}
        }}
        global.AbortController = FakeAbortController;

        const elements = new Map(
          ["speed", "speedValue", "notice", "modeToggle", "modeStatus",
           "motionStatus", "statusStrip", "controlDock", "mapActions",
           "navigationActions", "manualControls",
            "linearVelocity", "angularVelocity", "feedbackState",
            "mapCanvas", "mapZoomIn", "mapZoomOut",
            "mapFit", "mapCenterRobot"].map(
             (id) => [id, new FakeElement(id)]
           )
        );
        elements.set("mapCanvas", new FakeCanvas());
        const mapViewCalls = [];
        global.RobotMapView = {{
          create(options) {{
            assert.strictEqual(options.controlDock, elements.get("controlDock"));
            return {{
              cancelPlacement() {{
                mapViewCalls.push(["cancelPlacement"]);
                eventLog.push(["cancelPlacement"]);
              }},
              refreshViewport() {{ mapViewCalls.push(["refreshViewport"]); }}
            }};
          }}
        }};
        const directionButtons = ["forward", "backward", "left", "right"].map(
          (direction) => {{
            const button = new FakeElement(direction);
            button.dataset.direction = direction;
            button.disabled = true;
            return button;
          }}
        );
        const documentListeners = new Map();
        const windowListeners = new Map();
        global.document = {{
          hidden: false,
          getElementById(id) {{ return elements.get(id); }},
          querySelectorAll() {{ return directionButtons; }},
          addEventListener(name, callback) {{
            documentListeners.set(name, callback);
          }}
        }};
        global.window = {{
          addEventListener(name, callback) {{
            windowListeners.set(name, callback);
          }}
        }};

        let fakeNow = 0;
        global.performance = {{now: () => fakeNow}};
        const timers = [];
        global.setTimeout = (callback, milliseconds) => {{
          const timer = {{callback, milliseconds, canceled: false}};
          timers.push(timer);
          return timer;
        }};
        global.clearTimeout = (timer) => {{ timer.canceled = true; }};

        const requests = [];
        const eventLog = [];
        const pending = [];
        global.fetch = (path, options) => {{
          const request = {{
            path,
            body: JSON.parse(options.body),
            signal: options.signal,
            resolve: null,
            reject: null,
            settled: false
          }};
          requests.push(request);
          eventLog.push(["request", path]);
          return new Promise((resolve, reject) => {{
            request.resolve = ({{
              ok = true,
              status = 200,
              payload = {{ok: true, mode: "manual"}}
            }} = {{}}) => {{
              request.settled = true;
              resolve({{
                ok,
                status,
                async json() {{ return payload; }}
              }});
            }};
            request.reject = (error) => {{
              request.settled = true;
              reject(error);
            }};
            pending.push(request);
          }});
        }};

        async function flush() {{
          await Promise.resolve();
          await new Promise((resolve) => setImmediate(resolve));
        }}

        async function resolveNext(options) {{
          assert(pending.length > 0, "missing pending request");
          pending.shift().resolve(options);
          await flush();
        }}

        async function tick(expectedMilliseconds = 100) {{
          const timerIndex = timers.findIndex(
            (candidate) => !candidate.canceled
              && candidate.milliseconds === expectedMilliseconds
          );
          const timer = timerIndex === -1 ? null : timers.splice(timerIndex, 1)[0];
          if (timer === null) {{
            fakeNow += expectedMilliseconds;
            await flush();
            return;
          }}
          assert.strictEqual(timer.milliseconds, expectedMilliseconds);
          fakeNow += expectedMilliseconds;
          timer.callback();
          await flush();
        }}

        function advance(milliseconds) {{
          fakeNow += milliseconds;
        }}

        {script}

        async function runScenario() {{
          const scenario = {json.dumps(scenario)};
          await flush();
          assert.strictEqual(requests.length, 1);
          assert.strictEqual(requests[0].path, "/api/manual-session");
          assert.deepStrictEqual(requests[0].body, {{}});

          if (scenario === "contextual-layout") {{
            const modeStatus = elements.get("modeStatus");
            const navigationActions = elements.get("navigationActions");
            const manualControls = elements.get("manualControls");
            const modeToggle = elements.get("modeToggle");
            assert.strictEqual(modeToggle.hidden, false);

            applyMode("automatic");
            assert.strictEqual(modeStatus.textContent, "控制模式：自动导航");
            assert.strictEqual(navigationActions.hidden, false);
            assert.strictEqual(manualControls.hidden, true);
            assert.strictEqual(navigationActions.hidden && manualControls.hidden, false);

            applyMode("manual");
            assert.strictEqual(modeStatus.textContent, "控制模式：人工接管");
            assert.strictEqual(navigationActions.hidden, true);
            assert.strictEqual(manualControls.hidden, false);
            assert.strictEqual(navigationActions.hidden && manualControls.hidden, false);

            applyMode("unknown");
            assert.strictEqual(modeStatus.textContent, "控制模式：状态同步中");
            assert.strictEqual(navigationActions.hidden, true);
            assert.strictEqual(manualControls.hidden, true);

            applyMode("automatic");
            const pendingMode = modeToggle.emit("click");
            await flush();
            assert.strictEqual(modeStatus.textContent, "控制模式：切换中");
            assert.strictEqual(navigationActions.hidden, true);
            assert.strictEqual(manualControls.hidden, true);
            assert.strictEqual(modeToggle.hidden, false);
            assert(mapViewCalls.filter(
              (call) => call[0] === "refreshViewport"
            ).length >= 3);
            void pendingMode;
            return;
          }}

          if (scenario === "motion-feedback") {{
            applyMotionFeedback({{
              linear_x: 0.25,
              angular_z: -0.1,
              feedback_fresh: true
            }});
            assert.strictEqual(
              elements.get("linearVelocity").textContent,
              "0.25 m/s"
            );
            assert.strictEqual(
              elements.get("angularVelocity").textContent,
              "-0.10 rad/s"
            );
            assert.strictEqual(
              elements.get("feedbackState").textContent,
              ""
            );
            assert.strictEqual(elements.get("feedbackState").hidden, true);

            elements.get("speed").value = "35";
            await elements.get("speed").emit("input");
            assert.strictEqual(elements.get("speedValue").textContent, "35%");

            applyMotionFeedback({{
              linear_x: null,
              angular_z: null,
              feedback_fresh: false
            }});
            assert.strictEqual(
              elements.get("linearVelocity").textContent,
              "--"
            );
            assert.strictEqual(
              elements.get("angularVelocity").textContent,
              "--"
            );
            assert.strictEqual(
              elements.get("feedbackState").textContent,
              "底盘反馈中断"
            );
            assert.strictEqual(elements.get("feedbackState").hidden, false);

            await resolveNext({{
              payload: {{
                ok: true,
                session_id: "session-a",
                mode: "manual",
                linear_x: 0.25,
                angular_z: -0.1,
                feedback_fresh: true
              }}
            }});
            assert.strictEqual(
              elements.get("linearVelocity").textContent,
              "0.25 m/s"
            );
            assert.strictEqual(
              elements.get("angularVelocity").textContent,
              "-0.10 rad/s"
            );
            assert.strictEqual(
              elements.get("feedbackState").textContent,
              ""
            );
            assert.strictEqual(elements.get("feedbackState").hidden, true);
            return;
          }}

          if (scenario === "initial-and-authoritative-interlock") {{
            assert.strictEqual(currentMode, null);
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(elements.get("manualControls").hidden, true);
            assert.strictEqual(elements.get("modeToggle").disabled, true);
            assert.strictEqual(
              elements.get("modeToggle").textContent,
              "状态同步中…"
            );

            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "stop");
            documentListeners.get("keydown")({{
              key: "w",
              repeat: false,
              preventDefault() {{}}
            }});
            assert.strictEqual(desiredDirection, "stop");

            applyMode("manual");
            assert.strictEqual(currentMode, "manual");
            assert(directionButtons.every((button) => !button.disabled));
            assert.strictEqual(elements.get("manualControls").hidden, false);
            assert.strictEqual(elements.get("modeToggle").disabled, false);
            assert.strictEqual(
              elements.get("modeToggle").textContent,
              "恢复自动导航"
            );
            desiredDirection = "forward";
            heldMovementKeys.push("w");
            applyMode("automatic");
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(desiredDirection, "stop");
            assert.strictEqual(elements.get("manualControls").hidden, true);
            elements.get("notice").textContent = "自动导航状态通知";
            assert.strictEqual(elements.get("notice").hidden, false);
            assert.deepStrictEqual(heldMovementKeys, []);
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(elements.get("modeToggle").disabled, false);
            assert.strictEqual(
              elements.get("modeToggle").textContent,
              "人工接管"
            );

            applyMode("manual");
            desiredDirection = "forward";
            applyMode(null);
            assert.strictEqual(currentMode, null);
            assert.strictEqual(desiredDirection, "stop");
            assert.strictEqual(elements.get("manualControls").hidden, true);
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(elements.get("modeToggle").disabled, true);
            assert.strictEqual(
              elements.get("modeToggle").textContent,
              "状态同步中…"
            );
            return;
          }}

          await resolveNext({{
            payload: {{
              ok: true,
              session_id: "session-a",
              mode: "manual"
            }}
          }});
          assert.strictEqual(currentMode, "manual");
          assert(directionButtons.every((button) => !button.disabled));

          if (scenario === "sequenced-command-stream") {{
            elements.get("notice").textContent = "existing notice";
            noticeOwner = "mode";
            assert.deepStrictEqual(requests[1].body, {{
              session_id: "session-a",
              sequence: 1,
              direction: "stop",
              speed_percent: 20
            }});
            assert.strictEqual(
              timers.filter((timer) => !timer.canceled).length,
              2
            );

            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            elements.get("speed").value = "35";
            await elements.get("speed").emit("input");
            documentListeners.get("keydown")({{
              key: "w",
              repeat: false,
              preventDefault() {{}}
            }});
            assert.deepStrictEqual(heldMovementKeys, ["w"]);
            await directionButtons[0].emit("pointerup");
            assert.strictEqual(desiredDirection, "stop");
            await flush();
            assert.strictEqual(requests.length, 5);
            assert.deepStrictEqual(requests[2].body, {{
              session_id: "session-a",
              sequence: 2,
              direction: "forward",
              speed_percent: 20
            }});
            assert.deepStrictEqual(requests[3].body, {{
              session_id: "session-a",
              sequence: 3,
              direction: "forward",
              speed_percent: 35
            }});
            assert.deepStrictEqual(requests[4].body, {{
              session_id: "session-a",
              sequence: 4,
              direction: "stop",
              speed_percent: 35
            }});
            assert.deepStrictEqual(heldMovementKeys, []);
            assert.strictEqual(elements.get("notice").textContent, "existing notice");

            requests[4].resolve({{payload: {{
              ok: true, accepted: true, sequence: 4,
              last_sequence: 4, mode: "manual"
            }}}});
            await flush();
            requests[3].resolve({{payload: {{
              ok: true, accepted: true, sequence: 3,
              last_sequence: 3, mode: "manual"
            }}}});
            await flush();
            requests[2].resolve({{payload: {{
              ok: true, accepted: true, sequence: 2,
              last_sequence: 2, mode: "manual"
            }}}});
            await flush();
            assert.strictEqual(highestHandledSequence, 4);
            assert.strictEqual(elements.get("notice").textContent, "existing notice");

            await tick();
            assert.strictEqual(requests.at(-1).body.sequence, 5);
            requests.at(-1).resolve({{payload: {{
              ok: true, accepted: false, reason: "stale_sequence",
              sequence: 5, last_sequence: 5, mode: "manual"
            }}}});
            await flush();
            assert.strictEqual(highestHandledSequence, 5);
            assert.strictEqual(elements.get("notice").textContent, "existing notice");

            await tick(400);
            const timedOut = requests.find((request) => request.body.sequence === 1);
            assert(timedOut);
            timedOut.reject(new DOMException("timed out", "AbortError"));
            await flush();
            assert.strictEqual(timedOut.signal.aborted, true);

            await tick();
            assert.strictEqual(requests.at(-1).body.sequence, 6);
            const unresolvedCommand = requests.at(-1);
            await tick();
            const inactive = requests.at(-1);
            assert.strictEqual(inactive.body.sequence, 7);
            inactive.resolve({{
              ok: false,
              status: 409,
              payload: {{
                error: "inactive manual session",
                accepted: false,
                reason: "inactive_session",
                sequence: 7
              }}
            }});
            await flush();
            assert.strictEqual(commandLoopRunning, false);
            assert.strictEqual(manualSessionId, null);
            assert.strictEqual(
              timers.filter((timer) => !timer.canceled).length,
              0
            );
            assert.strictEqual(unresolvedCommand.settled, false);
            assert.strictEqual(unresolvedCommand.signal.aborted, true);
            assert.strictEqual(elements.get("notice").textContent, "existing notice");
            return;
          }}

          if (scenario === "all-stop-paths") {{
            const button = directionButtons[0];
            for (const eventName of [
              "pointerup", "pointercancel", "lostpointercapture", "blur"
            ]) {{
              await button.emit("pointerdown");
              assert.strictEqual(desiredDirection, "forward");
              await button.emit(eventName);
              assert.strictEqual(desiredDirection, "stop", eventName);
            }}
            await button.emit("keydown", {{key: "Enter"}});
            assert.strictEqual(desiredDirection, "forward");
            await button.emit("keyup", {{key: "Enter"}});
            assert.strictEqual(desiredDirection, "stop", "keyup");
            desiredDirection = "forward";
            windowListeners.get("blur")();
            assert.strictEqual(desiredDirection, "stop");
            desiredDirection = "forward";
            document.hidden = true;
            documentListeners.get("visibilitychange")();
            assert.strictEqual(desiredDirection, "stop");
            desiredDirection = "forward";
            windowListeners.get("pagehide")();
            assert.strictEqual(desiredDirection, "stop");
            return;
          }}

          if (scenario === "stale-button-events") {{
            await directionButtons[0].emit("pointerdown");
            await directionButtons[0].emit("pointerup");
            await directionButtons[1].emit("pointerdown");
            await directionButtons[0].emit("blur");
            assert.strictEqual(desiredDirection, "backward");
            await directionButtons[0].emit("lostpointercapture");
            assert.strictEqual(desiredDirection, "backward");
            await directionButtons[1].emit("blur");
            assert.strictEqual(desiredDirection, "stop");
            return;
          }}

          if (scenario === "authoritative-mode-button") {{
            const modeToggle = elements.get("modeToggle");
            applyMode(null);
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(modeToggle.textContent, "状态同步中…");
            const requestCount = requests.length;
            await modeToggle.emit("click");
            assert.strictEqual(requests.length, requestCount);

            applyMode("automatic");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "人工接管");
            mapViewCalls.length = 0;
            eventLog.length = 0;
            const takeoverPromise = modeToggle.emit("click");
            assert.deepStrictEqual(mapViewCalls[0], ["cancelPlacement"]);
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(desiredDirection, "stop");
            assert(directionButtons.every((button) => button.disabled));
            const cancelIndex = eventLog.findIndex((event) => event[0] === "cancelPlacement");
            const takeoverIndex = eventLog.findIndex(
              (event) => event[0] === "request" && event[1] === "/api/takeover-manual"
            );
            assert(cancelIndex >= 0);
            assert(takeoverIndex > cancelIndex);
            assert.strictEqual(requests.at(-1).path, "/api/takeover-manual");
            assert.strictEqual(requests.filter(
              (request) => request.path === "/api/takeover-manual"
            ).length, 1);
            const pendingRequestCount = requests.length;
            await modeToggle.emit("click");
            assert.strictEqual(requests.length, pendingRequestCount);
            pending.splice(
              pending.findIndex((request) =>
                request.path === "/api/takeover-manual"
              ),
              1
            )[0].resolve({{
              payload: {{ok: true, mode: "manual"}}
            }});
            await takeoverPromise;
            await flush();
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "恢复自动导航");
            assert(directionButtons.every((button) => !button.disabled));

            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            const resumePromise = modeToggle.emit("click");
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(desiredDirection, "stop");
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(requests.at(-1).path, "/api/resume-automatic");
            const delayedManual = pending[
              pending.findLastIndex((request) =>
                request.path === "/api/manual-command"
              )
            ];
            pending.splice(
              pending.findIndex((request) =>
                request.path === "/api/resume-automatic"
              ),
              1
            )[0].resolve({{
              payload: {{
                ok: true,
                mode: "automatic",
                linear_x: 0.4,
                angular_z: -0.2,
                feedback_fresh: true
              }}
            }});
            await resumePromise;
            await flush();
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "人工接管");
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(
              elements.get("linearVelocity").textContent,
              "0.40 m/s"
            );
            assert.strictEqual(
              elements.get("angularVelocity").textContent,
              "-0.20 rad/s"
            );
            assert.strictEqual(elements.get("feedbackState").hidden, true);

            delayedManual.resolve({{payload: {{
              ok: true,
              sequence: delayedManual.body.sequence,
              mode: "manual"
            }}}});
            await flush();
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(modeSwitchInProgress(), false);
            assert(directionButtons.every((button) => button.disabled));
            return;
          }}

          if (scenario === "manual-conflict-applies-mode") {{
            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            await tick();
            assert.strictEqual(requests.at(-1).body.direction, "forward");
            const conflictIndex = pending.findIndex(
              (request) => request.body.sequence === requests.at(-1).body.sequence
            );
            pending.splice(conflictIndex, 1)[0].resolve({{
              ok: false,
              status: 409,
              payload: {{
                error: "manual control is not active",
                mode: "automatic",
                linear_x: 0.25,
                angular_z: -0.1,
                feedback_fresh: true
              }}
            }});
            await flush();
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(desiredDirection, "stop");
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(
              elements.get("linearVelocity").textContent,
              "0.25 m/s"
            );
            assert.strictEqual(
              elements.get("angularVelocity").textContent,
              "-0.10 rad/s"
            );
            assert.strictEqual(
              elements.get("feedbackState").textContent,
              ""
            );
            assert.strictEqual(elements.get("feedbackState").hidden, true);
            return;
          }}

          if (scenario === "mode-pending-and-zero-convergence") {{
            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            const modeToggle = elements.get("modeToggle");
            const modePromise = modeToggle.emit("click");
            assert.strictEqual(desiredDirection, "stop");
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert.strictEqual(modeToggle.disabled, true);
            assert(directionButtons.every((button) => button.disabled));

            const modeIndex = pending.findIndex((request) =>
              request.path === "/api/resume-automatic"
            );
            pending.splice(modeIndex, 1)[0].resolve({{
              ok: true,
              status: 202,
              payload: {{
                ok: false,
                pending: true,
                error: "automatic resume unconfirmed",
                mode: "manual"
              }}
            }});
            await modePromise;
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              "切换结果尚未确认"
            );
            assert.notStrictEqual(
              elements.get("notice").textContent,
              "人工接管请求已完成"
            );
            assert.strictEqual(desiredDirection, "stop");
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert(directionButtons.every((button) => button.disabled));
            documentListeners.get("keydown")({{
              key: "w",
              repeat: false,
              preventDefault() {{}}
            }});
            assert.strictEqual(desiredDirection, "stop");

            await tick();
            assert.strictEqual(requests.at(-1).body.direction, "stop");
            const zeroRequest = pending.at(-1);
            zeroRequest.resolve({{
              payload: {{
                ok: true,
                sequence: zeroRequest.body.sequence,
                mode: "manual"
              }}
            }});
            await flush();
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert(directionButtons.every((button) => button.disabled));

            await tick();
            assert.strictEqual(requests.at(-1).body.direction, "stop");
            const convergenceRequest = pending.at(-1);
            convergenceRequest.resolve({{
              payload: {{
                ok: true,
                sequence: convergenceRequest.body.sequence,
                mode: "automatic"
              }}
            }});
            await flush();
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert(directionButtons.every((button) => button.disabled));
            return;
          }}

          if (scenario === "wasd-keyboard") {{
            const expectedDirections = new Map([
              ["w", "forward"],
              ["a", "left"],
              ["s", "backward"],
              ["d", "right"]
            ]);
            for (const [key, direction] of expectedDirections) {{
              documentListeners.get("keydown")({{
                key,
                repeat: false,
                preventDefault() {{}}
              }});
              assert.strictEqual(desiredDirection, direction, key);
              documentListeners.get("keyup")({{
                key,
                preventDefault() {{}}
              }});
              assert.strictEqual(desiredDirection, "stop", `${{key}} keyup`);
            }}

            documentListeners.get("keydown")({{
              key: "W",
              repeat: false,
              preventDefault() {{}}
            }});
            assert.strictEqual(desiredDirection, "forward");
            windowListeners.get("blur")();
            assert.strictEqual(desiredDirection, "stop", "window blur");

            documentListeners.get("keydown")({{
              key: "D",
              repeat: false,
              preventDefault() {{}}
            }});
            assert.strictEqual(desiredDirection, "right");
            document.hidden = true;
            documentListeners.get("visibilitychange")();
            assert.strictEqual(desiredDirection, "stop", "hidden document");
            return;
          }}

          if (scenario === "mode-notice-ownership") {{
            const modeToggle = elements.get("modeToggle");
            const resumePromise = modeToggle.emit("click");
            const modeIndex = pending.findIndex((request) =>
              request.path === "/api/resume-automatic"
            );
            assert.notStrictEqual(modeIndex, -1);
            pending.splice(modeIndex, 1)[0].resolve({{
              payload: {{ok: true, mode: "automatic"}}
            }});
            await resumePromise;
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              "恢复自动导航请求已完成"
            );

            await tick();
            const firstManualIndex = pending.findLastIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(firstManualIndex, 1)[0].resolve({{
              ok: false,
              status: 503,
              payload: {{error: "manual publisher unavailable"}}
            }});
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              "请求失败：manual publisher unavailable"
            );

            await tick();
            const nextManualIndex = pending.findLastIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(nextManualIndex, 1)[0].resolve({{
              payload: {{ok: true, mode: "automatic"}}
            }});
            await flush();
            assert.strictEqual(elements.get("notice").textContent, "");

            const takeoverPromise = modeToggle.emit("click");
            const takeoverIndex = pending.findIndex((request) =>
              request.path === "/api/takeover-manual"
            );
            pending.splice(takeoverIndex, 1)[0].resolve({{
              ok: false,
              status: 503,
              payload: {{error: "manual service unavailable"}}
            }});
            await takeoverPromise;
            await flush();
            const expectedNotice = "请求失败：manual service unavailable";
            assert.strictEqual(
              elements.get("notice").textContent,
              expectedNotice
            );
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "人工接管");
            assert(directionButtons.every((button) => button.disabled));

            await tick();
            const finalManualIndex = pending.findLastIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(finalManualIndex, 1)[0].resolve({{
              payload: {{ok: true, mode: "automatic"}}
            }});
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              expectedNotice
            );
            return;
          }}

          if (scenario === "wasd-rollover") {{
            const keyEvent = (key) => ({{
              key,
              repeat: false,
              preventDefault() {{}}
            }});
            documentListeners.get("keydown")(keyEvent("w"));
            assert.strictEqual(desiredDirection, "forward");
            documentListeners.get("keydown")(keyEvent("d"));
            assert.strictEqual(desiredDirection, "right");
            documentListeners.get("keyup")(keyEvent("d"));
            assert.strictEqual(desiredDirection, "forward");

            documentListeners.get("keydown")(keyEvent("a"));
            assert.strictEqual(desiredDirection, "left");
            documentListeners.get("keyup")(keyEvent("w"));
            assert.strictEqual(desiredDirection, "left");
            documentListeners.get("keyup")(keyEvent("a"));
            assert.strictEqual(desiredDirection, "stop");

            documentListeners.get("keydown")(keyEvent("w"));
            documentListeners.get("keydown")(keyEvent("d"));
            windowListeners.get("blur")();
            assert.strictEqual(desiredDirection, "stop");
            documentListeners.get("keyup")(keyEvent("d"));
            assert.strictEqual(desiredDirection, "stop");

            documentListeners.get("keydown")(keyEvent("w"));
            documentListeners.get("keydown")(keyEvent("d"));
            document.hidden = true;
            documentListeners.get("visibilitychange")();
            assert.strictEqual(desiredDirection, "stop");
            documentListeners.get("keyup")(keyEvent("d"));
            assert.strictEqual(desiredDirection, "stop");
            return;
          }}

          assert.fail(`unknown scenario: ${{scenario}}`);
        }}

        runScenario().catch((error) => {{
          console.error(error.stack || error);
          process.exitCode = 1;
        }});
        """
    )
    result = subprocess.run(
        [NODE, "-"],
        input=harness,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
@pytest.mark.parametrize(
    "scenario",
    [
        "contextual-layout",
        "initial-and-authoritative-interlock",
        "motion-feedback",
        "sequenced-command-stream",
        "all-stop-paths",
        "stale-button-events",
        "authoritative-mode-button",
        "manual-conflict-applies-mode",
        "mode-pending-and-zero-convergence",
        "wasd-keyboard",
        "mode-notice-ownership",
        "wasd-rollover",
    ],
)
def test_mobile_control_behavior(scenario):
    _run_browser_scenario(scenario)
