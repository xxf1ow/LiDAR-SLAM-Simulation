import json
from pathlib import Path
import shutil
import subprocess
import textwrap

import pytest


HTML = (
    Path(__file__).parents[1]
    / "can_driver_web_control"
    / "web"
    / "index.html"
)


def test_mobile_page_contains_required_controls_and_timing():
    source = HTML.read_text(encoding="utf-8")
    for element_id in (
        'id="speed"',
        'data-direction="forward"',
        'data-direction="backward"',
        'data-direction="left"',
        'data-direction="right"',
        'id="enable"',
        'id="disable"',
        'id="feedback"',
        'id="hardwareFeedback"',
    ):
        assert element_id in source
    assert 'min="0"' in source
    assert 'max="100"' in source
    assert 'value="20"' in source
    assert "setInterval(refreshCommand, 100)" in source
    assert "setInterval(refreshStatus, 500)" in source
    assert "pointercancel" in source
    assert "visibilitychange" in source
    assert "pagehide" in source
    assert 'addEventListener("blur"' in source
    assert "使能请求" in source
    assert "SDK 不提供使能成功/失败反馈" in source


def function_body(source, name, next_marker):
    function_marker = f"function {name}"
    assert function_marker in source
    start = source.index(function_marker)
    assert next_marker in source[start:]
    return source[start : source.index(next_marker, start)]


def assert_in_order(source, *fragments):
    cursor = 0
    for fragment in fragments:
        assert fragment in source[cursor:]
        cursor = source.index(fragment, cursor) + len(fragment)


def test_pointer_motion_has_single_owner_and_generation():
    source = HTML.read_text(encoding="utf-8")
    begin = function_body(source, "beginMotion", "function endPointerMotion")
    end = function_body(source, "endPointerMotion", "function beginKeyboardMotion")

    assert "let activePointerId = null;" in source
    assert "let motionGeneration = 0;" in source
    assert "if (activeButton !== null) return;" in begin
    assert "activePointerId = event.pointerId;" in begin
    assert "if (event.pointerId !== activePointerId) return;" in end
    assert "++motionGeneration;" in source
    assert 'button.addEventListener("pointerup", endPointerMotion);' in source
    assert 'button.addEventListener("lostpointercapture", endPointerMotion);' in source


def test_periodic_motion_has_only_one_in_flight_request():
    source = HTML.read_text(encoding="utf-8")
    refresh = function_body(source, "refreshCommand", "function clearActiveMotion")

    assert "let motionRequest = null;" in source
    assert "let stopBarrier = null;" in source
    assert "motionRequest !== null" in refresh
    assert "stopBarrier !== null" in refresh
    assert 'const request = post("/api/command"' in refresh
    assert "motionRequest = request;" in refresh
    assert "if (motionRequest === request) motionRequest = null;" in refresh


def test_stop_is_immediate_and_reasserted_after_in_flight_motion():
    source = HTML.read_text(encoding="utf-8")
    stop = function_body(source, "stopMotion", "document.querySelectorAll")

    assert_in_order(
        stop,
        "const inFlightMotion = motionRequest;",
        "++motionGeneration;",
        "const immediateStop = sendStop();",
        "Promise.allSettled([immediateStop, inFlightMotion, previousStopBarrier])",
        "if (inFlightMotion !== null) await sendStop();",
    )


def test_stale_motion_errors_cannot_stop_or_relabel_a_new_gesture():
    source = HTML.read_text(encoding="utf-8")
    refresh = function_body(source, "refreshCommand", "function clearActiveMotion")
    assert_in_order(
        refresh,
        ".catch((error)",
        "if (generation !== motionGeneration) return;",
        "notice.textContent = error.message;",
        "stopMotion();",
    )


def test_stop_failure_uses_one_retry_and_names_watchdog_fallback():
    source = HTML.read_text(encoding="utf-8")
    send_stop = function_body(source, "sendStop", "function stopMotion")

    assert "for (let attempt = 0; attempt < 2; attempt += 1)" in send_stop
    assert "后端看门狗将自动停车" in send_stop
    assert ".catch(() => {})" not in send_stop


def test_direction_buttons_support_keyboard_hold():
    source = HTML.read_text(encoding="utf-8")
    assert 'button.addEventListener("keydown", beginKeyboardMotion);' in source
    assert 'button.addEventListener("keyup", endKeyboardMotion);' in source
    assert 'event.key !== " " && event.key !== "Enter"' in source
    assert "event.repeat" in source


def test_keyboard_blur_stops_only_the_current_keyboard_gesture():
    source = HTML.read_text(encoding="utf-8")
    clear = function_body(source, "clearActiveMotion", "function beginDirectionMotion")
    begin = function_body(source, "beginKeyboardMotion", "function endKeyboardMotion")
    blur = function_body(source, "stopKeyboardMotionOnBlur", "async function sendStop")

    assert "let activeKeyboardButton = null;" in source
    assert "activeKeyboardButton = null;" in clear
    assert_in_order(
        begin,
        "if (activeButton !== null) return;",
        "activeKeyboardButton = event.currentTarget;",
        "beginDirectionMotion(event.currentTarget);",
    )
    assert_in_order(
        blur,
        "activeKeyboardButton !== event.currentTarget",
        "activeButton !== event.currentTarget",
        "activePointerId !== null",
        ") return;",
        "stopMotion();",
    )
    assert (
        'button.addEventListener("blur", stopKeyboardMotionOnBlur);'
        in source
    )


def test_status_checks_http_success_before_rendering():
    source = HTML.read_text(encoding="utf-8")
    refresh = function_body(source, "refreshStatus", "document.addEventListener")

    assert_in_order(
        refresh,
        'await fetch("/api/status", {cache: "no-store"})',
        "if (!response.ok)",
        "await response.json()",
        'document.getElementById("command").textContent',
    )


def test_driver_intents_share_generation_and_stop_barriers():
    source = HTML.read_text(encoding="utf-8")
    control = function_body(source, "setDriverEnabled", "async function refreshStatus")

    assert "let driverGeneration = 0;" in source
    assert "let driverRequest = null;" in source
    assert "let driverBarrier = null;" in source
    assert_in_order(
        control,
        "const generation = ++driverGeneration;",
        "const previousDriverBarrier = driverBarrier;",
        "if (!enabled) stopMotion();",
        "const currentStopBarrier = stopBarrier;",
        "const previousDriverRequest = driverRequest;",
        "const immediateDisable = enabled ? null : trackDriverRequest(false);",
        "await Promise.allSettled([",
        "previousDriverBarrier,",
        "currentStopBarrier,",
        "previousDriverRequest,",
        "immediateDisable,",
        "]);",
        "if (generation !== driverGeneration) return;",
        "await trackDriverRequest(enabled);",
    )
    assert control.count("if (generation !== driverGeneration) return;") >= 3
    assert "if (driverBarrier === barrier) driverBarrier = null;" in control


NODE = shutil.which("node")


def _run_driver_scenario(scenario):
    script = HTML.read_text(encoding="utf-8").split("<script>", 1)[1].split(
        "</script>", 1
    )[0]
    harness = textwrap.dedent(
        f"""
        const assert = require("assert");

        class FakeElement {{
          constructor(id = "") {{
            this.id = id;
            this.value = id === "speed" ? "20" : "";
            this.textContent = "";
            this.className = "";
            this.dataset = {{}};
            this.listeners = new Map();
            this.classList = {{add() {{}}, remove() {{}}}};
          }}
          addEventListener(name, callback) {{
            if (!this.listeners.has(name)) this.listeners.set(name, []);
            this.listeners.get(name).push(callback);
          }}
          emit(name, event = {{}}) {{
            const callbacks = this.listeners.get(name) || [];
            return Promise.all(callbacks.map((callback) => callback({{
              currentTarget: this,
              pointerId: 1,
              key: "",
              repeat: false,
              preventDefault() {{}},
              ...event
            }})));
          }}
          setPointerCapture() {{}}
        }}

        const elements = new Map([
          "speed", "speedValue", "notice", "connection", "hardwareFeedback", "driver",
          "command", "feedback", "enable", "disable"
        ].map((id) => [id, new FakeElement(id)]));
        const directionButtons = ["forward", "backward", "left", "right"]
          .map((direction) => {{
            const button = new FakeElement(direction);
            button.dataset.direction = direction;
            return button;
          }});
        global.document = {{
          hidden: false,
          getElementById(id) {{ return elements.get(id); }},
          querySelectorAll() {{ return directionButtons; }},
          addEventListener() {{}}
        }};
        global.window = {{addEventListener() {{}}}};
        global.setInterval = () => 1;
        global.clearInterval = () => {{}};

        let nextRequestId = 1;
        const pending = [];
        const events = [];
        let serverEnabled = false;
        const okResponse = (payload = {{}}) => ({{
          ok: true,
          status: 200,
          async json() {{ return payload; }}
        }});
        global.fetch = (path, options = {{}}) => {{
          if (path === "/api/status") {{
            events.push({{kind: "status"}});
            return Promise.resolve(okResponse({{
              driver_connected: true,
              hardware_feedback: false,
              enabled: serverEnabled,
              command: [0, 0],
              feedback: null
            }}));
          }}
          const body = JSON.parse(options.body);
          const request = {{id: nextRequestId++, path, body}};
          events.push({{kind: "sent", ...request}});
          return new Promise((resolve) => pending.push({{...request, resolve}}));
        }};

        const flush = async () => {{
          await Promise.resolve();
          await new Promise((resolve) => setImmediate(resolve));
        }};
        const findPending = (predicate) => {{
          const index = pending.findIndex(predicate);
          assert.notStrictEqual(index, -1, `missing pending request; events=${{JSON.stringify(events)}}`);
          return pending.splice(index, 1)[0];
        }};
        const resolveRequest = async (predicate) => {{
          const request = findPending(predicate);
          if (request.path === "/api/driver") serverEnabled = request.body.enabled;
          events.push({{kind: "resolved", id: request.id, path: request.path, body: request.body}});
          request.resolve(okResponse());
          await flush();
          return request;
        }};
        const sentDrivers = () => events.filter((event) =>
          event.kind === "sent" && event.path === "/api/driver"
        );

        {script}

        async function runScenario() {{
          const scenario = {json.dumps(scenario)};
          await flush();
          if (scenario === "stale-enable") {{
            const enableDone = elements.get("enable").emit("click");
            await flush();
            const disableDone = elements.get("disable").emit("click");
            await flush();
            await resolveRequest((request) =>
              request.path === "/api/driver" && request.body.enabled === false
            );
            await resolveRequest((request) => request.path === "/api/command");
            await resolveRequest((request) =>
              request.path === "/api/driver" && request.body.enabled === true
            );
            const finalDisable = findPending((request) =>
              request.path === "/api/driver" && request.body.enabled === false
            );
            finalDisable.resolve(okResponse());
            serverEnabled = false;
            await Promise.all([enableDone, disableDone]);
            await flush();
            assert.strictEqual(serverEnabled, false);
            const drivers = sentDrivers();
            assert.strictEqual(drivers[drivers.length - 1].body.enabled, false);
            return;
          }}

          if (scenario === "enable-waits-disable") {{
            const disableDone = elements.get("disable").emit("click");
            await flush();
            const enableDone = elements.get("enable").emit("click");
            await flush();
            assert.strictEqual(sentDrivers().some((event) => event.body.enabled), false);
            await resolveRequest((request) =>
              request.path === "/api/driver" && request.body.enabled === false
            );
            await resolveRequest((request) => request.path === "/api/command");
            const enableRequest = findPending((request) =>
              request.path === "/api/driver" && request.body.enabled === true
            );
            enableRequest.resolve(okResponse());
            serverEnabled = true;
            await Promise.all([disableDone, enableDone]);
            await flush();
            assert.strictEqual(serverEnabled, true);
            return;
          }}

          if (scenario === "motion-stops-before-enable") {{
            const firstEnable = elements.get("enable").emit("click");
            await flush();
            await resolveRequest((request) =>
              request.path === "/api/driver" && request.body.enabled === true
            );
            await firstEnable;
            const motionDone = directionButtons[0].emit("pointerdown", {{pointerId: 7}});
            await flush();
            const disableDone = elements.get("disable").emit("click");
            await flush();
            const enableDone = elements.get("enable").emit("click");
            await flush();
            assert.strictEqual(sentDrivers().filter((event) => event.body.enabled).length, 1);
            await resolveRequest((request) =>
              request.path === "/api/driver" && request.body.enabled === false
            );
            await resolveRequest((request) =>
              request.path === "/api/command" && request.body.direction === "stop"
            );
            assert.strictEqual(sentDrivers().filter((event) => event.body.enabled).length, 1);
            await resolveRequest((request) =>
              request.path === "/api/command" && request.body.direction === "forward"
            );
            const finalStop = findPending((request) =>
              request.path === "/api/command" && request.body.direction === "stop"
            );
            assert.strictEqual(sentDrivers().filter((event) => event.body.enabled).length, 1);
            finalStop.resolve(okResponse());
            events.push({{kind: "resolved", id: finalStop.id, path: finalStop.path, body: finalStop.body}});
            await flush();
            const secondEnable = findPending((request) =>
              request.path === "/api/driver" && request.body.enabled === true
            );
            const finalStopResolved = events.findIndex((event) =>
              event.kind === "resolved" && event.id === finalStop.id
            );
            const secondEnableSent = events.findIndex((event) =>
              event.kind === "sent" && event.id === secondEnable.id
            );
            assert(finalStopResolved < secondEnableSent, JSON.stringify(events));
            secondEnable.resolve(okResponse());
            await Promise.all([motionDone, disableDone, enableDone]);
            await flush();
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


@pytest.mark.skipif(NODE is None, reason="Node.js is required for browser race tests")
def test_stale_enable_completion_cannot_overturn_disable():
    _run_driver_scenario("stale-enable")


@pytest.mark.skipif(NODE is None, reason="Node.js is required for browser race tests")
def test_enable_waits_until_disable_barrier_finishes():
    _run_driver_scenario("enable-waits-disable")


@pytest.mark.skipif(NODE is None, reason="Node.js is required for browser race tests")
def test_late_motion_is_stopped_before_reenable_is_sent():
    _run_driver_scenario("motion-stops-before-enable")
