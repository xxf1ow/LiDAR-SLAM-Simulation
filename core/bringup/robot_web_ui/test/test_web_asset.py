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


def test_page_contains_only_the_required_mobile_controls():
    source = HTML.read_text(encoding="utf-8")

    assert source.count('<button disabled data-direction="') == 4
    for direction in ("forward", "backward", "left", "right"):
        assert f'data-direction="{direction}"' in source
    assert 'id="speed"' in source
    assert 'type="range"' in source
    assert 'min="0"' in source
    assert 'max="100"' in source
    assert (
        source.count(
            '<button id="modeToggle" disabled>状态同步中…</button>'
        )
        == 1
    )
    assert 'id="takeover"' not in source
    assert 'id="resume"' not in source
    assert 'id="linearVelocity"' in source
    assert 'id="angularVelocity"' in source
    assert 'id="feedbackState"' in source
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
        }}

        const elements = new Map(
          ["speed", "speedValue", "notice", "modeToggle",
           "linearVelocity", "angularVelocity", "feedbackState"].map(
             (id) => [id, new FakeElement(id)]
           )
        );
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

        const timers = [];
        global.setTimeout = (callback, milliseconds) => {{
          timers.push({{callback, milliseconds}});
          return timers.length;
        }};

        const requests = [];
        const pending = [];
        global.fetch = (path, options) => {{
          const request = {{
            path,
            body: JSON.parse(options.body),
            resolve: null
          }};
          requests.push(request);
          return new Promise((resolve) => {{
            request.resolve = ({{
              ok = true,
              status = 200,
              payload = {{ok: true, mode: "manual"}}
            }} = {{}}) => resolve({{
              ok,
              status,
              async json() {{ return payload; }}
            }});
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

        async function tick() {{
          assert(timers.length > 0, "missing command delay");
          const timer = timers.shift();
          assert.strictEqual(timer.milliseconds, 100);
          timer.callback();
          await flush();
        }}

        {script}

        async function runScenario() {{
          const scenario = {json.dumps(scenario)};
          await flush();
          assert.strictEqual(requests.length, 1);
          assert.deepStrictEqual(requests[0].body, {{
            direction: "stop",
            speed_percent: 20
          }});

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
              "底盘反馈正常"
            );

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

            await resolveNext({{
              payload: {{
                ok: true,
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
              "底盘反馈正常"
            );
            return;
          }}

          if (scenario === "initial-and-authoritative-interlock") {{
            assert.strictEqual(currentMode, null);
            assert(directionButtons.every((button) => button.disabled));
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
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(elements.get("modeToggle").disabled, true);
            assert.strictEqual(
              elements.get("modeToggle").textContent,
              "状态同步中…"
            );
            return;
          }}

          await resolveNext({{
            payload: {{ok: true, mode: "manual"}}
          }});
          assert.strictEqual(currentMode, "manual");
          assert(directionButtons.every((button) => !button.disabled));

          if (scenario === "single-flight-and-stop") {{
            commandLoop();
            await flush();
            assert.strictEqual(requests.length, 1);
            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            await tick();
            assert.strictEqual(requests.length, 2);
            assert.strictEqual(requests[1].body.direction, "forward");
            await directionButtons[0].emit("pointerup");
            assert.strictEqual(desiredDirection, "stop");
            await resolveNext();
            await tick();
            assert.strictEqual(requests[2].body.direction, "stop");
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
            const takeoverPromise = modeToggle.emit("click");
            assert.strictEqual(modeToggle.textContent, "切换中…");
            assert.strictEqual(modeToggle.disabled, true);
            assert.strictEqual(desiredDirection, "stop");
            assert(directionButtons.every((button) => button.disabled));
            assert.strictEqual(requests.at(-1).path, "/api/takeover-manual");
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
            pending.splice(
              pending.findIndex((request) =>
                request.path === "/api/resume-automatic"
              ),
              1
            )[0].resolve({{
              payload: {{ok: true, mode: "automatic"}}
            }});
            await resumePromise;
            await flush();
            assert.strictEqual(currentMode, "automatic");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "人工接管");
            assert(directionButtons.every((button) => button.disabled));
            return;
          }}

          if (scenario === "manual-conflict-applies-mode") {{
            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            await tick();
            assert.strictEqual(requests[1].body.direction, "forward");
            pending.shift().resolve({{
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
              "底盘反馈正常"
            );
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
            assert.strictEqual(currentMode, "manual");
            assert.strictEqual(modeToggle.disabled, false);
            assert.strictEqual(modeToggle.textContent, "恢复自动导航");

            await tick();
            assert.strictEqual(requests.at(-1).body.direction, "stop");
            const zeroIndex = pending.findIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(zeroIndex, 1)[0].resolve({{
              payload: {{ok: true, mode: "manual"}}
            }});
            await flush();
            assert.strictEqual(currentMode, "manual");
            assert(directionButtons.every((button) => !button.disabled));
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
            const firstManualIndex = pending.findIndex((request) =>
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
            const nextManualIndex = pending.findIndex((request) =>
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
            const finalManualIndex = pending.findIndex((request) =>
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
        "initial-and-authoritative-interlock",
        "motion-feedback",
        "single-flight-and-stop",
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
