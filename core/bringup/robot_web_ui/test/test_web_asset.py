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

    assert source.count('<button data-direction="') == 4
    for direction in ("forward", "backward", "left", "right"):
        assert f'data-direction="{direction}"' in source
    assert 'id="speed"' in source
    assert 'type="range"' in source
    assert 'min="0"' in source
    assert 'max="100"' in source
    assert "人工接管" in source
    assert "恢复自动导航" in source
    assert ":focus-visible" in source
    assert "prefers-reduced-motion: reduce" in source


def test_page_uses_neutral_api_and_has_no_hardware_feedback_surface():
    source = HTML.read_text(encoding="utf-8")

    assert 'post("/api/manual-command"' in source
    assert 'requestMode("/api/takeover-manual"' in source
    assert 'requestMode("/api/resume-automatic"' in source
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
          ["speed", "speedValue", "notice", "takeover", "resume"].map(
            (id) => [id, new FakeElement(id)]
          )
        );
        const directionButtons = ["forward", "backward", "left", "right"].map(
          (direction) => {{
            const button = new FakeElement(direction);
            button.dataset.direction = direction;
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
              payload = {{ok: true}}
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

        async function resolveNext() {{
          assert(pending.length > 0, "missing pending request");
          pending.shift().resolve();
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

          if (scenario === "single-flight-and-stop") {{
            commandLoop();
            await flush();
            assert.strictEqual(requests.length, 1);
            await directionButtons[0].emit("pointerdown");
            assert.strictEqual(desiredDirection, "forward");
            await resolveNext();
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
              desiredDirection = "forward";
              await button.emit(eventName);
              assert.strictEqual(desiredDirection, "stop", eventName);
            }}
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

          if (scenario === "idle-and-mode-actions") {{
            await resolveNext();
            await tick();
            assert.strictEqual(requests[1].body.direction, "stop");
            await elements.get("takeover").emit("click");
            await elements.get("resume").emit("click");
            assert(requests.some((request) =>
              request.path === "/api/takeover-manual"
            ));
            assert(requests.some((request) =>
              request.path === "/api/resume-automatic"
            ));
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
            await elements.get("takeover").emit("click");
            await flush();
            const modeIndex = pending.findIndex((request) =>
              request.path === "/api/takeover-manual"
            );
            assert.notStrictEqual(modeIndex, -1);
            pending.splice(modeIndex, 1)[0].resolve({{
              ok: false,
              status: 503,
              payload: {{error: "manual service unavailable"}}
            }});
            await flush();
            const expectedNotice = "请求失败：manual service unavailable";
            assert.strictEqual(
              elements.get("notice").textContent,
              expectedNotice
            );

            const firstManualIndex = pending.findIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(firstManualIndex, 1)[0].resolve();
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              expectedNotice
            );

            await tick();
            const nextManualIndex = pending.findIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(nextManualIndex, 1)[0].resolve({{
              ok: false,
              status: 503,
              payload: {{error: "manual publisher unavailable"}}
            }});
            await flush();
            assert.strictEqual(
              elements.get("notice").textContent,
              expectedNotice
            );

            await tick();
            const finalManualIndex = pending.findIndex((request) =>
              request.path === "/api/manual-command"
            );
            pending.splice(finalManualIndex, 1)[0].resolve();
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
        "single-flight-and-stop",
        "all-stop-paths",
        "idle-and-mode-actions",
        "wasd-keyboard",
        "mode-notice-ownership",
        "wasd-rollover",
    ],
)
def test_mobile_control_behavior(scenario):
    _run_browser_scenario(scenario)
