from pathlib import Path
import shutil
import subprocess
import textwrap

import pytest


NODE = shutil.which("node")
VIEW = Path(__file__).parents[1] / "web" / "tracking_view.js"


@pytest.mark.skipif(NODE is None, reason="Node.js is required")
def test_tracking_view_renders_snapshot_and_selects_base_footprint_target():
    script = textwrap.dedent(
        f"""
        const assert = require("assert");
        const fs = require("fs");
        const vm = require("vm");
        const calls = [];
        const listeners = new Map();
        const context = {{
          setTransform() {{}}, clearRect() {{}}, beginPath() {{}},
          arc(...args) {{ calls.push(["arc", ...args]); }},
          fill() {{}}, stroke() {{}}
        }};
        const canvas = {{
          width: 200, height: 100,
          getContext() {{ return context; }},
          getBoundingClientRect() {{ return {{left: 0, top: 0, width: 200, height: 100}}; }},
          addEventListener(name, callback) {{ listeners.set(name, callback); }}
        }};
        const sent = [];
        const errors = [];
        let successes = 0;
        vm.runInThisContext(fs.readFileSync({str(VIEW)!r}, "utf8"));
        const view = RobotTrackingView.create({{
          canvas,
          request: async (path, body) => {{
            sent.push([path, body]);
            if (sent.length === 1) throw new Error("subscriber unavailable");
          }},
          onSelectionError: (error) => errors.push(error.message),
          onSelectionSuccess: () => {{ successes += 1; }}
        }});
        view.setState({{
          frame_id: "base_footprint", stamp: {{sec: 2, nanosec: 0}},
          active: true, target: {{x: 0.4, y: -0.2}}, points: [[1, 0], [0, 1]]
        }});
        assert(calls.some((call) => call[0] === "arc" &&
          call[1] === 120 && call[2] === 10 && call[3] === 30));
        assert(calls.some((call) => call[0] === "arc" &&
          call[1] === 0 && call[2] === 50 && call[3] === 2));
        assert(calls.some((call) => call[0] === "arc" && call[1] === 100 && call[2] === 50));
        (async () => {{
          await listeners.get("dblclick")({{clientX: 100, clientY: 50}});
          assert.deepStrictEqual(errors, ["subscriber unavailable"]);
          assert.strictEqual(successes, 0);
          await listeners.get("dblclick")({{clientX: 100, clientY: 0}});
          assert.strictEqual(successes, 1);
          assert.deepStrictEqual(sent, [
            ["/api/tracking-target", {{x: 0, y: 0}}],
            ["/api/tracking-target", {{x: 0.5, y: 0}}]
          ]);
        }})().catch((error) => {{ console.error(error); process.exitCode = 1; }});
        """
    )
    result = subprocess.run([NODE, "-"], input=script, text=True,
                            capture_output=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
