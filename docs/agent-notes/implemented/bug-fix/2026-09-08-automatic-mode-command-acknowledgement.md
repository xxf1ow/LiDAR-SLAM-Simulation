# Agent Note: Automatic mode command acknowledgement

Status: implemented

## Problem

The browser starts a 100 ms manual-command loop after creating its control session. In automatic mode, each successful command response reapplied the authoritative mode, invoked the safety stop path, and immediately dispatched another stop command. Each response therefore created its successor without waiting for the timer, driving the Python HTTP and ROS publish paths continuously until the mode changed to manual.

## Decision

Mode application dispatches a safety stop only when the observed mode actually leaves `manual`. Repeated `automatic` acknowledgements update authoritative state and controls without dispatching another command. The 100 ms command loop remains the sole scheduler for unchanged stop commands.

Explicit stop paths still dispatch zero commands for operator mode changes, pointer and keyboard release, focus loss, document hiding, and page exit. An authoritative response that moves the UI from `manual` to another mode also sends one zero command and cannot form a response loop because the next acknowledgement observes no mode transition.

## Alternatives considered

**Always suppress stop dispatch while applying response modes.** This prevents the loop but does not send a final zero command when an authoritative command response reveals that the UI has left manual mode.

**Ignore forced stop dispatch when the desired direction is already `stop`.** This weakens explicit safety paths that intentionally resend zero commands even when local state already says stopped.

## Verification

The browser harness starts directly in automatic mode, resolves one manual-command acknowledgement, and verifies that no command is dispatched until the 100 ms timer fires. Restoring the unconditional non-manual stop condition makes this scenario fail because the acknowledgement immediately creates a second command.

## Consequences

Automatic mode retains the fixed-rate manual heartbeat, navigation-state polling, and revision-based map retrieval without an unbounded request chain. Manual takeover behavior, explicit zero commands, HTTP payloads, ROS topics, and server contracts remain unchanged. Runtime CPU reduction requires measurement after deploying the rebuilt Web UI package.
