# Agent Note: C++ Robot Web UI backend

Status: proposed

## Problem

An experimental branch moved raw odometry, localization, and TF traffic out of `rclpy`, reducing the server's idle CPU cost. Opening the page during an active navigation task still raised `robot_web_ui` from approximately 7–9% CPU to 25–30% on the Jetson.

Simulation isolates two remaining navigation-time inputs at approximately 50 Hz: `/behavior_tree_log` and NavigateToPose feedback. A Python ActionClient receives and deserializes feedback and wakes its executor even when application work is small; `/plan` can also make Python traverse and encode paths while navigation is active. Callback throttling after receipt cannot remove DDS take, Python message construction, or executor wake-up costs.

The Web server is a long-running control-plane process on the same board as SLAM and Nav2. Its backend CPU must fall substantially during active navigation so it does not compete materially with those workloads. The result matters more than meeting a fixed percentage threshold.

## Proposal

Convert `robot_web_ui` from `ament_python` to `ament_cmake` and implement its complete backend in C++17. The package will install one executable containing one ROS node named `robot_web_ui` and one cpp-httplib HTTP server. HTTP URLs, status codes, parking data, and binary map formats will remain compatible. The browser and JSON responses will drop measured linear/angular velocity feedback because the C++ node deliberately does not subscribe to odometry; all other fields remain compatible except for the navigation-phase simplification below.

The C++ node will consume `/gicp_localization/localization_snapshot` and both costmaps directly. It will not introduce `robot_web_ui_bridge`, `UiTelemetry`, `UiGrid`, or `UiLocalGrid`, and it will not subscribe to `/tf`, `/tf_static`, `/base_controller/odom`, `/localization`, or `/behavior_tree_log`.

All backend features will move to C++: static-map loading, localization state, global and local costmaps, path display, NavigateToPose goal and cancellation, initial-pose publication, manual velocity control, mode switching, parking-point CRUD and navigation, JSON sidecar persistence, HTTP routing, and static asset serving. No Python runtime or Python ROS node will remain in this package.

The experimental Bridge branch remains unmerged and serves only as a CPU baseline and implementation reference. This proposal starts from the common `master` baseline and replaces that experiment rather than depending on it.

## Runtime structure

The process will run a `rclcpp::executors::SingleThreadedExecutor` on the main thread. A listener thread will run cpp-httplib, whose built-in task queue will service concurrent requests. The application will not implement another thread pool, an application task queue, or a lock-free queue. The worker count will use the library's supported configuration rather than assuming that two fixed workers reduce CPU; sleeping workers are not the measured navigation-time cost.

ROS callbacks and HTTP workers will communicate through narrow thread-safe node operations. Small state uses ordinary mutexes whose critical sections cover only validation, state reservation, copying, or replacement. No lock is held while waiting for Nav2, a Service response, file I/O, or network I/O. Static map, costmap, and path bodies use immutable `shared_ptr<const BinarySnapshot>` values, allowing handlers to copy a pointer under lock and send the response after releasing it.

HTTP workers may initiate thread-safe ROS publish, Service, and Action operations. Action callbacks execute on the ROS executor and update the reserved goal generation. Mode-switch handlers may wait for the existing one-second Service outcome without holding shared state locks; unrelated HTTP workers remain available for manual control and state reads.

## ROS and snapshot behavior

With `navigation_sources_enabled=true`, the node will subscribe to the GICP localization snapshot, global and local costmaps, `/plan`, and `/cmd_vel_gate/mode`; it will create the NavigateToPose ActionClient, takeover and resume Service clients, and the existing command and initial-pose publishers. With the parameter set to `false`, it will not create navigation data subscriptions or the ActionClient, and navigation-dependent operations will report unavailable through the existing HTTP mapping.

Action feedback will only validate the current goal identity and update `distance_remaining`. It will not generate JSON, UI text, compressed data, or another ROS message. Costmaps and `/plan` will rebuild binary data, gzip content, revision, and ETag only when metadata or content changes. `/plan` will be processed only while a goal submitted through this node is active. HTTP polling will serialize small current-state projections and reuse prebuilt binary snapshots.

The navigation-state `motion` object and action-response `linear_x`, `angular_z`, and `feedback_fresh` fields will be absent. The assistant state will not report `feedback_unavailable`; map, localization, and navigation readiness remain its health inputs. The Web page will not show a measured-velocity row. Manual commands and the gate's command timeout remain unchanged.

## Navigation contract

Navigation states remain `idle`, `sending`, `navigating`, `canceling`, `succeeded`, `canceled`, and `failed`. `POST /api/navigation-goal` will retain map revision, bounds, automatic-mode, localization, Action Server, and active-goal checks. A successful asynchronous submission will return HTTP 202 with `goal_status: sending`; this response means that submission started, not that Nav2 accepted the goal. Parking-point navigation will resolve the stored pose and use the same goal path.

Only an accepted current goal can be canceled. A successfully submitted cancellation returns HTTP 202 with `goal_status: canceling`; a rejected or failed cancellation restores `navigating` and records the error. Nav2 results map to the existing succeeded, canceled, or failed terminal state.

Each goal will have a monotonically increasing generation and its Nav2 UUID. Goal, feedback, cancellation, and result callbacks must match the current identity before modifying state. Starting a goal and reaching a terminal state clear the displayed path. Initial-pose publication and active goal reservation remain mutually exclusive. A Nav2 goal submitted outside this node is not guaranteed to appear correctly because `/plan` carries no goal UUID.

The navigation-state `phase` field will remain present for JSON compatibility but will be `null`. The page will present only goal submission, navigation with optional remaining distance, cancellation, arrival, canceled, and failure text. BT-specific planning, path-following, clearing, spinning, waiting, and backing-up phases will not be exposed.

## Source and dependency boundaries

`web_ui_node.h/.cpp` will define the concrete root ROS capability and own subordinate resources. `http_server.h/.cpp` will own HTTP routing without exposing httplib types. `web_types.h` will contain ROS- and HTTP-independent value types. `map_snapshot.h/.cpp` will own map decoding, grid and path validation, binary encoding, change detection, gzip, and ETag generation. `parking_point_store.h/.cpp` will own parking-point validation and atomic sidecar replacement. Stateless one-use logic will remain translation-unit-local free functions rather than new manager or controller classes.

Third-party implementation dependencies are `cpp-httplib`, `nlohmann/json`, `yaml-cpp`, zlib, OpenSSL libcrypto for the existing SHA-256 ETag contract, and `tl::expected` for explicit fallible C++ interfaces. websocketpp, standalone Asio, OpenCV, and a lock-free queue library are not required. Third-party headers and types will not leak through the root node's public interface. The workspace-wide colcon defaults continue to select Release unless the caller explicitly supplies another `CMAKE_BUILD_TYPE`.

## Errors and lifecycle

HTTP errors retain the current 400 validation, 404 missing resource, 409 state conflict, 500 internal response failure, and 503 unavailable dependency semantics. A static-map or parking-sidecar error disables only dependent operations and remains visible in state; it does not terminate unrelated manual control or HTTP service. Invalid ROS input preserves the last valid snapshot and reports the associated layer error.

Startup must bind the HTTP port successfully before entering steady-state execution. Ordinary destruction will use RAII to stop the HTTP server, join its listener thread, and release the ROS node. The implementation will not add runtime recovery orchestration, a request-draining state machine, a watchdog, extensive shutdown tests, automatic navigation cancellation, or a control-mode transition during exit.

## Alternatives considered

**Retain Python and remove only `/behavior_tree_log`.** This removes one 50 Hz source but leaves NavigateToPose feedback deserialization and executor wake-ups in `rclpy`, so another high-cost Python path can remain or reappear as backend behavior evolves.

**Move navigation and other high-frequency ROS work to C++ while retaining Python HTTP and parking-point logic.** This requires a second private protocol and two processes while preserving Python's fixed runtime and synchronization boundary. Complete migration has a larger initial implementation surface but a clearer long-term ownership boundary and eliminates the class of Python ROS wake-up regressions.

**Use two fixed HTTP workers.** Two workers are sufficient for the expected page and serial voice-assistant traffic, but no evidence shows that replacing cpp-httplib's configuration with that fixed number materially reduces CPU. Retaining library-managed workers also preserves the existing requirement that manual control remain responsive while another request is waiting.

**Use a lock-free queue between HTTP and ROS.** The workload has low command volume, small state, and no measured mutex contention. A lock-free protocol would add ownership, cancellation, and shutdown complexity without addressing DDS deserialization or executor wake-ups.

## Acceptance criteria

- `robot_web_ui` builds as an `ament_cmake` C++17 package and installs one backend executable; its Python backend is absent, and no Bridge package or messages are introduced.
- The existing browser assets, HTTP paths, response fields, status codes, parking-point sidecar semantics, and binary map formats remain compatible except for the explicitly simplified navigation phase text.
- Focused tests cover snapshot content changes, parking-point persistence, navigation generations and terminal transitions, representative HTTP success and failure paths, and package topology. The migration does not reproduce every Python edge-case test.
- A full `sim + navigation` run through the formal bringup entry verifies localization, maps, path display, manual endpoints, and successful navigation while recording Web UI and total CPU for closed-page, open-idle, and active-navigation states.
- Under the same simulation scenario, active navigation does not produce the sustained approximately 25–30% Web UI CPU observed with the Python backend and shows a substantial reduction from the recorded baseline. If it still rises materially, the CPU problem remains unresolved regardless of build and unit-test results.
- Every simulation run terminates its launch and leaves no ROS, Gazebo, Nav2, GICP, or Web UI process behind; available memory recovers after cleanup.

## Risks

The migration replaces mature Python behavior across HTTP, ROS Action, binary encoding, and persistent parking data, so compatibility regressions are possible despite a deliberately focused test set. The design limits this risk by preserving external formats and testing representative real entry paths rather than introducing a new protocol.

cpp-httplib workers and the ROS executor access one node concurrently. Short mutex-protected operations and immutable snapshots make that concurrency explicit, but blocking a worker while holding shared state would reintroduce latency and deadlock risk.

The UI gives up Nav2 Behavior Tree phase detail. Operators retain the goal lifecycle, remaining distance, path, and final outcome, which are the information required for the Web control surface. Full tracking of goals created outside this node remains unsupported.
