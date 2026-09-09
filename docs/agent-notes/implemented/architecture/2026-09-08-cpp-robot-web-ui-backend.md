# Agent Note: C++ Robot Web UI backend

Status: implemented

## Problem

An experimental branch moved raw odometry, localization, and TF traffic out of `rclpy`, reducing the server's idle CPU cost. Opening the page during an active navigation task still raised `robot_web_ui` from approximately 7–9% CPU to 25–30% on the Jetson.

Simulation isolates two remaining navigation-time inputs at approximately 50 Hz: `/behavior_tree_log` and NavigateToPose feedback. A Python ActionClient receives and deserializes feedback and wakes its executor even when application work is small; `/plan` can also make Python traverse and encode paths while navigation is active. Callback throttling after receipt cannot remove DDS take, Python message construction, or executor wake-up costs.

The Web server is a long-running control-plane process on the same board as SLAM and Nav2. Its backend CPU must fall substantially during active navigation so it does not compete materially with those workloads. The result matters more than meeting a fixed percentage threshold.

## Decision

`robot_web_ui` is an `ament_cmake` C++17 package. It installs one executable containing one ROS node named `robot_web_ui` and one cpp-httplib HTTP server. HTTP URLs, status codes, parking data, and binary map formats remain compatible. The browser and JSON responses omit measured linear/angular velocity feedback because the C++ node deliberately does not subscribe to odometry; all other fields remain compatible except for the navigation-phase simplification below.

The C++ node consumes `/gicp_localization/localization_snapshot` and both costmaps directly. The runtime contains no `robot_web_ui_bridge`, `UiTelemetry`, `UiGrid`, or `UiLocalGrid`, and the node does not subscribe to `/tf`, `/tf_static`, `/base_controller/odom`, `/localization`, or `/behavior_tree_log`.

All backend features reside in C++: static-map loading, localization state, global and local costmaps, path display, NavigateToPose goal and cancellation, initial-pose publication, manual velocity control, mode switching, parking-point CRUD and navigation, JSON sidecar persistence, HTTP routing, and static asset serving. The package contains no Python runtime or Python ROS node.

The experimental Bridge branch remains unmerged and serves only as historical CPU evidence. The shipped implementation does not depend on it.

## Runtime structure

The process runs a `rclcpp::executors::SingleThreadedExecutor` on the main thread. A listener thread runs cpp-httplib, whose built-in task queue services concurrent requests. The application does not implement another thread pool, application task queue, or lock-free queue. The worker count uses the library's supported configuration; sleeping workers are not the measured navigation-time cost.

ROS callbacks and HTTP workers communicate through narrow thread-safe node operations. Small state uses ordinary mutexes whose critical sections cover only validation, state reservation, copying, or replacement. No lock is held while waiting for Nav2, a Service response, file I/O, or network I/O. Static map, costmap, and path bodies use immutable `shared_ptr<const BinarySnapshot>` values, allowing handlers to copy a pointer under lock and send the response after releasing it.

HTTP workers may initiate thread-safe ROS publish, Service, and Action operations. Action callbacks execute on the ROS executor and update the reserved goal generation. Mode-switch handlers may wait for the one-second Service outcome without holding shared state locks; unrelated HTTP workers remain available for manual control and state reads.

## ROS and snapshot behavior

With `navigation_sources_enabled=true`, the node subscribes to the GICP localization snapshot, global and local costmaps, `/plan`, and `/cmd_vel_gate/mode`; it creates the NavigateToPose ActionClient, takeover and resume Service clients, and the existing command and initial-pose publishers. With the parameter set to `false`, it omits navigation data subscriptions and the ActionClient, and navigation-dependent operations report unavailable through the existing HTTP mapping.

Action feedback only validates the current goal identity and updates `distance_remaining`. It does not generate JSON, UI text, compressed data, or another ROS message. Costmaps and `/plan` rebuild binary data, gzip content, revision, and ETag only when metadata or content changes. The node processes `/plan` only while a goal submitted through this node is active. HTTP polling serializes small current-state projections and reuses prebuilt binary snapshots.

The navigation-state `motion` object and action-response `linear_x`, `angular_z`, and `feedback_fresh` fields are absent. The assistant state does not report `feedback_unavailable`; map, localization, and navigation readiness are its health inputs. The Web page has no measured-velocity row. Manual commands and the gate's command timeout remain unchanged.

## Navigation contract

Navigation states are `idle`, `sending`, `navigating`, `canceling`, `succeeded`, `canceled`, and `failed`. `POST /api/navigation-goal` retains map revision, bounds, automatic-mode, localization, Action Server, and active-goal checks. A successful asynchronous submission returns HTTP 202 with `goal_status: sending`; this response means that submission started, not that Nav2 accepted the goal. A rejection or send failure for the current submission transitions that generation to `failed`, retains its error, and clears its displayed path and distance. Parking-point navigation resolves the stored pose and uses the same goal path.

Only an accepted current goal can be canceled. A successfully submitted cancellation returns HTTP 202 with `goal_status: canceling`; a rejected or failed cancellation restores `navigating` and records the error. Nav2 results map to the existing succeeded, canceled, or failed terminal state.

Each goal has a monotonically increasing generation and its Nav2 UUID. Goal, feedback, cancellation, and result callbacks must match the current identity before modifying state. Starting a goal and reaching a terminal state clear the displayed path. Initial-pose publication and active goal reservation remain mutually exclusive. A Nav2 goal submitted outside this node is not guaranteed to appear correctly because `/plan` carries no goal UUID.

The navigation-state `phase` field remains present for JSON compatibility and is always `null`. The browser presents only `发送中`, `导航中` with optional remaining distance, `取消中`, `已到达`, `已取消`, and `导航失败`. The assistant projects the same lifecycle states instead of Behavior Tree phases or a synthetic recovery state, avoiding a client dependency on Nav2 internals.

## Source and dependency boundaries

`web_ui_node.h/.cpp` defines the concrete root ROS capability and owns subordinate resources. `http_server.h/.cpp` owns HTTP routing without exposing httplib types. `web_types.h` contains ROS- and HTTP-independent value types. `map_snapshot.h/.cpp` owns map decoding, grid and path validation, binary encoding, change detection, gzip, and ETag generation. `parking_point_store.h/.cpp` owns parking-point validation and atomic sidecar replacement. Stateless one-use logic remains translation-unit-local free functions rather than new manager or controller classes.

Third-party implementation dependencies are `cpp-httplib`, `nlohmann/json`, `yaml-cpp`, zlib, OpenSSL libcrypto for the existing SHA-256 ETag contract, and `tl::expected` for explicit fallible C++ interfaces. websocketpp, standalone Asio, OpenCV, and a lock-free queue library are not required. Third-party headers and types do not leak through the root node's public interface. The workspace-wide colcon defaults continue to select Release unless the caller explicitly supplies another `CMAKE_BUILD_TYPE`.

## Implementation

`robot_web_ui_core` is an `ament_cmake` C++17 utility library. Its value types are independent of ROS and HTTP; it provides request validation, trinary Nav2 PGM decoding, deterministic gzip encoding, and strong SHA-256 ETags. ROS and HTTP integration remain outside this utility boundary.

`robot_web_ui_http` owns a one-shot `HttpServer` capability with private cpp-httplib state. Creation binds before returning and destruction stops and joins its listener and library-managed workers. Socket options enable address reuse but exclude port sharing so an occupied port fails creation. A manual-operation lease serializes server-wide session replacement and sequence acceptance while releasing the mutex during ROS publication; failed commands preserve the previous sequence. JSON serialization and network output occur after releasing that lease. Binary content providers retain immutable precompressed snapshots through network output and return bodyless 304 only for an exact strong ETag match.

The installed `robot_web_ui` executable creates and binds HTTP before its main-thread `SingleThreadedExecutor` spins. Bind failure logs one fatal diagnostic and exits nonzero. Static assets are restricted to `/` and `/map_view.js`; the request body limit is 4096 bytes and the socket read timeout is 0.5 seconds. cpp-httplib payload-limit errors map to the existing JSON 400 response. The package links the system cpp-httplib library through pkg-config; its build and runtime manifests use the supported `libcpp-httplib-dev` rosdep key, which installs the matching runtime library transitively.

`robot_web_ui_ros` owns the concrete `WebUiNode` and its internal `HttpActions` implementation. The HTTP module borrows that interface and must finish all calls before node destruction. ROS callbacks use the single-threaded executor. GICP messages must contain finite, same-stamp `map -> camera_init` and `map -> body` transforms with nonzero quaternion norms. Localization, the retained local grid, its affine, and transform availability/error are copied together; invalid localization keeps the last pose and affine but marks the transform unavailable. A valid transform updates the affine without rebuilding the grid body, and local grid bodies are accepted only while that transform is available.

Manual commands check and copy the current mode under the state mutex, then publish after releasing it. Initial-pose publication reserves an in-flight flag under the same mutex used by goal reservation; competing initial-pose and goal requests return 409 until publication finishes. An RAII guard clears the flag after either success or failure. Reliable ROS publication can block on transport capacity, so neither publisher call holds the shared-state mutex; state requests and unrelated controls remain available during publication.

Parking operations acquire a non-waiting atomic lease. A concurrent operation returns HTTP 503 without entering the serial store; RAII releases the lease on every return path. File I/O holds no node, snapshot, parking, or navigation mutex. Mode-service callbacks capture shared completion state, and timed-out requests are removed from the ROS client so an unavailable server cannot accumulate pending completions.

## Errors and lifecycle

HTTP errors retain the current 400 validation, 404 missing resource, 409 state conflict, 500 internal response failure, and 503 unavailable dependency semantics. A static-map or parking-sidecar error disables only dependent operations and remains visible in state; it does not terminate unrelated manual control or HTTP service. Invalid ROS input preserves the last valid snapshot and reports the associated layer error.

Startup must bind the HTTP port successfully before entering steady-state execution. Ordinary destruction uses RAII to stop the HTTP server, join its listener thread, and release the ROS node. The implementation has no runtime recovery orchestration, request-draining state machine, watchdog, extensive shutdown test matrix, automatic navigation cancellation, or control-mode transition during exit.

## Alternatives considered

**Retain Python and remove only `/behavior_tree_log`.** This removes one 50 Hz source but leaves NavigateToPose feedback deserialization and executor wake-ups in `rclpy`, so another high-cost Python path can remain or reappear as backend behavior evolves.

**Move navigation and other high-frequency ROS work to C++ while retaining Python HTTP and parking-point logic.** This requires a second private protocol and two processes while preserving Python's fixed runtime and synchronization boundary. Complete migration has a larger initial implementation surface but a clearer long-term ownership boundary and eliminates the class of Python ROS wake-up regressions.

**Use two fixed HTTP workers.** Two workers are sufficient for the expected page and serial voice-assistant traffic, but no evidence shows that replacing cpp-httplib's configuration with that fixed number materially reduces CPU. Retaining library-managed workers also preserves the existing requirement that manual control remain responsive while another request is waiting.

**Use a lock-free queue between HTTP and ROS.** The workload has low command volume, small state, and no measured mutex contention. A lock-free protocol would add ownership, cancellation, and shutdown complexity without addressing DDS deserialization or executor wake-ups.

**Preserve assistant `recovering` aggregation.** This requires Behavior Tree phase input or another synthetic recovery signal. Lifecycle states keep the assistant contract independent of Nav2 internals, so the shipped interface does not expose `recovering`.

## Verification

Focused C++ tests cover snapshot content changes, parking-point persistence, navigation generations and terminal transitions, representative HTTP success and failure paths, ROS interface selection, and package topology. The independent Node harness covers browser assets, including the 100 ms manual scheduler and automatic acknowledgement loop. The migration intentionally does not reproduce every Python edge-case test.

A full `sim + navigation` run through the formal bringup entry remains the dynamic acceptance gap. It must verify localization, maps, path display, manual endpoints, and successful navigation while recording Web UI and total CPU for closed-page, open-idle, and active-navigation states. Active navigation must show a substantial reduction from the sustained approximately 25–30% Python baseline, and cleanup must leave no ROS, Gazebo, Nav2, GICP, or Web UI process behind.

## Consequences

Replacing the Python backend preserves the external HTTP, persistence, and binary-map contracts while removing a process and Python ROS deserialization path. A focused test set reduces migration coverage compared with the mature Python suite, so representative real entry paths remain important.

cpp-httplib workers and the ROS executor access one node concurrently. Short mutex-protected operations and immutable snapshots make that concurrency explicit; blocking a worker while holding shared state would reintroduce latency and deadlock risk.

The UI and assistant give up Nav2 Behavior Tree phase and synthetic recovery detail. Operators retain the goal lifecycle, remaining distance, path, and final outcome. Full tracking of goals created outside this node remains unsupported.
