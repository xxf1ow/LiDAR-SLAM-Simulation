#ifndef ROBOT_WEB_UI_WEB_TYPES_H_
#define ROBOT_WEB_UI_WEB_TYPES_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_web_ui
{
/** Grid geometry with distances in meters and orientation in radians. */
struct GridInfo {
    /** Number of cells along the local x axis. */
    uint32_t width;
    /** Number of cells along the local y axis. */
    uint32_t height;
    /** Cell edge length in meters. */
    double resolution;
    /** World x coordinate of the grid origin in meters. */
    double origin_x;
    /** World y coordinate of the grid origin in meters. */
    double origin_y;
    /** World yaw of the grid origin in radians. */
    double origin_yaw;
    /** Frame containing the grid geometry. */
    std::string frame_id;
};

/** A validated navigation target in the static-map frame. */
struct NavigationPose {
    double x;
    double y;
    double yaw;
};

/** Binary map media type included in strong ETags and HTTP responses. */
inline constexpr char binary_media_type[] = "application/octet-stream";

/** A revisioned binary HTTP representation. */
struct BinarySnapshot {
    uint64_t revision;
    std::string etag;
    std::vector<uint8_t> data;
    std::vector<uint8_t> gzip_data;
};

/** Immutable grid geometry and binary body. */
struct GridSnapshot {
    GridInfo info;
    std::shared_ptr<const BinarySnapshot> binary;
};

/** Immutable path body encoded as float32 x/y pairs. */
struct PathSnapshot {
    std::string frame_id;
    std::shared_ptr<const BinarySnapshot> binary;
};

/** Shared immutable snapshot ownership for asynchronous consumers. */
using BinarySnapshotPtr = std::shared_ptr<const BinarySnapshot>;
using GridSnapshotPtr = std::shared_ptr<const GridSnapshot>;
using PathSnapshotPtr = std::shared_ptr<const PathSnapshot>;
} // namespace robot_web_ui

#endif  // ROBOT_WEB_UI_WEB_TYPES_H_
