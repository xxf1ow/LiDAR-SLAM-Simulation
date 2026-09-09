#ifndef XX_PARKING_POINT_STORE_H_
#define XX_PARKING_POINT_STORE_H_

#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <tl/expected.hpp>

namespace robot_web_ui
{
/** A named map-frame pose with x/y in meters and yaw in radians. */
struct ParkingPoint {
    std::string name;
    double x;
    double y;
    double yaw;
};

bool operator==(const ParkingPoint &left, const ParkingPoint &right);

/**
 * Serial persistent storage for ordered parking points. Callers synchronize
 * access. Names are valid UTF-8, trimmed of leading and trailing Unicode
 * whitespace, and contain 1 through 40 Unicode code points after trimming;
 * matching and duplicate detection use the resulting exact string.
 */
class ParkingPointStore {
public:
    template <typename T>
    using Result = tl::expected<T, std::error_code>;

    enum class Errc {
        invalid_point = 1,
        duplicate_name,
        not_found,
        corrupt_sidecar,
    };

    ParkingPointStore(const ParkingPointStore &) = delete;
    ParkingPointStore &operator=(const ParkingPointStore &) = delete;
    ParkingPointStore(ParkingPointStore &&) = delete;
    ParkingPointStore &operator=(ParkingPointStore &&) = delete;

    /**
     * Reads `<map-stem>.parking_points.json` beside `map_path` once at startup.
     * A missing sidecar creates an empty store. I/O returns its dependency error;
     * invalid content returns `corrupt_sidecar` without rewriting the file.
     * Later external changes are not reloaded.
     */
    [[nodiscard]] static Result<std::unique_ptr<ParkingPointStore>> create(const std::filesystem::path &map_path);

    /** Returns the current points in their persisted order. */
    [[nodiscard]] std::vector<ParkingPoint> list() const;

    /**
     * Appends a finite pose after name normalization and atomically replaces the
     * sidecar. A failed write preserves both memory and the target file. Success
     * does not provide an fsync durability guarantee. Invalid names or values,
     * duplicate normalized names, and I/O return their corresponding errors.
     */
    [[nodiscard]] Result<void> save(ParkingPoint point);

    /** Returns a normalized-name match, or a validation or not-found error. */
    [[nodiscard]] Result<ParkingPoint> get(const std::string &name) const;

    /**
     * Removes a normalized-name match and atomically replaces the sidecar. A
     * failed write preserves both memory and the target file. Success does not
     * provide an fsync durability guarantee. Invalid names, absent matches, and
     * I/O return their corresponding errors.
     */
    [[nodiscard]] Result<void> erase(const std::string &name);

    /** Returns this type's explicit error code for a domain error. */
    [[nodiscard]] static std::error_code make_error_code(Errc error) noexcept;

private:
    ParkingPointStore(std::filesystem::path sidecar_path, std::vector<ParkingPoint> points);

    [[nodiscard]] Result<void> write_points(const std::vector<ParkingPoint> &points) const;

    std::filesystem::path sidecar_path_;
    std::vector<ParkingPoint> points_;
};
} // namespace robot_web_ui

#endif  // XX_PARKING_POINT_STORE_H_
