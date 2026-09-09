#ifndef ROBOT_WEB_UI_PARKING_POINT_STORE_H_
#define ROBOT_WEB_UI_PARKING_POINT_STORE_H_

#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <tl/expected.hpp>

namespace robot_web_ui
{
/** A named pose persisted in a map's parking-point sidecar. */
struct ParkingPoint {
    std::string name;
    double x;
    double y;
    double yaw;
};

bool operator==(const ParkingPoint &left, const ParkingPoint &right);

/** Serial persistent storage for ordered parking points. Callers synchronize access. */
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

    /** Opens the map's sidecar, returning an error without changing corrupt input. */
    [[nodiscard]] static Result<std::unique_ptr<ParkingPointStore>> create(const std::filesystem::path &map_path);

    /** Returns the current points in their persisted order. */
    [[nodiscard]] std::vector<ParkingPoint> list() const;

    /** Atomically appends a validated point, or returns a validation, duplicate, or I/O error. */
    [[nodiscard]] Result<void> save(ParkingPoint point);

    /** Returns a normalized-name match, or a validation or not-found error. */
    [[nodiscard]] Result<ParkingPoint> get(const std::string &name) const;

    /** Atomically removes a normalized-name match, or returns a validation, not-found, or I/O error. */
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

#endif  // ROBOT_WEB_UI_PARKING_POINT_STORE_H_
