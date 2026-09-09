#include "robot_web_ui/parking_point_store.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <unistd.h>
#include <utility>
#include <vector>

namespace robot_web_ui
{
/*******************************************************************************************************
 *
 * @brief error code
 *
 *******************************************************************************************************/
namespace
{
class ParkingPointStoreErrorCategory final : public std::error_category {
public:
    const char *name() const noexcept override;
    std::string message(int value) const override;
};

const char *ParkingPointStoreErrorCategory::name() const noexcept
{
    return "parking_point_store";
}

std::string ParkingPointStoreErrorCategory::message(int value) const
{
    switch (static_cast<ParkingPointStore::Errc>(value)) {
    case ParkingPointStore::Errc::invalid_point:
        return "invalid parking point";
    case ParkingPointStore::Errc::duplicate_name:
        return "duplicate parking-point name";
    case ParkingPointStore::Errc::not_found:
        return "parking point not found";
    case ParkingPointStore::Errc::corrupt_sidecar:
        return "corrupt parking-point sidecar";
    }
    return "unknown parking-point error";
}

const std::error_category &parking_point_store_error_category() noexcept
{
    static const ParkingPointStoreErrorCategory category;
    return category;
}

std::error_code current_io_error() noexcept
{
    if (errno != 0)
        return {errno, std::generic_category()};
    return std::make_error_code(std::errc::io_error);
}

tl::unexpected<std::error_code> failure(ParkingPointStore::Errc error) noexcept
{
    return tl::make_unexpected(ParkingPointStore::make_error_code(error));
}
} // namespace

/*******************************************************************************************************
 *
 * @brief sidecar path and parking-point validation
 *
 *******************************************************************************************************/
namespace
{
std::filesystem::path parking_sidecar_path(const std::filesystem::path &map_path)
{
    return map_path.parent_path() / (map_path.stem().string() + ".parking_points.json");
}

bool is_unicode_whitespace(uint32_t code_point)
{
    switch (code_point) {
    case 0x0009:
    case 0x000a:
    case 0x000b:
    case 0x000c:
    case 0x000d:
    case 0x001c:
    case 0x001d:
    case 0x001e:
    case 0x001f:
    case 0x0020:
    case 0x0085:
    case 0x00a0:
    case 0x1680:
    case 0x2000:
    case 0x2001:
    case 0x2002:
    case 0x2003:
    case 0x2004:
    case 0x2005:
    case 0x2006:
    case 0x2007:
    case 0x2008:
    case 0x2009:
    case 0x200a:
    case 0x2028:
    case 0x2029:
    case 0x202f:
    case 0x205f:
    case 0x3000:
        return true;
    default:
        return false;
    }
}

bool decode_utf8(const std::string &value, std::vector<std::pair<size_t, uint32_t>> *code_points)
{
    code_points->clear();
    for (size_t index = 0; index < value.size();) {
        const unsigned char first = static_cast<unsigned char>(value[index]);
        size_t length = 0;
        uint32_t code_point = 0;
        if (first <= 0x7f) {
            length = 1;
            code_point = first;
        } else if (first >= 0xc2 && first <= 0xdf) {
            length = 2;
            code_point = first & 0x1f;
        } else if (first >= 0xe0 && first <= 0xef) {
            length = 3;
            code_point = first & 0x0f;
        } else if (first >= 0xf0 && first <= 0xf4) {
            length = 4;
            code_point = first & 0x07;
        } else {
            return false;
        }
        if (index + length > value.size())
            return false;
        for (size_t offset = 1; offset < length; ++offset) {
            const unsigned char byte = static_cast<unsigned char>(value[index + offset]);
            if ((byte & 0xc0) != 0x80)
                return false;
            code_point = (code_point << 6U) | (byte & 0x3fU);
        }
        if ((length == 3 && code_point < 0x800) || (length == 4 && code_point < 0x10000) ||
            code_point > 0x10ffff || (code_point >= 0xd800 && code_point <= 0xdfff))
            return false;
        code_points->push_back({index, code_point});
        index += length;
    }
    return true;
}

std::optional<std::string> normalize_name(const std::string &value)
{
    std::vector<std::pair<size_t, uint32_t>> code_points;
    if (!decode_utf8(value, &code_points))
        return std::nullopt;
    size_t first = 0;
    while (first < code_points.size() && is_unicode_whitespace(code_points[first].second))
        ++first;
    size_t last = code_points.size();
    while (last > first && is_unicode_whitespace(code_points[last - 1].second))
        --last;
    if (last - first < 1 || last - first > 40)
        return std::nullopt;
    const size_t start_byte = code_points[first].first;
    const size_t end_byte = last == code_points.size() ? value.size() : code_points[last].first;
    return value.substr(start_byte, end_byte - start_byte);
}

std::optional<ParkingPoint> validate_point(ParkingPoint point)
{
    const std::optional<std::string> name = normalize_name(point.name);
    if (!name || !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.yaw))
        return std::nullopt;
    point.name = *name;
    return point;
}
} // namespace

/*******************************************************************************************************
 *
 * @brief atomic sidecar writing
 *
 *******************************************************************************************************/
namespace
{
bool write_all(int descriptor, const std::string &contents, std::error_code *error)
{
    size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t written = write(descriptor, contents.data() + offset, contents.size() - offset);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            *error = {errno, std::generic_category()};
            return false;
        }
        if (written == 0) {
            *error = std::make_error_code(std::errc::io_error);
            return false;
        }
        offset += static_cast<size_t>(written);
    }
    return true;
}
} // namespace

/*******************************************************************************************************
 *
 * @brief parking-point store
 *
 *******************************************************************************************************/
bool operator==(const ParkingPoint &left, const ParkingPoint &right)
{
    return left.name == right.name && left.x == right.x && left.y == right.y && left.yaw == right.yaw;
}

ParkingPointStore::ParkingPointStore(std::filesystem::path sidecar_path, std::vector<ParkingPoint> points)
    : sidecar_path_(std::move(sidecar_path)), points_(std::move(points))
{
}

ParkingPointStore::Result<std::unique_ptr<ParkingPointStore>> ParkingPointStore::create(
    const std::filesystem::path &map_path)
{
    const std::filesystem::path sidecar_path = parking_sidecar_path(map_path);
    std::error_code filesystem_error;
    if (!std::filesystem::exists(sidecar_path, filesystem_error)) {
        if (filesystem_error)
            return tl::make_unexpected(filesystem_error);
        return std::unique_ptr<ParkingPointStore>(new ParkingPointStore(sidecar_path, {}));
    }

    std::string contents;
    errno = 0;
    try {
        std::ifstream input(sidecar_path, std::ios::binary);
        if (!input)
            return tl::make_unexpected(current_io_error());
        input.exceptions(std::ios::badbit);
        contents.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
        if (input.bad())
            return tl::make_unexpected(current_io_error());
    } catch (const std::ios_base::failure &) {
        return tl::make_unexpected(current_io_error());
    }

    try {
        const nlohmann::json root = nlohmann::json::parse(contents);
        if (!root.is_object() || root.size() != 2 || !root.contains("version") || !root.contains("points") ||
            !root.at("version").is_number_integer() || root.at("version") != 1 || !root.at("points").is_array())
            return failure(Errc::corrupt_sidecar);
        std::vector<ParkingPoint> points;
        for (const nlohmann::json &item : root.at("points")) {
            if (!item.is_object() || item.size() != 4 || !item.contains("name") || !item.contains("x") ||
                !item.contains("y") || !item.contains("yaw") || !item.at("name").is_string() ||
                !item.at("x").is_number() || !item.at("y").is_number() || !item.at("yaw").is_number())
                return failure(Errc::corrupt_sidecar);
            ParkingPoint point{
                item.at("name").get<std::string>(),
                item.at("x").get<double>(),
                item.at("y").get<double>(),
                item.at("yaw").get<double>(),
            };
            const std::optional<ParkingPoint> validated = validate_point(point);
            if (!validated || validated->name != point.name)
                return failure(Errc::corrupt_sidecar);
            for (const ParkingPoint &existing : points) {
                if (existing.name == validated->name)
                    return failure(Errc::corrupt_sidecar);
            }
            points.push_back(*validated);
        }
        return std::unique_ptr<ParkingPointStore>(new ParkingPointStore(sidecar_path, std::move(points)));
    } catch (const nlohmann::json::exception &) {
        return failure(Errc::corrupt_sidecar);
    }
}

std::vector<ParkingPoint> ParkingPointStore::list() const
{
    return points_;
}

ParkingPointStore::Result<void> ParkingPointStore::save(ParkingPoint point)
{
    const std::optional<ParkingPoint> validated = validate_point(std::move(point));
    if (!validated)
        return failure(Errc::invalid_point);
    for (const ParkingPoint &existing : points_) {
        if (existing.name == validated->name)
            return failure(Errc::duplicate_name);
    }
    std::vector<ParkingPoint> points = points_;
    points.push_back(*validated);
    if (Result<void> result = write_points(points); !result)
        return result;
    points_ = std::move(points);
    return {};
}

ParkingPointStore::Result<ParkingPoint> ParkingPointStore::get(const std::string &name) const
{
    const std::optional<std::string> normalized_name = normalize_name(name);
    if (!normalized_name)
        return failure(Errc::invalid_point);
    for (const ParkingPoint &point : points_) {
        if (point.name == *normalized_name)
            return point;
    }
    return failure(Errc::not_found);
}

ParkingPointStore::Result<void> ParkingPointStore::erase(const std::string &name)
{
    const std::optional<std::string> normalized_name = normalize_name(name);
    if (!normalized_name)
        return failure(Errc::invalid_point);
    std::vector<ParkingPoint> points;
    points.reserve(points_.size());
    for (const ParkingPoint &point : points_) {
        if (point.name != *normalized_name)
            points.push_back(point);
    }
    if (points.size() == points_.size())
        return failure(Errc::not_found);
    if (Result<void> result = write_points(points); !result)
        return result;
    points_ = std::move(points);
    return {};
}

ParkingPointStore::Result<void> ParkingPointStore::write_points(const std::vector<ParkingPoint> &points) const
{
    nlohmann::json root = {
        {"version", 1},
        {"points", nlohmann::json::array()},
    };
    for (const ParkingPoint &point : points) {
        root["points"].push_back({
            {"name", point.name},
            {"x", point.x},
            {"y", point.y},
            {"yaw", point.yaw},
        });
    }
    const std::string contents = root.dump(2) + '\n';
    std::string temporary_template = (sidecar_path_.parent_path() / ".parking_points_XXXXXX").string();
    std::vector<char> temporary_name(temporary_template.begin(), temporary_template.end());
    temporary_name.push_back('\0');
    const int descriptor = mkstemp(temporary_name.data());
    if (descriptor < 0)
        return tl::make_unexpected(std::error_code(errno, std::generic_category()));
    const std::filesystem::path temporary_path(temporary_name.data());
    std::error_code error;
    const bool wrote = write_all(descriptor, contents, &error);
    if (close(descriptor) != 0 && !error)
        error = {errno, std::generic_category()};
    if (!wrote || error) {
        unlink(temporary_path.c_str());
        return tl::make_unexpected(error);
    }
    if (rename(temporary_path.c_str(), sidecar_path_.c_str()) != 0) {
        error = {errno, std::generic_category()};
        unlink(temporary_path.c_str());
        return tl::make_unexpected(error);
    }
    return {};
}

std::error_code ParkingPointStore::make_error_code(Errc error) noexcept
{
    return {static_cast<int>(error), parking_point_store_error_category()};
}
} // namespace robot_web_ui
