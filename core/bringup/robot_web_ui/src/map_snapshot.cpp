#include "robot_web_ui/map_snapshot.h"

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>
#include <zlib.h>

#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>

#include <nlohmann/json.hpp>

namespace robot_web_ui
{
/*******************************************************************************************************
 *
 * @brief binary snapshot encoding
 *
 *******************************************************************************************************/
namespace
{
bool same_grid_info(const GridInfo &left, const GridInfo &right)
{
    return left.width == right.width && left.height == right.height && left.resolution == right.resolution &&
           left.origin_x == right.origin_x && left.origin_y == right.origin_y && left.origin_yaw == right.origin_yaw &&
           left.frame_id == right.frame_id;
}

nlohmann::json grid_metadata(const GridInfo &info)
{
    return {{"width", info.width},
            {"height", info.height},
            {"resolution", info.resolution},
            {"origin", {info.origin_x, info.origin_y, info.origin_yaw}},
            {"frame_id", info.frame_id}};
}

std::vector<uint8_t> gzip_data(const std::vector<uint8_t> &data)
{
    z_stream stream{};
    if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 16 + MAX_WBITS, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        return {};
    gz_header header{};
    header.time = 0;
    if (deflateSetHeader(&stream, &header) != Z_OK) {
        deflateEnd(&stream);
        return {};
    }

    stream.next_in = const_cast<Bytef *>(data.data());
    stream.avail_in = static_cast<uInt>(data.size());
    std::vector<uint8_t> result;
    std::array<uint8_t, 4096> output{};
    int status = Z_OK;
    while (status == Z_OK) {
        stream.next_out = output.data();
        stream.avail_out = static_cast<uInt>(output.size());
        status = deflate(&stream, Z_FINISH);
        result.insert(result.end(), output.begin(), output.begin() + output.size() - stream.avail_out);
    }
    deflateEnd(&stream);
    if (status != Z_STREAM_END)
        return {};
    return result;
}

BinarySnapshotPtr make_binary(uint64_t revision, const nlohmann::json &metadata, const std::vector<uint8_t> &data)
{
    const std::string metadata_bytes = metadata.dump();
    std::vector<uint8_t> hash_input;
    hash_input.reserve(sizeof(binary_media_type) - 1 + metadata_bytes.size() + data.size());
    hash_input.insert(hash_input.end(), binary_media_type, binary_media_type + sizeof(binary_media_type) - 1);
    hash_input.insert(hash_input.end(), metadata_bytes.begin(), metadata_bytes.end());
    hash_input.insert(hash_input.end(), data.begin(), data.end());

    std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
    SHA256(hash_input.data(), hash_input.size(), digest.data());
    std::ostringstream etag;
    etag << '\"' << std::hex << std::setfill('0');
    for (const unsigned char byte : digest)
        etag << std::setw(2) << static_cast<unsigned int>(byte);
    etag << '\"';

    return std::make_shared<const BinarySnapshot>(BinarySnapshot{revision, etag.str(), data, gzip_data(data)});
}
} // namespace

/*******************************************************************************************************
 *
 * @brief Nav2 PGM decoding
 *
 *******************************************************************************************************/
namespace
{
bool read_pgm_token(const std::vector<uint8_t> &raw, size_t *index, std::string *token)
{
    while (*index < raw.size()) {
        const uint8_t byte = raw[*index];
        if (std::isspace(byte) != 0) {
            ++*index;
            continue;
        }
        if (byte == '#') {
            while (*index < raw.size() && raw[*index] != '\n')
                ++*index;
            continue;
        }
        break;
    }
    const size_t start = *index;
    while (*index < raw.size() && std::isspace(raw[*index]) == 0 && raw[*index] != '#')
        ++*index;
    if (start == *index)
        return false;
    token->assign(reinterpret_cast<const char *>(raw.data() + start), *index - start);
    return true;
}

bool parse_pgm_uint32(const std::string &token, uint32_t *value)
{
    if (token.empty())
        return false;
    uint32_t parsed = 0;
    for (const unsigned char character : token) {
        if (character < '0' || character > '9')
            return false;
        const uint32_t digit = character - '0';
        if (parsed > (std::numeric_limits<uint32_t>::max() - digit) / 10)
            return false;
        parsed = parsed * 10 + digit;
    }
    *value = parsed;
    return true;
}

tl::expected<std::tuple<uint32_t, uint32_t, std::vector<uint8_t>>, std::string> read_pgm(
    const std::filesystem::path &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
        return tl::make_unexpected("invalid image: cannot read " + path.string());
    std::vector<uint8_t> raw;
    try {
        raw.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    } catch (const std::ios_base::failure &) {
        return tl::make_unexpected("invalid image: cannot read " + path.string());
    }

    size_t index = 0;
    std::string magic;
    std::string width_token;
    std::string height_token;
    std::string max_value_token;
    if (!read_pgm_token(raw, &index, &magic) || !read_pgm_token(raw, &index, &width_token) ||
        !read_pgm_token(raw, &index, &height_token) || !read_pgm_token(raw, &index, &max_value_token)) {
        return tl::make_unexpected("invalid PGM header: missing token");
    }
    if (magic != "P5")
        return tl::make_unexpected("invalid PGM magic: expected P5");

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t max_value = 0;
    if (!parse_pgm_uint32(width_token, &width) || !parse_pgm_uint32(height_token, &height) ||
        !parse_pgm_uint32(max_value_token, &max_value)) {
        return tl::make_unexpected("invalid PGM header dimensions or max value");
    }
    if (width == 0 || height == 0 || max_value != 255)
        return tl::make_unexpected("invalid PGM dimensions or max value");
    if (index >= raw.size() || std::isspace(raw[index]) == 0)
        return tl::make_unexpected("invalid PGM header: missing raster separator");
    if (raw[index] == '\r' && index + 1 < raw.size() && raw[index + 1] == '\n')
        index += 2;
    else
        ++index;

    if (static_cast<size_t>(height) > std::numeric_limits<size_t>::max() / width)
        return tl::make_unexpected("invalid PGM dimensions");
    const size_t expected = static_cast<size_t>(width) * height;
    if (raw.size() - index != expected)
        return tl::make_unexpected("invalid PGM pixel count");
    return std::make_tuple(width, height, std::vector<uint8_t>(raw.begin() + index, raw.end()));
}

tl::expected<double, std::string> yaml_number(const YAML::Node &root, const char *field)
{
    const YAML::Node value = root[field];
    if (!value || !value.IsScalar())
        return tl::make_unexpected(std::string("invalid ") + field + ": expected a number");
    try {
        const double number = value.as<double>();
        if (!std::isfinite(number))
            return tl::make_unexpected(std::string("invalid ") + field + ": expected a finite number");
        return number;
    } catch (const YAML::Exception &) {
        return tl::make_unexpected(std::string("invalid ") + field + ": expected a number");
    }
}
} // namespace

/*******************************************************************************************************
 *
 * @brief static map loading
 *
 *******************************************************************************************************/
tl::expected<GridSnapshotPtr, std::string> load_nav2_pgm(const std::filesystem::path &yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path.string());
    } catch (const YAML::Exception &error) {
        return tl::make_unexpected(std::string("invalid YAML: ") + error.what());
    }
    if (!root.IsMap())
        return tl::make_unexpected("invalid YAML root: expected a mapping");

    const YAML::Node image = root["image"];
    if (!image || !image.IsScalar() || image.as<std::string>().empty())
        return tl::make_unexpected("invalid image: expected a non-empty path");
    const auto resolution = yaml_number(root, "resolution");
    const auto occupied_threshold = yaml_number(root, "occupied_thresh");
    const auto free_threshold = yaml_number(root, "free_thresh");
    if (!resolution || !occupied_threshold || !free_threshold)
        return tl::make_unexpected(!resolution ? resolution.error() :
                                   !occupied_threshold ? occupied_threshold.error() : free_threshold.error());
    if (*resolution <= 0.0 || *free_threshold < 0.0 || *free_threshold > 1.0 || *occupied_threshold < 0.0 ||
        *occupied_threshold > 1.0 || *free_threshold >= *occupied_threshold) {
        return tl::make_unexpected("invalid map thresholds or resolution");
    }

    const YAML::Node origin = root["origin"];
    if (!origin || !origin.IsSequence() || origin.size() != 3)
        return tl::make_unexpected("invalid origin: expected three numbers");
    std::array<double, 3> origin_values{};
    for (size_t index = 0; index < origin_values.size(); ++index) {
        try {
            origin_values[index] = origin[index].as<double>();
        } catch (const YAML::Exception &) {
            return tl::make_unexpected("invalid origin: expected three numbers");
        }
        if (!std::isfinite(origin_values[index]))
            return tl::make_unexpected("invalid origin: expected finite numbers");
    }

    const YAML::Node negate = root["negate"];
    if (!negate || !negate.IsScalar())
        return tl::make_unexpected("invalid negate: expected 0 or 1");
    int negate_value = 0;
    try {
        negate_value = negate.as<int>();
    } catch (const YAML::Exception &) {
        return tl::make_unexpected("invalid negate: expected 0 or 1");
    }
    if (negate_value != 0 && negate_value != 1)
        return tl::make_unexpected("invalid negate: expected 0 or 1");
    if (!root["mode"] || !root["mode"].IsScalar())
        return tl::make_unexpected("invalid mode: only trinary is supported");
    try {
        if (root["mode"].as<std::string>() != "trinary")
            return tl::make_unexpected("invalid mode: only trinary is supported");
    } catch (const YAML::Exception &) {
        return tl::make_unexpected("invalid mode: only trinary is supported");
    }

    std::filesystem::path image_path = image.as<std::string>();
    if (image_path.is_relative())
        image_path = yaml_path.parent_path() / image_path;
    const auto pgm = read_pgm(image_path);
    if (!pgm)
        return tl::make_unexpected(pgm.error());

    const uint32_t width = std::get<0>(*pgm);
    const uint32_t height = std::get<1>(*pgm);
    const std::vector<uint8_t> &pixels = std::get<2>(*pgm);
    std::vector<uint8_t> data;
    data.reserve(pixels.size());
    const bool inverted = negate_value == 1;
    for (uint32_t row = height; row-- > 0;) {
        for (uint32_t column = 0; column < width; ++column) {
            const uint8_t pixel = pixels[static_cast<size_t>(row) * width + column];
            const double occupancy = inverted ? pixel / 255.0 : (255 - pixel) / 255.0;
            data.push_back(occupancy > *occupied_threshold ? 100 : occupancy < *free_threshold ? 0 : 255);
        }
    }
    return update_grid_snapshot(nullptr, GridInfo{width, height, *resolution, origin_values[0], origin_values[1],
                                                  origin_values[2], "map"},
                                data);
}

/*******************************************************************************************************
 *
 * @brief occupancy-grid snapshots
 *
 *******************************************************************************************************/
GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<uint8_t> &data)
{
    if (info.width == 0 || info.height == 0 || data.size() != static_cast<size_t>(info.width) * info.height)
        return nullptr;
    if (current && same_grid_info(current->info, info) && current->binary->data == data)
        return current;
    const uint64_t revision = current ? current->binary->revision + 1 : 1;
    return std::make_shared<const GridSnapshot>(GridSnapshot{info, make_binary(revision, grid_metadata(info), data)});
}

GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<int8_t> &data)
{
    std::vector<uint8_t> converted;
    converted.reserve(data.size());
    for (const int8_t value : data)
        converted.push_back(static_cast<uint8_t>(value));
    return update_grid_snapshot(std::move(current), info, converted);
}

/*******************************************************************************************************
 *
 * @brief path snapshots
 *
 *******************************************************************************************************/
PathSnapshotPtr update_path_snapshot(
    PathSnapshotPtr current, const std::string &frame_id, const std::vector<std::pair<double, double>> &points)
{
    std::vector<uint8_t> data;
    data.reserve(points.size() * sizeof(float) * 2);
    for (const auto &[x, y] : points) {
        const float coordinates[] = {static_cast<float>(x), static_cast<float>(y)};
        for (const float coordinate : coordinates) {
            uint32_t bits = 0;
            static_assert(sizeof(bits) == sizeof(coordinate));
            std::memcpy(&bits, &coordinate, sizeof(bits));
            for (size_t shift = 0; shift < sizeof(bits); ++shift)
                data.push_back(static_cast<uint8_t>(bits >> (shift * 8)));
        }
    }
    if (current && current->frame_id == frame_id && current->binary->data == data)
        return current;
    const uint64_t revision = current ? current->binary->revision + 1 : 1;
    return std::make_shared<const PathSnapshot>(
        PathSnapshot{frame_id, make_binary(revision, {{"frame_id", frame_id}}, data)});
}
} // namespace robot_web_ui
