#include "robot_web_ui/map_snapshot.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <zlib.h>

namespace robot_web_ui
{
namespace
{
std::filesystem::path write_map_fixture(const std::vector<uint8_t> &pgm_data, const std::string &yaml)
{
    const auto directory = std::filesystem::temp_directory_path() /
                           ("robot_web_ui_snapshot_" +
                            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(directory);
    {
        std::ofstream pgm(directory / "map.pgm", std::ios::binary);
        pgm.write("P5\n2 2\n255\n", 11);
        pgm.write(reinterpret_cast<const char *>(pgm_data.data()), static_cast<std::streamsize>(pgm_data.size()));
    }
    {
        std::ofstream yaml_file(directory / "map.yaml");
        yaml_file << yaml;
    }
    return directory / "map.yaml";
}

std::vector<uint8_t> inflate_gzip(const std::vector<uint8_t> &gzip_data)
{
    z_stream stream{};
    EXPECT_EQ(inflateInit2(&stream, 16 + MAX_WBITS), Z_OK);
    std::vector<uint8_t> result;
    uint8_t output[32];
    stream.next_in = const_cast<Bytef *>(gzip_data.data());
    stream.avail_in = static_cast<uInt>(gzip_data.size());
    int status = Z_OK;
    while (status == Z_OK) {
        stream.next_out = output;
        stream.avail_out = sizeof(output);
        status = inflate(&stream, Z_NO_FLUSH);
        result.insert(result.end(), output, output + sizeof(output) - stream.avail_out);
    }
    EXPECT_EQ(status, Z_STREAM_END);
    EXPECT_EQ(inflateEnd(&stream), Z_OK);
    return result;
}
} // namespace

TEST(LoadNav2Pgm, ConvertsTrinaryPixelsAndFlipsRows)
{
    const auto yaml_path = write_map_fixture(
        {0x00, 0xfe, 0xcd, 0xfe},
        "image: map.pgm\nresolution: 0.2\norigin: [1.5, -2.0, 0.25]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n");

    const auto snapshot = load_nav2_pgm(yaml_path);

    ASSERT_TRUE(snapshot);
    EXPECT_EQ((*snapshot)->info.width, 2U);
    EXPECT_EQ((*snapshot)->info.height, 2U);
    EXPECT_EQ((*snapshot)->info.frame_id, "map");
    EXPECT_EQ((*snapshot)->binary->data, std::vector<uint8_t>({0, 0, 100, 0}));
    std::filesystem::remove_all(yaml_path.parent_path());
}

TEST(LoadNav2Pgm, RejectsInvalidPgm)
{
    const auto yaml_path = write_map_fixture(
        {0x00, 0xfe, 0xcd, 0xfe},
        "image: map.pgm\nresolution: 0.2\norigin: [1.5, -2.0, 0.25]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n");
    {
        std::ofstream pgm(yaml_path.parent_path() / "map.pgm", std::ios::binary | std::ios::trunc);
        pgm << "P2\n2 2\n255\n";
    }

    EXPECT_FALSE(load_nav2_pgm(yaml_path));
    std::filesystem::remove_all(yaml_path.parent_path());
}

TEST(LoadNav2Pgm, DirectoryImageReturnsReadError)
{
    const auto yaml_path = write_map_fixture(
        {},
        "image: .\nresolution: 0.2\norigin: [1.5, -2.0, 0.25]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n");

    const auto snapshot = load_nav2_pgm(yaml_path);

    ASSERT_FALSE(snapshot);
    EXPECT_EQ(snapshot.error(), "invalid image: cannot read " + (yaml_path.parent_path() / ".").string());
    std::filesystem::remove_all(yaml_path.parent_path());
}

TEST(LoadNav2Pgm, RejectsNonDecimalAndUnrepresentableDimensions)
{
    const std::vector<uint8_t> pixels{0x00, 0xfe, 0xcd, 0xfe};
    const std::string yaml =
        "image: map.pgm\nresolution: 0.2\norigin: [1.5, -2.0, 0.25]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n";
    const std::string dimensions[] = {"2junk 2", "4294967298 2"};

    for (const std::string &dimension : dimensions) {
        const auto yaml_path = write_map_fixture(pixels, yaml);
        {
            std::ofstream pgm(yaml_path.parent_path() / "map.pgm", std::ios::binary | std::ios::trunc);
            pgm << "P5\n" << dimension << "\n255\n";
            pgm.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
        }

        EXPECT_FALSE(load_nav2_pgm(yaml_path)) << dimension;
        std::filesystem::remove_all(yaml_path.parent_path());
    }
}

TEST(GridSnapshot, ReusesEqualContentAndHashesChangedContent)
{
    const GridInfo info{2, 2, 0.2, 1.5, -2.0, 0.25, "map"};
    const auto grid = update_grid_snapshot(nullptr, info, std::vector<uint8_t>({0, 100, 255, 0}));
    const auto equal = update_grid_snapshot(grid, info, std::vector<uint8_t>({0, 100, 255, 0}));
    const auto independently_built = update_grid_snapshot(nullptr, info, std::vector<uint8_t>({0, 100, 255, 0}));
    const auto changed = update_grid_snapshot(grid, info, std::vector<uint8_t>({0, 100, 255, 100}));

    ASSERT_TRUE(grid);
    EXPECT_EQ(grid->binary->revision, 1U);
    EXPECT_EQ(grid->binary->etag,
              "\"fae42947f8e378c986ec6af7eaa4b76ca50c75ed5945815d81deba90615d07a3\"");
    EXPECT_EQ(equal.get(), grid.get());
    EXPECT_EQ(grid->binary->gzip_data, independently_built->binary->gzip_data);
    EXPECT_EQ(inflate_gzip(grid->binary->gzip_data), grid->binary->data);
    EXPECT_EQ(changed->binary->revision, 2U);
}

TEST(GridSnapshot, ConvertsSignedCostsBeforeStorage)
{
    const GridInfo info{2, 1, 0.2, 0.0, 0.0, 0.0, "map"};

    const auto grid = update_grid_snapshot(nullptr, info, std::vector<int8_t>({-1, 37}));

    ASSERT_TRUE(grid);
    EXPECT_EQ(grid->binary->data, std::vector<uint8_t>({255, 37}));
}

TEST(PathSnapshot, EncodesLittleEndianPointsAndHashesMetadata)
{
    const auto path = update_path_snapshot(nullptr, "map", {{1.25, -2.5}, {3.5, 4.25}});
    const auto equal = update_path_snapshot(path, "map", {{1.25, -2.5}, {3.5, 4.25}});
    const auto changed = update_path_snapshot(path, "map", {{1.25, -2.5}, {3.5, 4.5}});

    ASSERT_TRUE(path);
    EXPECT_EQ(path->binary->data,
              std::vector<uint8_t>({0x00, 0x00, 0xa0, 0x3f, 0x00, 0x00, 0x20, 0xc0,
                                    0x00, 0x00, 0x60, 0x40, 0x00, 0x00, 0x88, 0x40}));
    EXPECT_EQ(path->binary->etag,
              "\"7ce252039ad7e15a6ec50497438e41d367930442da763dce85cebcb9f76fc5e2\"");
    EXPECT_EQ(equal.get(), path.get());
    EXPECT_EQ(changed->binary->revision, 2U);
}
} // namespace robot_web_ui
