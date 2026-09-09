#include "robot_web_ui/parking_point_store.h"

#include <gtest/gtest.h>

#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <type_traits>
#include <unistd.h>

namespace robot_web_ui
{
namespace
{
static_assert(!std::is_copy_constructible_v<ParkingPointStore>);
static_assert(!std::is_copy_assignable_v<ParkingPointStore>);
static_assert(!std::is_move_constructible_v<ParkingPointStore>);
static_assert(!std::is_move_assignable_v<ParkingPointStore>);

class TemporaryDirectory {
public:
    TemporaryDirectory();
    ~TemporaryDirectory();

    [[nodiscard]] std::filesystem::path path() const;

private:
    std::filesystem::path path_;
};

TemporaryDirectory::TemporaryDirectory()
{
    char template_path[] = "/tmp/robot_web_ui_parking_XXXXXX";
    char *created_path = mkdtemp(template_path);
    if (created_path == nullptr)
        throw std::runtime_error("mkdtemp failed");
    path_ = created_path;
}

TemporaryDirectory::~TemporaryDirectory()
{
    std::error_code error;
    std::filesystem::remove_all(path_, error);
}

std::filesystem::path TemporaryDirectory::path() const
{
    return path_;
}

std::string read_file(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void write_file(const std::filesystem::path &path, const std::string &contents)
{
    std::ofstream output(path, std::ios::binary);
    output << contents;
}

class FileSizeLimit {
public:
    FileSizeLimit();
    ~FileSizeLimit();

private:
    rlimit original_ {};
    using SignalHandler = void (*)(int);
    SignalHandler original_signal_ = SIG_DFL;
};

FileSizeLimit::FileSizeLimit()
{
    if (getrlimit(RLIMIT_FSIZE, &original_) != 0)
        throw std::runtime_error("getrlimit failed");
    original_signal_ = signal(SIGXFSZ, SIG_IGN);
    rlimit limited = original_;
    limited.rlim_cur = 0;
    if (setrlimit(RLIMIT_FSIZE, &limited) != 0)
        throw std::runtime_error("setrlimit failed");
}

FileSizeLimit::~FileSizeLimit()
{
    setrlimit(RLIMIT_FSIZE, &original_);
    signal(SIGXFSZ, original_signal_);
}

TEST(ParkingPointStore, MissingSidecarLoadsEmptyAndSaveWritesVersionOneJson)
{
    TemporaryDirectory directory;
    const std::filesystem::path map_path = directory.path() / "warehouse.yaml";

    auto store = ParkingPointStore::create(map_path);
    ASSERT_TRUE(store);
    EXPECT_TRUE((*store)->list().empty());

    EXPECT_TRUE((*store)->save(ParkingPoint{"充电区", 1.0, 2.0, 0.25}));
    const std::vector<ParkingPoint> points = (*store)->list();
    ASSERT_EQ(points.size(), 1U);
    EXPECT_EQ(points.at(0).name, "充电区");
    EXPECT_DOUBLE_EQ(points.at(0).x, 1.0);

    const std::string sidecar = read_file(directory.path() / "warehouse.parking_points.json");
    EXPECT_NE(sidecar.find("\"version\": 1"), std::string::npos);
    EXPECT_NE(sidecar.find("充电区"), std::string::npos);
}

TEST(ParkingPointStore, NormalizesUnicodeWhitespaceAndRejectsDuplicateNames)
{
    TemporaryDirectory directory;
    auto store = ParkingPointStore::create(directory.path() / "warehouse.yaml");
    ASSERT_TRUE(store);

    EXPECT_TRUE((*store)->save(ParkingPoint{"\u3000充电区\u00a0", 1.0, 2.0, 0.25}));
    EXPECT_FALSE((*store)->save(ParkingPoint{" 充电区 ", 9.0, 9.0, 0.0}));
    EXPECT_EQ((*store)->list().at(0).name, "充电区");

    std::string forty_code_points;
    for (size_t index = 0; index < 40; ++index)
        forty_code_points += "充";
    EXPECT_TRUE((*store)->save(ParkingPoint{"\u3000" + forty_code_points + "\u3000", 3.0, 3.0, 0.0}));
    EXPECT_FALSE((*store)->save(ParkingPoint{forty_code_points + "充", 4.0, 4.0, 0.0}));
}

TEST(ParkingPointStore, DeletesPointWithoutChangingTheOrderOfRemainingPoints)
{
    TemporaryDirectory directory;
    auto store = ParkingPointStore::create(directory.path() / "warehouse.yaml");
    ASSERT_TRUE(store);
    ASSERT_TRUE((*store)->save(ParkingPoint{"one", 1.0, 1.0, 0.0}));
    ASSERT_TRUE((*store)->save(ParkingPoint{"two", 2.0, 2.0, 0.0}));
    ASSERT_TRUE((*store)->save(ParkingPoint{"three", 3.0, 3.0, 0.0}));

    EXPECT_TRUE((*store)->erase("two"));
    const std::vector<ParkingPoint> points = (*store)->list();
    ASSERT_EQ(points.size(), 2U);
    EXPECT_EQ(points.at(0).name, "one");
    EXPECT_EQ(points.at(1).name, "three");
}

TEST(ParkingPointStore, CorruptSidecarFailsCreationWithoutOverwritingIt)
{
    TemporaryDirectory directory;
    const std::filesystem::path sidecar = directory.path() / "warehouse.parking_points.json";
    write_file(sidecar, "not json");

    const auto store = ParkingPointStore::create(directory.path() / "warehouse.yaml");
    EXPECT_FALSE(store);
    EXPECT_EQ(read_file(sidecar), "not json");
}

TEST(ParkingPointStore, DirectorySidecarReturnsAnIoErrorWithoutThrowing)
{
    TemporaryDirectory directory;
    const std::filesystem::path sidecar = directory.path() / "warehouse.parking_points.json";
    ASSERT_TRUE(std::filesystem::create_directory(sidecar));

    EXPECT_NO_THROW({
        const auto store = ParkingPointStore::create(directory.path() / "warehouse.yaml");
        ASSERT_FALSE(store);
        EXPECT_EQ(store.error(), std::make_error_code(std::errc::is_a_directory));
    });
}

TEST(ParkingPointStore, InvalidUnicodeScalarIsRejectedWithoutWriting)
{
    TemporaryDirectory directory;
    auto store = ParkingPointStore::create(directory.path() / "warehouse.yaml");
    ASSERT_TRUE(store);
    const std::string invalid_scalar("\xf4\x90\x80\x80", 4);

    EXPECT_NO_THROW({
        EXPECT_FALSE((*store)->save(ParkingPoint{invalid_scalar, 1.0, 2.0, 0.0}));
    });
    EXPECT_TRUE((*store)->list().empty());
    EXPECT_FALSE(std::filesystem::exists(directory.path() / "warehouse.parking_points.json"));
}

TEST(ParkingPointStore, WriteFailurePreservesMemoryAndTargetBytes)
{
    TemporaryDirectory directory;
    const std::filesystem::path map_path = directory.path() / "warehouse.yaml";
    auto store = ParkingPointStore::create(map_path);
    ASSERT_TRUE(store);
    ASSERT_TRUE((*store)->save(ParkingPoint{"one", 1.0, 1.0, 0.0}));
    const std::filesystem::path sidecar = directory.path() / "warehouse.parking_points.json";
    const std::string before_bytes = read_file(sidecar);
    const std::vector<ParkingPoint> before_points = (*store)->list();

    {
        FileSizeLimit limit;
        EXPECT_FALSE((*store)->save(ParkingPoint{"two", 2.0, 2.0, 0.0}));
    }

    EXPECT_EQ((*store)->list(), before_points);
    EXPECT_EQ(read_file(sidecar), before_bytes);
}
} // namespace
} // namespace robot_web_ui
