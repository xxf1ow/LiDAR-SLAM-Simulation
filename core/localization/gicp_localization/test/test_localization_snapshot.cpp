#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>

#include "gicp_localization/localization_snapshot.hpp"

namespace gicp_localization {
namespace {

TEST(LocalizationSnapshot, CarriesAtomicMapTransforms)
{
  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  map_to_odom.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  odom_to_base.translation() = Eigen::Vector3d(4.0, -1.0, 2.0);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;

  const auto snapshot = make_localization_snapshot(
      map_to_odom, odom_to_base, stamp, "map", "camera_init", "body");

  ASSERT_EQ(snapshot.transforms.size(), 2U);
  EXPECT_EQ(snapshot.transforms[0].header.stamp, stamp);
  EXPECT_EQ(snapshot.transforms[0].header.frame_id, "map");
  EXPECT_EQ(snapshot.transforms[0].child_frame_id, "camera_init");
  EXPECT_DOUBLE_EQ(snapshot.transforms[0].transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(snapshot.transforms[0].transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(snapshot.transforms[0].transform.translation.z, 3.0);

  EXPECT_EQ(snapshot.transforms[1].header.stamp, stamp);
  EXPECT_EQ(snapshot.transforms[1].header.frame_id, "map");
  EXPECT_EQ(snapshot.transforms[1].child_frame_id, "body");
  EXPECT_DOUBLE_EQ(snapshot.transforms[1].transform.translation.x, 5.0);
  EXPECT_DOUBLE_EQ(snapshot.transforms[1].transform.translation.y, 1.0);
  EXPECT_DOUBLE_EQ(snapshot.transforms[1].transform.translation.z, 5.0);
}

}  // namespace
}  // namespace gicp_localization
