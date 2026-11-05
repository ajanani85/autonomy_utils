#include <gtest/gtest.h>
#include <autonomy_utils/lib/geometry.h>

TEST(add, ExampleTest) {
    EXPECT_EQ(2+3, 5);  // Replace with your actual test case
}

// Reference values from: echo "<lat> <lon> 0 2026.84" | cs2cs -d 9 EPSG:7912 EPSG:6319
TEST(geometry, Wgs84ToNad83) {
    double lat, lon;
    // Tormes, TX
    ASSERT_TRUE(ros2::wgs84_to_nad83(32.052278422, -96.663425000, lat, lon, 2026.84));
    EXPECT_NEAR(lat, 32.052273692, 1e-8);
    EXPECT_NEAR(lon, -96.663413022, 1e-8);
    // Davis, CA
    ASSERT_TRUE(ros2::wgs84_to_nad83(38.528546, -121.696258, lat, lon, 2026.84));
    EXPECT_NEAR(lat, 38.528544388, 1e-8);
    EXPECT_NEAR(lon, -121.696240049, 1e-8);
    // Default epoch (now) still produces a real (non-zero) shift
    ASSERT_TRUE(ros2::wgs84_to_nad83(32.052278422, -96.663425000, lat, lon));
    EXPECT_GT(std::abs(lon - -96.663425000), 5e-6);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}