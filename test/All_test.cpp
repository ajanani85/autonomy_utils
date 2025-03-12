#include <gtest/gtest.h>

TEST(add, ExampleTest) {
    EXPECT_EQ(2+3, 5);  // Replace with your actual test case
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}