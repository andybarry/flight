#include "StereoFilter.hpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "StereoFilterTest*";
  return RUN_ALL_TESTS();
}
