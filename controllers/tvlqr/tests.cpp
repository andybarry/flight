#include "tvlqr-controller.hpp"
#include "gtest/gtest.h"


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "TvlqrControl*";
  return RUN_ALL_TESTS();
}
