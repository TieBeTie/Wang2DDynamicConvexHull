#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "Calculator.hpp"
#include "DynamicSlidingConvex2DHull.hpp"
#include "geometry/geometry_utils.hpp"
#include "io_utils.hpp"

static constexpr double EPS = 1e-5;

void compare(const std::vector<Angles>& got, const std::vector<Angles>& exp) {
  ASSERT_EQ(got.size(), exp.size()) << "Size mismatch";
  for (size_t i = 0; i < got.size(); ++i) {
    EXPECT_NEAR(got[i].alpha, exp[i].alpha, EPS)
        << "Alpha mismatch at index " << i;
    // EXPECT_NEAR(got[i].beta, exp[i].beta, EPS)
    //     << "Beta  mismatch at index " << i;
  }
}

class PennantTest : public ::testing::Test {
 protected:
  std::vector<double> ys;

  void SetUp() override { io_utils::readCSV("data/input.csv", ys); }
};

// TEST_F(PennantTest, Window10) {
//   std::vector<Angles> got(ys.size());
//   std::vector<Angles> exp;
//   calculate(ys, got, 10);
//   io_utils::readCSV("data/window_10.csv", exp);
//   compare(got, exp);
// }

// TEST_F(PennantTest, Window100) {
//   std::vector<Angles> got(ys.size());
//   std::vector<Angles> exp;
//   io_utils::readCSV("data/window_100.csv", exp);
//   calculate(ys, got, 100);
//   compare(got, exp);
// }

// TEST_F(PennantTest, Window1000) {
//   std::vector<Angles> got(ys.size());
//   std::vector<Angles> exp;
//   io_utils::readCSV("data/window_1000.csv", exp);
//   calculate(ys, got, 1000);
//   compare(got, exp);
// }

TEST_F(PennantTest, Window10_First22) {
  ys.resize(50);
  std::vector<Angles> got(ys.size());
  std::vector<Angles> exp;
  calculate(ys, got, 10);
  io_utils::readCSV("data/window_10.csv", exp);
  exp.resize(50);
  compare(got, exp);
}
