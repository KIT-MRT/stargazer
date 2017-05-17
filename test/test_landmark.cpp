// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion
// Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);
// expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2); val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2); val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2); val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2); val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2); val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the
// two float values are almost equal (4 ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the
// two double values are almost equal (4 ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the
// difference between val1 and val2 doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "StargazerTypes.h"
#include "gtest/gtest.h"

using namespace stargazer;

TEST(Landmark, CornerPoints) {
    Landmark a = Landmark(0);

    // Count
    ASSERT_EQ(3, a.points.size());

    // Coordinates
    ASSERT_EQ(0, a.points[0][(int)POINT::X]);
    ASSERT_EQ(0, a.points[0][(int)POINT::Y]);

    ASSERT_EQ(3 * Landmark::kGridDistance, a.points[1][(int)POINT::X]);
    ASSERT_EQ(0 * Landmark::kGridDistance, a.points[1][(int)POINT::Y]);

    ASSERT_EQ(3 * Landmark::kGridDistance, a.points[2][(int)POINT::X]);
    ASSERT_EQ(3 * Landmark::kGridDistance, a.points[2][(int)POINT::Y]);
}

TEST(Landmark, TestID4184) {
    // 4184 in Hex is 16772 in decimal
    Landmark a = Landmark(16772);

    // Count
    ASSERT_EQ(7, a.points.size());

    ASSERT_EQ(2 * Landmark::kGridDistance, a.points[3][(int)POINT::X]);
    ASSERT_EQ(0 * Landmark::kGridDistance, a.points[3][(int)POINT::Y]);

    ASSERT_EQ(3 * Landmark::kGridDistance, a.points[4][(int)POINT::X]);
    ASSERT_EQ(1 * Landmark::kGridDistance, a.points[4][(int)POINT::Y]);

    ASSERT_EQ(0 * Landmark::kGridDistance, a.points[5][(int)POINT::X]);
    ASSERT_EQ(2 * Landmark::kGridDistance, a.points[5][(int)POINT::Y]);

    ASSERT_EQ(2 * Landmark::kGridDistance, a.points[6][(int)POINT::X]);
    ASSERT_EQ(3 * Landmark::kGridDistance, a.points[6][(int)POINT::Y]);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

