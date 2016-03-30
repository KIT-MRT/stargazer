//google test docs
//wiki page: https://code.google.com/p/googletest/w/list
//primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
//FAQ: https://code.google.com/p/googletest/wiki/FAQ
//advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
//samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
//List of some basic tests fuctions:
//Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
//ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
//ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
//ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
//ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
//ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
//ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4 ULPs)
//ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4 ULPs)
//ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2 doesn't exceed the given absolute error
//
//Note: more information about ULPs can be found here: http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
//Example of two unit test:
//TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
//TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "gtest/gtest.h"
#include <iostream>
#include "StargazerTypes.h"
#include "util_print/prettyprint.h"

TEST(Landmark, CornerPoints) {
Landmark a = Landmark(0);

//Count
ASSERT_EQ(3, a.points.size());

// IDS
ASSERT_EQ(0, std::get<(int)POINT::ID>(a.points[0]));
ASSERT_EQ(3, std::get<(int)POINT::ID>(a.points[1]));
ASSERT_EQ(15, std::get<(int)POINT::ID>(a.points[2]));

// Coordinates
ASSERT_EQ(0, std::get<(int)POINT::X>(a.points[0]));
ASSERT_EQ(0, std::get<(int)POINT::Y>(a.points[0]));

ASSERT_EQ(3 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[1]));
ASSERT_EQ(0 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[1]));

ASSERT_EQ(3 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[2]));
ASSERT_EQ(3 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[2]));

}


TEST(Landmark, TestID2) {
  Landmark a = Landmark(2);

//Count
  ASSERT_EQ(4, a.points.size());

// IDS
  ASSERT_EQ(0, std::get<(int)POINT::ID>(a.points[0]));
  ASSERT_EQ(3, std::get<(int)POINT::ID>(a.points[1]));
  ASSERT_EQ(15, std::get<(int)POINT::ID>(a.points[2]));
  ASSERT_EQ(1, std::get<(int)POINT::ID>(a.points[3]));

}

TEST(Landmark, TestID4184) {
  // 4184 in Hex is 16772 in decimal
  Landmark a = Landmark(16772);

//Count
  ASSERT_EQ(7, a.points.size());

// IDS
//  for (auto& i:a.points) std::cout << std::get<(int)POINT::ID>(i) << std::endl;
  ASSERT_EQ(0,  std::get<(int)POINT::ID>(a.points[0]));
  ASSERT_EQ(3,  std::get<(int)POINT::ID>(a.points[1]));
  ASSERT_EQ(15, std::get<(int)POINT::ID>(a.points[2]));

  ASSERT_EQ(2,  std::get<(int)POINT::ID>(a.points[3]));  //0x04
  ASSERT_EQ(2 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[3]));
  ASSERT_EQ(0 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[3]));

  ASSERT_EQ(7,  std::get<(int)POINT::ID>(a.points[4]));  //0x20
  ASSERT_EQ(3 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[4]));
  ASSERT_EQ(1 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[4]));

  ASSERT_EQ(8, std::get<(int)POINT::ID>(a.points[5])); // 0x200
  ASSERT_EQ(0 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[5]));
  ASSERT_EQ(2 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[5]));

  ASSERT_EQ(14, std::get<(int)POINT::ID>(a.points[6])); // 0x400
  ASSERT_EQ(2 * Landmark::kGridDistance, std::get<(int)POINT::X>(a.points[6]));
  ASSERT_EQ(3 * Landmark::kGridDistance, std::get<(int)POINT::Y>(a.points[6]));
}



