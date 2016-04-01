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
#include "internal/CostFunction.h"
#include "util_print/prettyprint.h"

//A google test function (uncomment the next function, add code and
//change the names TestGroupName and TestName)
TEST(CostFunctor, identity) {
  LM2ImgReprojectionFunctor f = LM2ImgReprojectionFunctor(0, 0, 2, 1);
  double residuals[2];
  std::array<double, (int) POSE::N_PARAMS> lm_pose = {0, 0, 10, 0, 0, 0};
  std::array<double, (int) POSE::N_PARAMS> camera_pose = {0, 0, 0, 0, 0, 0};
  std::array<double, (int) POSE::N_PARAMS> camera_intrinsics = {1, 0, 0, 1, 1, 90};
//  std::cout << "lm_pose: " << lm_pose << std::endl;
//  std::cout << "camera_pose: " << camera_pose<< std::endl;
//  std::cout << "camera_intrinsics: " << camera_intrinsics<< std::endl;
  f.operator()(&lm_pose[0], &camera_pose[0], &camera_intrinsics[0], residuals);

  ASSERT_NEAR(0.2, residuals[0], 10e-10);
  ASSERT_NEAR(0.1, residuals[1], 10e-10);
}

//!TODO Deprecated
//TEST(CostFunctor, Landmark2Img) {
//  // See matlab file
//  LM2ImgReprojectionFunctor f = LM2ImgReprojectionFunctor(5.177542142857143e+02,-14.674357142857131, 2, 1);
//  double residuals[2];
//  std::array<double, (int) POSE::N_PARAMS> lm_pose = {2, 5, 3, 0, 0, -90};
//  std::array<double, (int) POSE::N_PARAMS> camera_pose = {1.5, 5.5, 0.2, 0, 0, 0};
//  std::array<double, (int) POSE::N_PARAMS> camera_intrinsics = {279.082, 368.246, 234.506, 1, 1, 90};
//  f.operator()(&lm_pose[0], &camera_pose[0], &camera_intrinsics[0], residuals);
//  ASSERT_NEAR(0.0, residuals[0], 10e-10);
//  ASSERT_NEAR(0.0, residuals[1], 10e-10);
//}