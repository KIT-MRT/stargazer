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
#include <ceres/rotation.h>
#include <iostream>

//A google test function (uncomment the next function, add code and
//change the names TestGroupName and TestName)
//TEST(TurnPoint, OneDimensionalForward) {
//  double point[3] = {1, 0, 0};
//  double angleAxis[3] = {0, 0, M_PI / 2};
//  double result[3];
//  ceres::AngleAxisRotatePoint(angleAxis, point, result);
//  double correct[3] = {0, 1, 0};
//  ASSERT_NEAR(correct[0], result[0], 10e-10);
//  ASSERT_NEAR(correct[1], result[1], 10e-10);
//  ASSERT_NEAR(correct[2], result[2], 10e-10);
//}
//
//TEST(TurnPoint, OneDimensionalBackward) {
//  double point[3] = {1, 0, 0};
//  double angleAxis[3] = {0, 0, -M_PI / 2};
//  double result[3];
//  ceres::AngleAxisRotatePoint(angleAxis, point, result);
//  double correct[3] = {0, -1, 0};
//  ASSERT_NEAR(correct[0], result[0], 10e-10);
//  ASSERT_NEAR(correct[1], result[1], 10e-10);
//  ASSERT_NEAR(correct[2], result[2], 10e-10);
//}
//
//TEST(TransformPoint, NurTranslation) {
//  double point[6] = {1, 2, 3};
//  double angleAxis[3] = {0, 0, 0};
//  double pose[3] = {1, 2, 3};
//  double result[3];
//  ceres::AngleAxisRotatePoint(angleAxis, point, result);
//  result[0] += pose[0];
//  result[1] += pose[1];
//  result[2] += pose[2];
//  double correct[3] = {2, 4, 6};
//  ASSERT_NEAR(correct[0], result[0], 10e-10);
//  ASSERT_NEAR(correct[1], result[1], 10e-10);
//  ASSERT_NEAR(correct[2], result[2], 10e-10);
//}
//
//TEST(TransformPoint, 90deg) {
//  double point[6] = {1, 2, 3};
//  double angleAxis[3] = {0, M_PI, 0};
//  double pose[3] = {1, 2, 3};
//  double result[3];
//  ceres::AngleAxisRotatePoint(angleAxis, point, result);
//  result[0] += pose[0];
//  result[1] += pose[1];
//  result[2] += pose[2];
//  double correct[3] = {0, 4, 0};
//  ASSERT_NEAR(correct[0], result[0], 10e-10);
//  ASSERT_NEAR(correct[1], result[1], 10e-10);
//  ASSERT_NEAR(correct[2], result[2], 10e-10);
//}

TEST(EulerAngles, Trivial) {
// Euler angle (in degrees) rotation representations.
//
// The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z}
// axes, respectively.  They are applied in that same order, so the
// total rotation R is Rz * Ry * Rx.
  double eulerAngles[3] = {0, 0, 0};
  double point[3] = {1, 0, 0};
  double correct[3] = {1, 0, 0};

  double R[9]; //rotation matrix
  double camera_axis_angles[3];
  double result[3];

  ceres::EulerAnglesToRotationMatrix(eulerAngles,3,R);
  ceres::RotationMatrixToAngleAxis(R, camera_axis_angles);
  ceres::AngleAxisRotatePoint(camera_axis_angles, point, result);

  ASSERT_NEAR(correct[0], result[0], 10e-10);
  ASSERT_NEAR(correct[1], result[1], 10e-10);
  ASSERT_NEAR(correct[2], result[2], 10e-10);
}



