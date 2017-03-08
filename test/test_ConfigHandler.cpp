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
// ASSERT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two
// float values are almost equal (4 ULPs)
// ASSERT_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two
// double values are almost equal (4 ULPs)
// ASSERT_EQ(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the
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
//	  ASSERT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "StargazerConfig.h"
#include "gtest/gtest.h"

using namespace stargazer;

TEST(ConfigHandler, Read) {

    std::string cfgfile{"res/stargazer.yaml"};
    camera_params_t camera_intrinsics;
    landmark_map_t landmarks;
    ASSERT_NO_THROW(readConfig(cfgfile, camera_intrinsics, landmarks));

    ASSERT_EQ(279.082, camera_intrinsics[(int)INTRINSICS::fu]);
    ASSERT_EQ(279.082, camera_intrinsics[(int)INTRINSICS::fv]);
    ASSERT_EQ(368.246, camera_intrinsics[(int)INTRINSICS::u0]);
    ASSERT_EQ(234.506, camera_intrinsics[(int)INTRINSICS::v0]);

    ASSERT_EQ(19, landmarks.size());
}

TEST(ConfigHandler, Write) {
    std::string cfgfile{"res/stargazer.yaml"};
    camera_params_t camera_intrinsics;
    landmark_map_t landmarks;
    ASSERT_NO_THROW(readConfig(cfgfile, camera_intrinsics, landmarks));

    std::string testfile{"res/stargazer_test.yaml"};
    ASSERT_NO_THROW(writeConfig(testfile, camera_intrinsics, landmarks));
    camera_params_t camera_intrinsics_test;
    landmark_map_t landmarks_test;
    ASSERT_NO_THROW(readConfig(testfile, camera_intrinsics_test, landmarks_test));
    ASSERT_EQ(landmarks.size(), landmarks_test.size());
}
