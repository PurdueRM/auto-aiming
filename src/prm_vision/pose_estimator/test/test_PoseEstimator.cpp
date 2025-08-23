#include "PoseEstimator.h"
#include "gtest/gtest.h"

class PoseEstimatorTest : public ::testing::Test
{
protected:
    PoseEstimator *pose_estimator;

    void SetUp() override
    {
        pose_estimator = new PoseEstimator();
    }

    void TearDown() override
    {
        delete pose_estimator;
    }
};

TEST_F(PoseEstimatorTest, test_estimateTranslation_basic)
{
    cv::Mat tvec, rvec;
    std::vector<cv::Point2f> image_points = {
        {345, 522}, {342, 544}, {396, 521}, {395, 545}};
    pose_estimator->estimateTranslation(image_points, false, tvec, rvec);

    // Check translation vector (current best guess)
    EXPECT_NEAR(tvec.at<double>(0), -535.0, 50.0);
    EXPECT_NEAR(tvec.at<double>(1), 27.0, 10.0);
    EXPECT_NEAR(tvec.at<double>(2), 2331, 50.0);

    // Check rotation vector (current best guess)
    EXPECT_NEAR(rvec.at<double>(0), -0.198, 0.1);
    EXPECT_NEAR(rvec.at<double>(1), 0.2654, 0.1);
    EXPECT_NEAR(rvec.at<double>(2), 0.0217, 0.01);
}