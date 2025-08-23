#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "LBFGSB.h" // L-BFGS-B optimization library for yaw estimation

// Classes
#include "ValidityFilter.hpp"

// Constants for armor plates
#define LIGHTBAR_HALF_HEIGHT 54.f / 2.f
#define SMALL_ARMOR_HALF_WIDTH 134.f / 2.f
#define LARGE_ARMOR_HALF_WIDTH 225.f / 2.f

using Eigen::VectorXd;
using namespace LBFGSpp;

class PoseEstimator
{
public:
    PoseEstimator() {}
    ~PoseEstimator() {}

    ValidityFilter validity_filter_ = ValidityFilter();

    // Class methods
    double estimateYaw(const double yaw_guess, const std::vector<cv::Point2f> &image_points, const cv::Mat &tvec);
    void estimateTranslation(const std::vector<cv::Point2f> &image_points, bool largeArmor, cv::Mat &tvec, cv::Mat &rvec);
    bool isValid(float x, float y, float z, std::string &auto_aim_status, bool &reset_kalman);

    // Setters
    void setNumFramesToFireAfter(int num_frames_to_fire_after) { _num_frames_to_fire_after = num_frames_to_fire_after; }
    void setAllowedMissedFramesBeforeNoFire(int allowed_missed_frames_before_no_fire) { _allowed_missed_frames_before_no_fire = allowed_missed_frames_before_no_fire; }

private:
    // Class methods
    double computeLoss(double yaw_guess, std::vector<cv::Point2f> image_points, cv::Mat tvec);
    double gradientWrtYawFinitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat tvec);

    // Class variables
    int _consecutive_tracking_frames_ctr = 0;
    int _num_frames_to_fire_after = 3;
    int _allowed_missed_frames_before_no_fire = 150;
    int _remaining_missed_frames_before_no_fire = 0; // Gets reset when we have a valid pose estimate
    std::chrono::time_point<std::chrono::system_clock> _last_fire_time;

    // Validity filter parameters
    int _lock_in_after = 3;
    float _max_distance = 10000;
    float _min_distance = 10;
    float _max_shift_distance = 150;
    int _prev_len = 5;

    /**
     * @brief Functor for the loss function and gradient computation.
     *
     * Used for the L-BFGS-B optimization library.
     *
     * @param poseEstimator Reference to the PoseEstimator instance.
     * @param image_points The ground truth image points of the detected armor.
     * @param tvec The translation vector of the detected armor.
     * @param x The input vector containing a guess for the yaw angle.
     * @return double The loss value.
     * @return VectorXd The gradient vector.
     */
    class LossAndGradient
    {
    public:
        // Constructor to accept reference to PoseEstimator instance
        LossAndGradient(PoseEstimator &poseEstimator, std::vector<cv::Point2f> image_points, cv::Mat tvec) : poseEstimator(poseEstimator), image_points(image_points), tvec(tvec) {}

        double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad)
        {
            double yaw = x(0);
            double loss = poseEstimator.computeLoss(yaw, image_points, tvec);
            grad(0) = poseEstimator.gradientWrtYawFinitediff(yaw, image_points, tvec);
            return loss;
        }

    private:
        std::vector<cv::Point2f> image_points;
        cv::Mat tvec;
        PoseEstimator &poseEstimator;
    };

    // Camera intrinsics
    const cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << 1019.109, 0, 601.885,
                                   0, 1016.785, 521.005,
                                   0, 0, 1);
    const cv::Mat DISTORTION_COEFFS = (cv::Mat_<double>(1, 4) << -0.1088, -0.0721, -0.000847, 0.0);

    /* 3D object points (measured armor dimensions)
     * Coordinate system (camera is facing us):
     *        
     *                      +x (pitch)
     *               +---->
     *             / |
     * +z (roll)  L  v +y (yaw)
     *  
     */
    const std::vector<cv::Point3f> SMALL_ARMOR_OBJECT_POINTS = {
        {-SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };

    const std::vector<cv::Point3f> LARGE_ARMOR_OBJECT_POINTS = {
        {-LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };
};

#endif // POSE_ESTIMATOR_HPP