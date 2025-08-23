#include "PoseEstimator.h"

void PoseEstimator::estimateTranslation(const std::vector<cv::Point2f> &image_points, bool largeArmor, cv::Mat &tvec, cv::Mat &rvec)
{
    // SolvePnP requires 3D object points
    std::vector<cv::Point3f> object_points;
    if (largeArmor)
    {
        object_points = LARGE_ARMOR_OBJECT_POINTS;
    }
    else
    {
        object_points = SMALL_ARMOR_OBJECT_POINTS;
    }

    std::string auto_aim_status;
    cv::solvePnP(object_points, image_points, CAMERA_MATRIX, DISTORTION_COEFFS, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

/**
 * @brief Estimate the yaw angle of the detected armor.
 *
 * Uses L-BFGS-B optimization to minimize the reprojection error given a guess for the yaw angle.
 * L-BFGS-B is a limited-memory quasi-Newton bounded optimization algorithm that works well for this one-dimensional problem.
 *
 * NOTE: This function assumes that pitch and roll are zero.
 * This is valid due to robot design constraints. Pitch actually is ~15 degrees, but 0 seems to work well.
 *
 * @param yaw_guess The initial guess for the yaw angle in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The estimated yaw angle in radians.
 */
double PoseEstimator::estimateYaw(const double yaw_guess, const std::vector<cv::Point2f> &image_points, const cv::Mat &tvec)
{
    // We only have one parameter to optimize: yaw
    const int n = 1;
    double fx;

    // L-BFGS-B parameters (experimentally tuned)
    LBFGSBParam<double> param;
    param.epsilon = 0.4;
    param.max_iterations = 50;

    LBFGSBSolver<double> solver(param);
    LossAndGradient loss(*this, image_points, tvec);

    // Bound yaw to -60 and 60 degrees
    VectorXd lb = VectorXd::Constant(n, -CV_PI / 3);
    VectorXd ub = VectorXd::Constant(n, CV_PI / 3);
    VectorXd x = VectorXd::Constant(n, yaw_guess);

    solver.minimize(loss, x, fx, lb, ub);
    return x(0);
}

/**
 * @brief Check if the estimated pose is valid based on the validity filter. Also set the auto-aim status.
 *
 * Rough overview of the auto-aiming system:
 * 1. IDLING: No valid detections, waiting for a valid detection.
 * 2. TRACKING: Enough valid detections to track the target.
 * 3. STOPPING: Too many invalid detections, stop tracking the target.
 * 4. FIRE: We have been tracking the target for enough frames, fire at the target.
 *
 * @param x The x-coordinate of the detected armor.
 * @param y The y-coordinate of the detected armor.
 * @param z The z-coordinate of the detected armor.
 * @param auto_aim_status Reference to the status of the auto-aiming system. Overwritten by this function.
 * @return true If the pose is valid for auto-aiming based on the validity filter.
 */
bool PoseEstimator::isValid(float x, float y, float z, std::string &auto_aim_status, bool &reset_kalman)
{
    // Validates the pose estimate and sets the auto-aim status. Also check if we should reset the Kalman filter.
    reset_kalman = validity_filter_.shouldResetKalman(x, y, z);

    // Stopping motor, we want to publish 0s for predicted armor (no auto aiming)
    if (validity_filter_.state == STOPPING)
    {
        auto_aim_status = "STOPPING";
        _consecutive_tracking_frames_ctr = 0;
        return false;
    }

    // We have a detection but not yet tracking
    else if (validity_filter_.state == IDLING)
    {
        auto_aim_status = "IDLING";
        _consecutive_tracking_frames_ctr = 0;
        return false;
    }

    // We have enough valid detections to track the target
    else if (validity_filter_.state == TRACKING)
    {
        // if validity filter valid for last number of frames, increment locked_in_frames
        if (validity_filter_.getLockInCounter() == validity_filter_.getLockInAfter())
        {
            _consecutive_tracking_frames_ctr++;
        }
        else
        {
            // We are not locked in, reset locked_in_frames
            _consecutive_tracking_frames_ctr = 0;
        }

        // if tracking and we have locked in for enough frames, fire at the target
        if (_consecutive_tracking_frames_ctr >= _num_frames_to_fire_after)
        {
            auto_aim_status = "FIRE";
            _remaining_missed_frames_before_no_fire = _allowed_missed_frames_before_no_fire;
        }
        else
        {
            // If currently firing, allow for a few more frames to fire
            if (_remaining_missed_frames_before_no_fire > 0)
            {
                _remaining_missed_frames_before_no_fire--;
                auto_aim_status = "FIRE";
            }
            else
            {
                auto_aim_status = "TRACKING";
            }
        }
    }

    return true;
}

/**
 * @brief Compute the gradient of the loss function with respect to yaw using finite differences.
 *
 * @param yaw The yaw angle in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The gradient value.
 */
double PoseEstimator::gradientWrtYawFinitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat tvec)
{
    double h = 1e-4;
    double loss_plus = computeLoss(yaw + h, image_points, tvec);
    double loss_minus = computeLoss(yaw - h, image_points, tvec);
    double grad = (loss_plus - loss_minus) / (2 * h);
    return grad;
}

/**
 * @brief Compute the loss between image points and reprojected points based on the yaw guess.
 *
 * @param yaw_guess The yaw angle guess in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The loss value.
 */
double PoseEstimator::computeLoss(double yaw_guess, std::vector<cv::Point2f> image_points, cv::Mat tvec)
{
    // We assume -15 deg pitch and 0 roll
    double pitch = -15 * (M_PI / 180);

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(yaw_guess), 0, sin(yaw_guess),
                  0, 1, 0,
                  -sin(yaw_guess), 0, cos(yaw_guess));
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                  0, cos(pitch), -sin(pitch),
                  0, sin(pitch), cos(pitch));
    cv::Mat R = Ry * Rx;

    // Reproject the 3D object points using predicted yaw to the image plane
    std::vector<cv::Point2f>
        projected_points;
    cv::projectPoints(SMALL_ARMOR_OBJECT_POINTS, R, tvec, CAMERA_MATRIX, DISTORTION_COEFFS, projected_points);

    // Loss = sqrt(0.25 * sum((x - x')^2 + (y - y')^2))
    double loss = 0;
    for (int i = 0; i < image_points.size(); i++)
    {
        cv::Point2d diff = image_points[i] - projected_points[i];
        loss += 0.25 * (diff.x * diff.x + diff.y * diff.y); // Squared reprojection error
    }
    return pow(loss, 0.5);
}