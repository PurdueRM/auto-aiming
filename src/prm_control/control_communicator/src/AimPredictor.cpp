#include "AimPredictor.h"

AimPredictor::AimPredictor() {
    this->last_prediction_time = std::chrono::steady_clock::now();
    this->gimbal_position.tvec = cv::Mat::zeros(3, 1, CV_64F);
    this->gimbal_position.rvec = cv::Mat::zeros(3, 1, CV_64F);
    this->gimbal_velocity.tvec = cv::Mat::zeros(3, 1, CV_64F);
    this->gimbal_velocity.rvec = cv::Mat::zeros(3, 1, CV_64F);
    this->initArmorKalmanFilter();
}

/**
 * @brief Initializes the Kalman filter for tracking armor positions
 */
void AimPredictor::initArmorKalmanFilter() {
    // State: [x, y, z, vx, vy, vz]^T, Measurement: [x, y, z]^T (constant-velocity model)
    this->armor_kalman_filter.init(6, 3, 0, CV_64F);

    cv::setIdentity(this->armor_kalman_filter.transitionMatrix);

    this->armor_kalman_filter.measurementMatrix = cv::Mat::zeros(3, 6, CV_64F);
    for (int i = 0; i < 3; i++) {
        this->armor_kalman_filter.measurementMatrix.at<double>(i, i) = 1.0f;
    }

    cv::setIdentity(this->armor_kalman_filter.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(this->armor_kalman_filter.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(this->armor_kalman_filter.errorCovPost, cv::Scalar::all(1.0));
}

/**
 * @brief Updates the Kalman filter with a new armor position measurement and estimates the current position and velocity
 * @param measured_position The measured position of the armor in 3D space
 * @param dt The time elapsed since the last update
 * @param estimated_position The estimated position of the armor after the update
 * @param estimated_velocity The estimated velocity of the armor after the update
 */
void AimPredictor::updateArmorKalmanFilter(const cv::Mat &measured_position, double dt,
                                            cv::Mat &estimated_position, cv::Mat &estimated_velocity) {
    if (!this->armor_kalman_initialized) {
        // Seed the filter with the first measurement and assume zero initial velocity
        this->armor_kalman_filter.statePost.at<double>(0) = measured_position.at<double>(0);
        this->armor_kalman_filter.statePost.at<double>(1) = measured_position.at<double>(1);
        this->armor_kalman_filter.statePost.at<double>(2) = measured_position.at<double>(2);
        this->armor_kalman_filter.statePost.at<double>(3) = 0.0f;
        this->armor_kalman_filter.statePost.at<double>(4) = 0.0f;
        this->armor_kalman_filter.statePost.at<double>(5) = 0.0f;
        this->armor_kalman_initialized = true;
    }

    // Refresh the position-velocity coupling in the transition matrix with the elapsed time
    for (int i = 0; i < 3; i++) {
        this->armor_kalman_filter.transitionMatrix.at<double>(i, i + 3) = static_cast<double>(dt);
    }

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt2 * dt2;

    this->armor_kalman_filter.processNoiseCov = cv::Mat::zeros(6, 6, CV_64F);
    for (int axis = 0; axis < 3; ++axis) {
        this->armor_kalman_filter.processNoiseCov.at<double>(axis, axis) = this->acceleration_variance * dt4 / 4.0;
        this->armor_kalman_filter.processNoiseCov.at<double>(axis, axis + 3) = this->acceleration_variance * dt3 / 2.0;
        this->armor_kalman_filter.processNoiseCov.at<double>(axis + 3, axis) = this->acceleration_variance * dt3 / 2.0;
        this->armor_kalman_filter.processNoiseCov.at<double>(axis + 3, axis + 3) = this->acceleration_variance * dt2;
    }

    this->armor_kalman_filter.predict();
    cv::Mat corrected = this->armor_kalman_filter.correct(measured_position);

    estimated_position = corrected.rowRange(0, 3).clone();
    estimated_velocity = corrected.rowRange(3, 6).clone();
}

AimPredictor::~AimPredictor() = default;

/**
 * @brief Drops the current track so the next measurement seeds a fresh one
 *
 * Re-initializing the filter (rather than only clearing the initialized flag) also
 * restores errorCovPost, otherwise the re-seeded state inherits the converged
 * covariance of the old track and the filter is overconfident in it.
 */
void AimPredictor::reset() {
    this->initArmorKalmanFilter();
    this->armor_kalman_initialized = false;
    this->last_prediction_time = std::chrono::steady_clock::now();
}

/**
 * @brief Sets the current state of the gimbal, including its position and velocity
 * @param position The current position of the gimbal in 3D space
 * @param velocity The current velocity of the gimbal in 3D space
 */
void AimPredictor::setGimbalState(coordinate_t position, coordinate_t velocity) {
    this->gimbal_position = position;
    this->gimbal_velocity = velocity;
}

/**
 * @brief Converts a local coordinate to a global coordinate based on the current gimbal state
 * @param coordinate The local coordinate to be converted
 * @return The corresponding global coordinate
 */
coordinate_t AimPredictor::toGlobalCoordinate(coordinate_t coordinate) {
    coordinate_t global_coordinate;

    cv::Mat gimbal_rotation_matrix;
    cv::Rodrigues(this->gimbal_position.rvec, gimbal_rotation_matrix);

    global_coordinate.tvec = gimbal_rotation_matrix * coordinate.tvec + this->gimbal_position.tvec;

    cv::Mat local_rotation_matrix;
    cv::Mat global_rotation_matrix;
    cv::Rodrigues(coordinate.rvec, local_rotation_matrix);
    global_rotation_matrix = gimbal_rotation_matrix * local_rotation_matrix;
    cv::Rodrigues(global_rotation_matrix, global_coordinate.rvec);

    return global_coordinate;
}

/**
 * @brief Converts a global coordinate to a local coordinate based on the current gimbal state
 * @param coordinate The global coordinate to be converted
 * @return The corresponding local coordinate
 */
coordinate_t AimPredictor::toLocalCoordinate(coordinate_t coordinate) {
    coordinate_t local_coordinate;

    cv::Mat gimbal_rotation_matrix;
    cv::Rodrigues(this->gimbal_position.rvec, gimbal_rotation_matrix);

    cv::Mat gimbal_rotation_matrix_inv = gimbal_rotation_matrix.inv();
    local_coordinate.tvec = gimbal_rotation_matrix_inv * (coordinate.tvec - this->gimbal_position.tvec);

    cv::Mat local_rotation_matrix;
    cv::Mat global_rotation_matrix;
    cv::Rodrigues(coordinate.rvec, global_rotation_matrix);
    local_rotation_matrix = gimbal_rotation_matrix_inv * global_rotation_matrix;
    cv::Rodrigues(local_rotation_matrix, local_coordinate.rvec);

    return local_coordinate;
}

/**
 * @brief Calculates the interception point where a projectile should meet a moving armor
 * 
 * Assumes constant velocity motion for both the armor and the projectile. Currently does
 * not account for gimbal velocity
 * 
 * @param armor_position The current position of the armor in 3D space
 * @param armor_velocity The current velocity of the armor in 3D space
 * @return The calculated interception point in 3D space
 */
coordinate_t AimPredictor::interceptionPoint(coordinate_t armor_position, coordinate_t armor_velocity) {
    double x = armor_position.tvec.at<double>(0);
    double z = armor_position.tvec.at<double>(2);
    double vx = armor_velocity.tvec.at<double>(0);
    double vz = armor_velocity.tvec.at<double>(2);
    double projectile_speed_mm_s = this->projectile_speed * 1000;
    
    double a = vx * vx + vz * vz - projectile_speed_mm_s * projectile_speed_mm_s;
    double b = 2 * (x * vx + z * vz);
    double c = x * x + z * z;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        // No real solution, return the current armor position as the interception point
        // I hope no robot moves faster than the projectile speed ^_^
        return armor_position;
    }
    
    double t_one = (-b + std::sqrt(discriminant)) / (2 * a);
    double t_two = (-b - std::sqrt(discriminant)) / (2 * a);

    // If two roots are positive, pick the smallest one since it represents the earliest interception
    double t = std::min(t_one, t_two);
    if (t < 0) {
        // Pick the positive root since we cannot aim at the past
        t = std::max(t_one, t_two);
    }

    coordinate_t interception_point;
    interception_point.tvec = armor_position.tvec + armor_velocity.tvec * t;
    interception_point.rvec = cv::Mat::zeros(3, 1, CV_64F);
    return interception_point;
}

/**
 * @brief Predicts the aim point for the gimbal to hit the moving armor
 * 
 * Uses the current armor position, the gimbal's state, and the projectile speed to calculate
 * the interception point where the gimbal should aim.
 * 
 * @param armor_position The current position of the armor in 3D space
 * @return The predicted aim point in 3D space
 */
coordinate_t AimPredictor::predictAim(coordinate_t armor_position) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - this->last_prediction_time).count();
    this->last_prediction_time = now;

    // Convert the armor position into global coordinates
    coordinate_t armor_global_position = this->toGlobalCoordinate(armor_position);

    // Run kalman filter to estimate armor's global position and velocity
    cv::Mat estimated_position;
    cv::Mat estimated_velocity;
    this->updateArmorKalmanFilter(armor_global_position.tvec, dt, estimated_position, estimated_velocity);
    armor_global_position.tvec = estimated_position;
    
    coordinate_t armor_global_velocity;
    armor_global_velocity.tvec = estimated_velocity;
    armor_global_velocity.rvec = cv::Mat::zeros(3, 1, CV_64F);

    // Calculate the relative position and velocity of the armor with respect to the gimbal
    // Interception calculation is rotation invariant, so no need to calculate relative rotation
    coordinate_t rel_position, rel_velocity;
    rel_position.tvec = estimated_position - this->gimbal_position.tvec;
    rel_velocity.tvec = estimated_velocity - this->gimbal_velocity.tvec;

    // Solve for the interception point
    coordinate_t interception_point = this->interceptionPoint(rel_position, rel_velocity);
    
    // Rotate the interception point back into the gimbal's local coordinate system
    cv::Mat gimbal_rotation_matrix;
    cv::Rodrigues(this->gimbal_position.rvec, gimbal_rotation_matrix);
    cv::Mat gimbal_rotation_matrix_inv = gimbal_rotation_matrix.inv();
    interception_point.tvec = gimbal_rotation_matrix_inv * interception_point.tvec;
    
    return interception_point;
}