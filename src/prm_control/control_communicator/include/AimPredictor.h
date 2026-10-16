#ifndef AIM_PREDICTOR_H
#define AIM_PREDICTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <chrono>
#include <stdio.h>
#include <math.h>

typedef struct Coordinate {
    cv::Mat tvec;
    cv::Mat rvec;
} coordinate_t;

class AimPredictor {
    public:
        AimPredictor();
        ~AimPredictor();

        coordinate_t predictAim(coordinate_t armor_position);
        void reset();
        void setGimbalState(coordinate_t position, coordinate_t velocity);
    private:
        double projectile_speed = 25.0; // m/s
        double acceleration_variance = 1000000.0; // 1000 mm/s^2 sd
        std::chrono::time_point<std::chrono::steady_clock> last_prediction_time;
        coordinate_t gimbal_position;
        coordinate_t gimbal_velocity;
        coordinate_t toGlobalCoordinate(coordinate_t coordinate);
        coordinate_t toLocalCoordinate(coordinate_t coordinate);
        coordinate_t interceptionPoint(coordinate_t armor_position, coordinate_t armor_velocity);

        // Constant-velocity Kalman filter tracking the armor's global position and velocity
        cv::KalmanFilter armor_kalman_filter;
        bool armor_kalman_initialized = false;
        void initArmorKalmanFilter();
        void updateArmorKalmanFilter(const cv::Mat &measured_position, double dt,
                                      cv::Mat &estimated_position, cv::Mat &estimated_velocity);
};

#endif // AIM_PREDICTOR_H