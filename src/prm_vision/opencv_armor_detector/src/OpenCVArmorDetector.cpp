#include "OpenCVArmorDetector.h"

void OpenCVArmorDetector::setConfig(DetectorConfig config)
{
    _config = config;

    // Set the color limits based on config
    _blue_lower_limit = cv::Scalar(105 - _config._hue_range_limit, _config._saturation_lower_limit, _config._value_lower_limit);
    _blue_upper_limit = cv::Scalar(105 + _config._hue_range_limit, 255, 255);
    _red_lower_limit_1 = cv::Scalar(0, _config._saturation_lower_limit, _config._value_lower_limit);
    _red_upper_limit_1 = cv::Scalar(_config._hue_range_limit, 255, 255);
    _red_lower_limit_2 = cv::Scalar(179 - _config._hue_range_limit, _config._saturation_lower_limit, _config._value_lower_limit);
    _red_upper_limit_2 = cv::Scalar(179, 255, 255);

    // Set the other config variables
    _targetColor = config._target_color;
    _max_missed_frames = config._max_missed_frames;
    _reduce_search_area = config._reduce_search_area;
}

std::vector<_Float32> OpenCVArmorDetector::search(cv::Mat &frame)
{
    std::vector<_Float32> detected_keypoints(8, 0);
    static auto last_time = std::chrono::steady_clock::now(); // Static to persist across calls

    if (_reset_search_area)
    {
        _search_area[0] = 0;
        _search_area[1] = 0;
        _search_area[2] = WIDTH;
        _search_area[3] = HEIGHT;
        _reset_search_area = false;
        _missed_frames = 0;
    }
    cv::Mat croppedFrame = frame(cv::Range(_search_area[1], _search_area[3]), cv::Range(_search_area[0], _search_area[2])).clone();

    // Detect the armor in the cropped frame
    std::vector<cv::Point2f> points = detectArmorsInFrame(croppedFrame);
    _frame_count++;

// Display the cropped frame for debugging
#ifdef DEBUG
    cv::resize(croppedFrame, croppedFrame, cv::Size(WIDTH / 2, HEIGHT / 2));

    // Create a static window name
    const std::string window_name = "Detection Results";
    cv::imshow(window_name, croppedFrame);

    // Update the window title
    cv::setWindowTitle(window_name,
                       "detected: " + std::to_string(_detected_frame) + " / " +
                           std::to_string(_frame_count) + " (" +
                           std::to_string(_detected_frame * 100 / _frame_count) + "%) and missed: " + std::to_string(_missed_frames) + std::string(" frames"));

    cv::waitKey(30);
#endif

    // If we didn't find an armor for a few frames (ROS2 param), reset the search area 
    if (points.size() == 0)
    {
        _missed_frames++;
        if (_missed_frames >= _max_missed_frames)
        {
            _reset_search_area = true;
        }
    }
    else
    {
        // We found an armor, so reset the missed frames and return the keypoints
        _missed_frames = 0;
        std::vector<cv::Point2f> image_points;
        for (int i = 0; i < 4; i++)
        {
            detected_keypoints[i * 2] = points.at(i).x + _search_area[0];
            detected_keypoints[i * 2 + 1] = points.at(i).y + _search_area[1];

            image_points.emplace_back(cv::Point2f(points.at(i).x + _search_area[0], points.at(i).y + _search_area[1]));
        }

        if (_reduce_search_area)
        {
            // Change the search area to the bounding box of the armor with a 50 pixel buffer
            _reset_search_area = false; // We got a detection, so don't reset the search area next frame
            int x_min = (int)std::min(detected_keypoints[0], detected_keypoints[2]);
            int x_max = (int)std::max(detected_keypoints[4], detected_keypoints[6]);
            int y_min = (int)std::min(detected_keypoints[1], detected_keypoints[5]);
            int y_max = (int)std::max(detected_keypoints[3], detected_keypoints[7]);
            _search_area[0] = std::max(x_min - 50, 0);
            _search_area[1] = std::max(y_min - 50, 0);
            _search_area[2] = std::min(x_max + 50, WIDTH);
            _search_area[3] = std::min(y_max + 50, HEIGHT);
        }
        else
        {
            // Reset the search area to the full frame
            _reset_search_area = true;
        }

        _detected_frame++;
    }

    return detected_keypoints;
}

/**
 * @brief Detects an armor plate in the given frame.
 *
 * We Gaussian blur the input frame and convert it to HSV color space.
 * A mask is created for the desired color ranges (blue and red) and we find contours in the mask.
 * We then fit ellipses to the contours to find candidate light bars.
 * Finally, we filter for probable light bars, and attempt to find a match forming an armor plate.
 *
 * @param frame The input frame in which to search for armor.
 * @return A vector of two cv::RotatedRect objects representing the detected armor, or an empty vector if no armor is found.
 */
std::vector<cv::Point2f> OpenCVArmorDetector::detectArmorsInFrame(cv::Mat &frame)
{
    cv::Mat hsvFrame, result;

    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

    if (_targetColor == BLUE)
    {
        // Mask for blue color
        cv::inRange(hsvFrame, _blue_lower_limit, _blue_upper_limit, result);
    }
    else
    {
        // Create mask for both red ranges, mask separately and combine
        cv::Mat lower_red_inrange, upper_red_inrange;
        cv::inRange(hsvFrame, _red_lower_limit_1, _red_upper_limit_1, lower_red_inrange);
        cv::inRange(hsvFrame, _red_lower_limit_2, _red_upper_limit_2, upper_red_inrange);
        cv::bitwise_or(lower_red_inrange, upper_red_inrange, result);
    }

    // Find contours in the masked image
    std::vector<std::vector<cv::Point>> contours;
    cv::Canny(result, result, 30.0, 90.0, 3, false);
    cv::findContours(result, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // We find the light bar candidates among the contours
    std::vector<cv::RotatedRect> light_bar_candidates;
    for (auto contour : contours)
    {
        if (contour.size() > 20)
        {
            // Use convex hull to get a convex contour
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);

            auto rect_bounding = cv::boundingRect(hull);
            cv::RotatedRect rect = cv::RotatedRect(cv::Point2f(rect_bounding.x + rect_bounding.width / 2, rect_bounding.y + rect_bounding.height / 2), cv::Size2f(rect_bounding.width, rect_bounding.height), 0);

            // draw rotated rectangle
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
            {
                cv::line(result, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
            }

            // Rect rotates, so we need to ensure height is always the longer side
            if (rect.angle > 45)
            {
                std::swap(rect.size.width, rect.size.height);
                rect.angle -= 90;
            }

            if (isLightBar(rect))
            {
                light_bar_candidates.push_back(rect);
            }
        }
    }

    // Give priority to the light bar with the leftmost center
    // TODO: Have a better metric such as distance from last detected armor
    std::sort(light_bar_candidates.begin(), light_bar_candidates.end(), [](cv::RotatedRect &a, cv::RotatedRect &b)
              { return a.center.x < b.center.x; });

    // If we have at least 2 light bars, we can attempt to match them to find an armor
    if (light_bar_candidates.size() >= 2)
    {
        for (int i = 0; i < light_bar_candidates.size() - 1; i++)
        {
            for (int j = i + 1; j < light_bar_candidates.size(); j++)
            {
                cv::RotatedRect rect1 = light_bar_candidates[i];
                cv::RotatedRect rect2 = light_bar_candidates[j];

                // Check if the pair of light bars likely form an armor plate
                if (isArmor(rect1, rect2))
                {
                    // We have found a match, return the pair (sorted left, right)
                    auto &first = (rect1.center.x < rect2.center.x) ? rect1 : rect2;
                    auto &second = (rect1.center.x < rect2.center.x) ? rect2 : rect1;

                    std::vector<cv::Point2f> armor_points_1 = rectToPoint(first);
                    std::vector<cv::Point2f> armor_points_2 = rectToPoint(second);

                    // Draw a dot on the top and bottom of each light bar using rectToPoint
#ifdef DEBUG
                    cv::circle(frame, armor_points_1[0], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_1[1], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_2[0], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_2[1], 0, cv::Scalar(0, 255, 0), -1);
#endif
                    return {rectToPoint(first)[0], rectToPoint(first)[1], rectToPoint(second)[0], rectToPoint(second)[1]};
                }
            }
        }
    }
    return {}; // no armor found
}

void OpenCVArmorDetector::drawRotatedRect(cv::Mat &frame, cv::RotatedRect &rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
}

/**
 * @brief Checks if the given RotatedRect is likely to be a light bar.
 *
 * We check the width, height, angle, and aspect ratio of the RotatedRect.
 *
 * @param rect The RotatedRect to check.
 * @return true if the RotatedRect is likely to be a light bar, false otherwise.
 */
bool OpenCVArmorDetector::isLightBar(cv::RotatedRect &rect)
{
    if (rect.size.width < LIGHT_BAR_WIDTH_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.size.height < LIGHT_BAR_HEIGHT_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.angle < 180 - LIGHT_BAR_ANGLE_LIMIT && rect.angle > LIGHT_BAR_ANGLE_LIMIT)
    {
        return false;
    }

    // Note we check height/width not width/height (standard aspect ratio is width/height)
    if (rect.size.height / rect.size.width < LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT)
    {
        return false;
    }
    return true;
}

/**
 * @brief Checks if a pair of given RotatedRects are likely to form an armor plate.
 *
 * We check the angle difference, aspect ratio ratio, y position ratio, height ratio, and aspect ratio of the two RotatedRects.
 *
 * @param left_rect The left RotatedRect.
 * @param right_rect The right RotatedRect.
 * @return true if the pair of RotatedRects are likely to form an armor plate, false otherwise.
 */
bool OpenCVArmorDetector::isArmor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect)
{
    float angle_diff = std::abs(left_rect.angle - right_rect.angle);

    // Light Bar Parallel Check
    if (angle_diff > ARMOR_ANGLE_DIFF_LIMIT && angle_diff < 180 - ARMOR_ANGLE_DIFF_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Ratio Difference Check
    float ar_left = left_rect.size.height / left_rect.size.width;
    float ar_right = right_rect.size.height / right_rect.size.width;
    if (ar_left / ar_right > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT ||
        ar_right / ar_left > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT)
    {
        return false;
    }

    // X Position distance Check
    if (std::abs(left_rect.center.x - right_rect.center.x) < left_rect.size.width)
    {
        return false;
    }

    // Light Bar Y Position Ratio Check
    float avg_height = (left_rect.size.height + right_rect.size.height) / 2;
    if (std::abs(left_rect.center.y - right_rect.center.y) / avg_height > ARMOR_Y_DIFF_LIMIT)
    {
        return false;
    }

    // Height Ratio Check
    if (left_rect.size.height / right_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT ||
        right_rect.size.height / left_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Check
    float max_h = std::max(left_rect.size.height, right_rect.size.height);
    float w = cv::norm(left_rect.center - right_rect.center);
    if (w / max_h > ARMOR_ASPECT_RATIO_LIMIT)
    {
        return false;
    }

    return true;
}

/**
 * @brief Converts a RotatedRect to a vector of points.
 *
 * @param rect The RotatedRect to convert.
 * @return A vector of points representing the RotatedRect.
 */
std::vector<cv::Point2f> OpenCVArmorDetector::rectToPoint(cv::RotatedRect &rect)
{
    float rad = rect.angle < 90 ? rect.angle * M_PI / 180.f : (rect.angle - 180) * M_PI / 180.f;
    float x_offset = rect.size.height * std::sin(rad) / 2.f;
    float y_offset = rect.size.height * std::cos(rad) / 2.f;

    std::vector<cv::Point2f> points;
    points = std::vector<cv::Point2f>();
    points.push_back(cv::Point2f(int(rect.center.x + x_offset), int(rect.center.y - y_offset)));
    points.push_back(cv::Point2f(int(rect.center.x - x_offset), int(rect.center.y + y_offset)));
    return points;
}