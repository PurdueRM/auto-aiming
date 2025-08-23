#include <gtest/gtest.h>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "OpenCVArmorDetector.h"

#include <fstream>
#include <string>
#include <vector>
#include <sstream>

// Helpers
void readGT(std::string file_path, std::vector<std::vector<cv::Point2f>> &gt);
void testVideo(std::string video_path, std::string gt_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix);
void testImgs(std::string folder_path, std::string gt_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix);

/* DETECTOR REQUIREMENTS */
// Performance requirements
#define MIN_FPS 80 // Minimum FPS required

// Pre-selected images (tighter requirements since we know an armor is present)
#define MIN_DETECTION_RATE_EASY 0.95     // 95% detection rate required
#define MIN_DETECTION_RATE_FAR_BACK 0.90 // 90% detection rate required
#define MAX_LOSS_EASY 2                  // 2 pixels loss allowed
#define MAX_LOSS_FAR_BACK 3              // 3 pixels loss allowed

// Realistic videos (looser requirements due to spintop and obstruction)
#define MIN_DETECTION_RATE_FAR_BACK_SPIN_AND_MOVE 0.70 // 70% detection rate required
#define MIN_DETECTION_RATE_CLOSE_VIDEO 0.65            // 65% detection rate required
#define MAX_LOSS_FAR_BACK_SPIN_AND_MOVE 3              // 3 pixels loss allowed
#define MAX_LOSS_CLOSE_VIDEO 3                         // 3 pixels loss allowed
/* END DETECTOR REQUIREMENTS */

class OpenCVArmorDetectorTest : public ::testing::Test
{
protected:
    OpenCVArmorDetector *detector;

    void SetUp() override
    {
        DetectorConfig config = {RED, 30, 100, 150, 1, true};
        detector = new OpenCVArmorDetector(config);
    }

    void TearDown() override
    {
        delete detector;
    }
};

////////////////////////
// START CONFIG TESTS //
////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_Config)
{
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, RED);
    EXPECT_EQ(c._hue_range_limit, 30);
    EXPECT_EQ(c._saturation_lower_limit, 100);
    EXPECT_EQ(c._value_lower_limit, 150);
    EXPECT_EQ(c._max_missed_frames, 1);
    EXPECT_EQ(c._reduce_search_area, true);
}

TEST_F(OpenCVArmorDetectorTest, test_SetConfig)
{
    std::cout << "Testing setConfig" << std::endl;
    DetectorConfig config = {BLUE, 1, 2, 3, 4, false};
    detector->setConfig(config);
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, BLUE);
    EXPECT_EQ(c._hue_range_limit, 1);
    EXPECT_EQ(c._saturation_lower_limit, 2);
    EXPECT_EQ(c._value_lower_limit, 3);
    EXPECT_EQ(c._max_missed_frames, 4);
    EXPECT_EQ(c._reduce_search_area, false);
    detector->setConfig({RED, 30, 100, 150, 1, true}); // Reset the config
}
//////////////////////
// END CONFIG TESTS //
//////////////////////

////////////////////////////////////////
// START isLightBar AND isArmor TESTS //
//    We want 100% branch coverage    //
////////////////////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_isLightBarFalses)
{
    // Width too small (LIGHT_BAR_WIDTH_LOWER_LIMIT)
    cv::RotatedRect rect1(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, 100), 0);
    EXPECT_FALSE(detector->isLightBar(rect1));

    // Height too small (LIGHT_BAR_HEIGHT_LOWER_LIMIT)
    cv::RotatedRect rect2(cv::Point2f(0, 0), cv::Size2f(100, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect2));

    // Roll angle more than LIGHT_BAR_ANGLE_LIMIT from the horizontal
    cv::RotatedRect rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), LIGHT_BAR_ANGLE_LIMIT + 1);
    cv::RotatedRect rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 180 - LIGHT_BAR_ANGLE_LIMIT - 1);
    EXPECT_FALSE(detector->isLightBar(rect3));
    EXPECT_FALSE(detector->isLightBar(rect4));

    // Aspect ratio (h/w) too low (LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT)
    cv::RotatedRect rect5(cv::Point2f(0, 0), cv::Size2f(100, (LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT * 100) - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect5));

    // MIX ALL THE ABOVE FALSES
    // Angle out of bounds and width too small
    cv::RotatedRect rect6(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, 100), 90);
    EXPECT_FALSE(detector->isLightBar(rect6));

    // Angle out of bounds and height too small
    cv::RotatedRect rect7(cv::Point2f(0, 0), cv::Size2f(100, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 150);
    EXPECT_FALSE(detector->isLightBar(rect7));

    // Width too small and height too small
    cv::RotatedRect rect8(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect8));
}

TEST_F(OpenCVArmorDetectorTest, test_isLightBarTrue)
{
    // Check some light bars which should pass (not using defined limits)
    cv::RotatedRect rect1(cv::Point2f(0, 0), cv::Size2f(40, 135), 0);
    cv::RotatedRect rect2(cv::Point2f(0, 0), cv::Size2f(40, 135), 15);
    cv::RotatedRect rect3(cv::Point2f(0, 0), cv::Size2f(40, 135), 180 - 15);
    cv::RotatedRect rect4(cv::Point2f(0, 0), cv::Size2f(40, 135), 180);
    cv::RotatedRect rect5(cv::Point2f(0, 0), cv::Size2f(40, 135), 180 + 15);

    // Generate 500 random light bars meeting the criteria
    for (int i = 0; i < 500; i++)
    {
        // width >= LIGHT_BAR_WIDTH_LOWER_LIMIT
        double width = LIGHT_BAR_WIDTH_LOWER_LIMIT + (rand() % 100);

        // height >= LIGHT_BAR_HEIGHT_LOWER_LIMIT, and height >= width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT
        // So height is whichever is bigger between LIGHT_BAR_HEIGHT_LOWER_LIMIT and width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT (plus a bit of randomness)
        double height = std::max(LIGHT_BAR_HEIGHT_LOWER_LIMIT, width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT) + static_cast<double>(rand() % 100);

        // angle in [0, LIGHT_BAR_ANGLE_LIMIT] or [180 - LIGHT_BAR_ANGLE_LIMIT, 180]
        double angle = i % 2 == 0 ? rand() % (int)LIGHT_BAR_ANGLE_LIMIT : 180 - (rand() % (int)LIGHT_BAR_ANGLE_LIMIT);

        cv::RotatedRect rect(cv::Point2f(0, 0), cv::Size2f(width, height), angle);
        EXPECT_TRUE(detector->isLightBar(rect));
    }
}

TEST_F(OpenCVArmorDetectorTest, test_isArmorFalses)
{
    // Angle difference too large (ARMOR_ANGLE_DIFF_LIMIT)
    cv::RotatedRect left_rect1(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect1(cv::Point2f(0, 0), cv::Size2f(100, 100), ARMOR_ANGLE_DIFF_LIMIT + 1);
    EXPECT_FALSE(detector->isArmor(left_rect1, right_rect1));

    // Aspect ratio ratio difference too large (ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT)
    cv::RotatedRect left_rect2(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect2(cv::Point2f(0, 0), cv::Size2f(100, 600), 0);
    EXPECT_FALSE(detector->isArmor(left_rect2, right_rect2));

    // Distance too small or too large
    cv::RotatedRect left_rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect3, right_rect3));

    // X position distance too small
    cv::RotatedRect left_rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect4, right_rect4));

    // Light bar Y position ratio too large
    cv::RotatedRect left_rect5(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect5(cv::Point2f(0, 60), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect5, right_rect5));
}

///////////////////////////////////////////////////
//       END isLightBar AND isArmor TESTS        //
// We will test the true case in the search test //
///////////////////////////////////////////////////

////////////////////////
// START search TESTS //
////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_search_no_armor)
{
    // Test the search method with a blank frame
    EXPECT_EQ(detector->getMissedFrames(), 0);
    EXPECT_EQ(detector->_detected_frame, 0);

    cv::Mat frame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    std::vector<_Float32> points = detector->search(frame);
    EXPECT_EQ(points.size(), 8);
    for (int i = 0; i < 8; i++)
    {
        EXPECT_EQ(points.at(i), 0);
    }

    EXPECT_EQ(detector->getMissedFrames(), 1);
    EXPECT_EQ(detector->_detected_frame, 0);
}

//
// This is the key test section for the armor detection. We will test the search method with frames containing armor
// We must meet a detection rate and loss threshold to pass
//

TEST_F(OpenCVArmorDetectorTest, test_search_armor_easy)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");
    OpenCVArmorDetector *detector_easy = new OpenCVArmorDetector({RED, 30, 100, 150, 1, false});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_easy->getMissedFrames(), 0);
    EXPECT_EQ(detector_easy->_detected_frame, 0);
    EXPECT_EQ(detector_easy->_frame_count, 0);

    // TEST THE EASY CASES
    std::string easy_path = (std::filesystem::path(package_share_dir) / "resources/easy").string();
    std::string gtPath = (std::filesystem::path(package_share_dir) / "resources/easy/ground_truth.csv").string();
    testImgs(easy_path, gtPath, detector_easy, MIN_DETECTION_RATE_EASY, MAX_LOSS_EASY);

    delete detector_easy;
}

TEST_F(OpenCVArmorDetectorTest, test_search_armor_blue)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");
    OpenCVArmorDetector *detector_blue = new OpenCVArmorDetector({BLUE, 30, 100, 150, 1, false});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_blue->getMissedFrames(), 0);
    EXPECT_EQ(detector_blue->_detected_frame, 0);
    EXPECT_EQ(detector_blue->_frame_count, 0);

    // TODO: The ground truth for this is a placeholder. We are just testing the detection rate
    // The assumption I make here is that if the detection of blue contours is working, the rest of the detector will work as it does for red
    std::string easy_path = (std::filesystem::path(package_share_dir) / "resources/blue").string();
    std::string gtPath = (std::filesystem::path(package_share_dir) / "resources/blue/ground_truth.csv").string();
    testImgs(easy_path, gtPath, detector_blue, MIN_DETECTION_RATE_EASY, MAX_LOSS_EASY);
}

TEST_F(OpenCVArmorDetectorTest, test_search_armor_far_back)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");
    OpenCVArmorDetector *detector_far_back = new OpenCVArmorDetector({RED, 30, 100, 150, 1, false});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_far_back->getMissedFrames(), 0);
    EXPECT_EQ(detector_far_back->_detected_frame, 0);
    EXPECT_EQ(detector_far_back->_frame_count, 0);

    std::string far_back_path = (std::filesystem::path(package_share_dir) / "resources/far_back").string();
    std::string gtPath = (std::filesystem::path(package_share_dir) / "resources/far_back/ground_truth.csv").string();
    testImgs(far_back_path, gtPath, detector_far_back, MIN_DETECTION_RATE_FAR_BACK, MAX_LOSS_FAR_BACK);

    delete detector_far_back;
}

TEST_F(OpenCVArmorDetectorTest, test_search_armor_close_video)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");
    OpenCVArmorDetector *detector_close = new OpenCVArmorDetector({RED, 30, 100, 150, 1, true});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_close->getMissedFrames(), 0);
    EXPECT_EQ(detector_close->_detected_frame, 0);
    EXPECT_EQ(detector_close->_frame_count, 0);

    std::string videoPath = (std::filesystem::path(package_share_dir) / "resources/close_video/close.avi").string();
    std::string gtPath = (std::filesystem::path(package_share_dir) / "resources/close_video/ground_truth.csv").string();
    testVideo(videoPath, gtPath, detector_close, MIN_DETECTION_RATE_CLOSE_VIDEO, MAX_LOSS_CLOSE_VIDEO);
}

// This an overall test. Contains up-close, far, spin, and hidden. So target accuracy is lower
TEST_F(OpenCVArmorDetectorTest, test_search_armor_far_back_spin_and_move_video)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");
    OpenCVArmorDetector *detector_far_back_spin_and_move = new OpenCVArmorDetector({RED, 30, 100, 150, 1, true});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_far_back_spin_and_move->getMissedFrames(), 0);
    EXPECT_EQ(detector_far_back_spin_and_move->_detected_frame, 0);
    EXPECT_EQ(detector_far_back_spin_and_move->_frame_count, 0);

    std::string videoPath = (std::filesystem::path(package_share_dir) / "resources/far_back_spin_and_move/far.avi").string();
    std::string gtPath = (std::filesystem::path(package_share_dir) / "resources/far_back_spin_and_move/ground_truth.csv").string();
    testVideo(videoPath, gtPath, detector_far_back_spin_and_move, MIN_DETECTION_RATE_FAR_BACK_SPIN_AND_MOVE, MAX_LOSS_FAR_BACK_SPIN_AND_MOVE);
}

//////////////////////
// HELPER FUNCTIONS //
//////////////////////
void testImgs(std::string folder_path, std::string gt_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix)
{
    // Read ground truths
    std::vector<std::vector<cv::Point2f>> gt;
    readGT(gt_path, gt);

    // Loss is the sum of the L2 norm between the detected points and the ground truth
    float total_loss = 0;
    int frame_idx = 0;

    // Loop through all images in the folder
    for (const auto &entry : std::filesystem::directory_iterator(folder_path))
    {
        // if not png or jpg, skip
        if (entry.path().extension() != ".png" && entry.path().extension() != ".jpg")
        {
            continue;
        }

        // Read the image and run the detector
        std::string img_path = entry.path().string();
        cv::Mat frame = cv::imread(img_path);
        cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
        std::vector<_Float32> points = detector->search(frame);

        // Loss between the detected points and the ground truth
        // Skip if no armor detected in gt or in the frame (all zeros) since we want to check loss given a detection. Detection rate is accounted for separately
        if ((gt.at(frame_idx).at(0) == cv::Point2f(0, 0) && gt.at(frame_idx).at(1) == cv::Point2f(0, 0) && gt.at(frame_idx).at(2) == cv::Point2f(0, 0) && gt.at(frame_idx).at(3) == cv::Point2f(0, 0)) || (points.at(0) == 0 && points.at(1) == 0 && points.at(2) == 0 && points.at(3) == 0 && points.at(4) == 0 && points.at(5) == 0 && points.at(6) == 0 && points.at(7) == 0))
        {
            frame_idx++;
            continue;
        }

        for (int i = 0; i < 4; i++)
        {
            cv::Point2f detected_point(points.at(i * 2), points.at(i * 2 + 1));
            cv::Point2f gt_point = gt.at(frame_idx).at(i);

            total_loss += 0.25 * cv::norm(detected_point - gt_point);
        }

        frame_idx++;
    }

    // Calculate the average loss across all frames
    double loss = total_loss / detector->_frame_count;

    // Check detection rate and loss
    double detection_rate = static_cast<double>(detector->_detected_frame) / static_cast<double>(detector->_frame_count);
    EXPECT_GE(detection_rate, min_detection_rate);
    EXPECT_LT(loss / detector->_frame_count, max_loss_pix);
}

void testVideo(std::string video_path, std::string gt_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix)
{
    // Start a stopwatch for performance testing
    auto start = std::chrono::high_resolution_clock::now();

    // Read ground truths
    std::vector<std::vector<cv::Point2f>> gt;
    readGT(gt_path, gt);

    // Open the video file
    cv::VideoCapture cap(video_path);

    // Check if the file was opened
    if (!cap.isOpened())
    {
        // fail test
        std::cerr << "Error opening video file: " << video_path << std::endl;
        FAIL();
    }

    // Read the video frame by frame
    cv::Mat frame;

    float total_loss = 0;
    int frame_idx = 0;
    while (cap.read(frame))
    {
        cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
        std::vector<_Float32> points = detector->search(frame);

        // Loss between the detected points and the ground truth
        // Skip if no armor detected in gt or in the frame (all zeros) since we want to check loss given a detection. Detection rate is accounted for separately
        if ((gt.at(frame_idx).at(0) == cv::Point2f(0, 0) && gt.at(frame_idx).at(1) == cv::Point2f(0, 0) && gt.at(frame_idx).at(2) == cv::Point2f(0, 0) && gt.at(frame_idx).at(3) == cv::Point2f(0, 0)) || (points.at(0) == 0 && points.at(1) == 0 && points.at(2) == 0 && points.at(3) == 0 && points.at(4) == 0 && points.at(5) == 0 && points.at(6) == 0 && points.at(7) == 0))
        {
            frame_idx++;
            continue;
        }

        // Loss is the sum of the L2 norm between the detected points and the ground truth
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f detected_point(points.at(i * 2), points.at(i * 2 + 1));
            cv::Point2f gt_point = gt.at(frame_idx).at(i);

            total_loss += 0.25 * cv::norm(detected_point - gt_point);
        }

        frame_idx++;
    }

    // Stop the stopwatch
    auto stop = std::chrono::high_resolution_clock::now();

    // Calculate the average loss across all frames
    double loss = total_loss / detector->_frame_count;

    // Check detection rate and loss
    double detection_rate = static_cast<double>(detector->_detected_frame) / static_cast<double>(detector->_frame_count);
    EXPECT_GE(detection_rate, min_detection_rate);
    EXPECT_LT(loss / detector->_frame_count, max_loss_pix);

    // Check the FPS
    double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
    double fps = static_cast<double>(detector->_frame_count) / (elapsed_time / 1000.0);
    EXPECT_GE(fps, MIN_FPS);
}

void readGT(std::string file_path, std::vector<std::vector<cv::Point2f>> &gt)
{
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line))
    {
        std::vector<cv::Point2f> row;
        std::stringstream ss(line);
        float x, y;

        while (ss >> x && ss.peek() == ',' && ss.ignore() && ss >> y)
        {
            row.emplace_back(x, y);
            if (ss.peek() == ',')
                ss.ignore();
        }

        gt.push_back(row);
    }
}