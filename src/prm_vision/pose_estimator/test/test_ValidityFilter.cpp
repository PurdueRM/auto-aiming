#include "ValidityFilter.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>

class ValidityFilterTest : public ::testing::Test
{
protected:
    ValidityFilter *filter;

    void SetUp() override
    {
        // Explicitly set the parameters, in case a default param changes
        filter = new ValidityFilter(3, 10000, 10, 150, 5);
    }

    void TearDown() override
    {
        delete filter;
    }
};

TEST_F(ValidityFilterTest, test_updatePrev)
{
    ValidityFilter *filter_temp = new ValidityFilter(3, 10000, 10, 150, 5);

    // write 6 values to the filter, should wrap around
    for (int i = 0; i < 6; i++)
    {
        filter_temp->updatePrev(i, i, i);
    }

    // check the values, first should be overwritten as 5
    // 0 -> 01 -> 012 -> 0123 -> 01234 -> 51234
    float *prev_x = filter_temp->getPrevX();
    float *prev_y = filter_temp->getPrevY();
    float *prev_z = filter_temp->getPrevZ();

    EXPECT_EQ(prev_x[0], 5);
    EXPECT_EQ(prev_y[0], 5);
    EXPECT_EQ(prev_z[0], 5);

    for (int i = 1; i < 5; i++)
    {
        EXPECT_EQ(prev_x[i], i);
        EXPECT_EQ(prev_y[i], i);
        EXPECT_EQ(prev_z[i], i);
    }
}

TEST_F(ValidityFilterTest, test_distanceValidity)
{
    // Valid values
    EXPECT_TRUE(filter->distanceValidity(10, 0, 0));
    EXPECT_TRUE(filter->distanceValidity(0, 10, 0));
    EXPECT_TRUE(filter->distanceValidity(0, 0, 10));
    EXPECT_TRUE(filter->distanceValidity(10, 10, 10));
    EXPECT_TRUE(filter->distanceValidity(10000, 0, 0));
    EXPECT_TRUE(filter->distanceValidity(0, 10000, 0));
    EXPECT_TRUE(filter->distanceValidity(0, 0, 10000));

    // Right at the edge
    // dist = sqrt(x^2 + x^2 + x^2) -> x = sqrt(dist^2 / 3)
    double lower = std::sqrt(pow(10000, 2) / 3);
    EXPECT_TRUE(filter->distanceValidity(lower, lower, lower));

    // Too high values
    EXPECT_FALSE(filter->distanceValidity(10001, 0, 0));
    EXPECT_FALSE(filter->distanceValidity(0, 10001, 0));
    EXPECT_FALSE(filter->distanceValidity(0, 0, 10001));
    EXPECT_FALSE(filter->distanceValidity(10001, 10001, 10001));
    EXPECT_FALSE(filter->distanceValidity(lower + 0.01, lower, lower));

    // Too low values
    EXPECT_FALSE(filter->distanceValidity(0, 0, 0));
    EXPECT_FALSE(filter->distanceValidity(0, 0, 9.99));
    EXPECT_FALSE(filter->distanceValidity(0, 9, 0.5));
}

TEST_F(ValidityFilterTest, test_positionValidity)
{
    // Valid values
    filter->updatePrev(0, 0, 0);
    EXPECT_TRUE(filter->positionValidity(0, 0, 0));
    EXPECT_TRUE(filter->positionValidity(0, 149.0, 0.5));
    EXPECT_TRUE(filter->positionValidity(0, 0, 0));
    EXPECT_TRUE(filter->positionValidity(0, 0, 0));
    EXPECT_TRUE(filter->positionValidity(0, 0, 0));

    // Too high values
    EXPECT_FALSE(filter->positionValidity(150.1, 0, 0));
    EXPECT_FALSE(filter->positionValidity(0, 150.1, 0));
    EXPECT_FALSE(filter->positionValidity(0, 0, 150.1));
    EXPECT_FALSE(filter->positionValidity(150.1, 150.1, 150.1));

    // Initially invalid, then valid after a "detection" with values closer
    EXPECT_FALSE(filter->positionValidity(150.1, 0, 0));
    filter->updatePrev(100, 0, 0);
    EXPECT_TRUE(filter->positionValidity(150.1, 0, 0));

    // Initially invalid, then still invalid, then valid after a "detection" with values closer
    EXPECT_FALSE(filter->positionValidity(450.1, 0, 0));
    filter->updatePrev(100, 0, 0);
    EXPECT_FALSE(filter->positionValidity(450.1, 0, 0));
    filter->updatePrev(400, 0, 0);
    EXPECT_TRUE(filter->positionValidity(450.1, 0, 0));
}

TEST_F(ValidityFilterTest, test_shouldResetKalman_states)
{
    ValidityFilter *filter_temp = new ValidityFilter(3, 10000, 10, 150, 5);

    // Initial detection will always be INVALID since we have no prior detections for positionValidity, and dt is too high
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    EXPECT_EQ(filter_temp->getPrevX()[0], 0);
    EXPECT_EQ(filter_temp->getPrevY()[0], 0);
    EXPECT_EQ(filter_temp->getPrevZ()[0], 0);
    EXPECT_FALSE(filter_temp->shouldResetKalman(100, 200, 100)); // But do not reset KF since we are in STOPPING
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    // However we still cache the values
    EXPECT_EQ(filter_temp->getPrevX()[0], 100);
    EXPECT_EQ(filter_temp->getPrevY()[0], 200);
    EXPECT_EQ(filter_temp->getPrevZ()[0], 100);
    // Now let's get another detection, this time it should be valid and put us in IDLING
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 1);
    EXPECT_EQ(filter_temp->getPrevX()[1], 123);
    EXPECT_EQ(filter_temp->getPrevY()[1], 210);
    EXPECT_EQ(filter_temp->getPrevZ()[1], 123);
    // Another valid detection, still in IDLING
    EXPECT_FALSE(filter_temp->shouldResetKalman(124, 211, 124));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
    EXPECT_EQ(filter_temp->getPrevX()[2], 124);
    EXPECT_EQ(filter_temp->getPrevY()[2], 211);
    EXPECT_EQ(filter_temp->getPrevZ()[2], 124);
    // Let's do an invalid detection, we should still be in IDLING but counter should decrement
    EXPECT_TRUE(filter_temp->shouldResetKalman(999, 999, 999)); // Reset KF since we are too far
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 1);
    EXPECT_EQ(filter_temp->getPrevX()[3], 999);
    EXPECT_EQ(filter_temp->getPrevY()[3], 999);
    EXPECT_EQ(filter_temp->getPrevZ()[3], 999);
    // Another invalid detection, we will go to STOPPING since lock in counter decrements to 0
    EXPECT_FALSE(filter_temp->shouldResetKalman(1999, 1999, 1999)); // This puts us in STOPPING, so we actually don't reset KF
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    EXPECT_FALSE(filter_temp->shouldResetKalman(2999, 2999, 2999)); // This is invalid, but we are already in STOPPING
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    // Should advance to IDLING after a valid detection
    EXPECT_FALSE(filter_temp->shouldResetKalman(125, 212, 125));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 1);
    // Should advance to TRACKING after 2 more valid detections
    EXPECT_FALSE(filter_temp->shouldResetKalman(126, 213, 126));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
    EXPECT_FALSE(filter_temp->shouldResetKalman(127, 214, 127));
    EXPECT_EQ(filter_temp->state, TRACKING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 3);
    // Should stay in TRACKING after 3 more valid detections
    EXPECT_FALSE(filter_temp->shouldResetKalman(128, 215, 128));
    EXPECT_EQ(filter_temp->state, TRACKING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 3);
    EXPECT_FALSE(filter_temp->shouldResetKalman(129, 216, 129));
    EXPECT_EQ(filter_temp->state, TRACKING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 3);
    EXPECT_FALSE(filter_temp->shouldResetKalman(130, 217, 130));
    EXPECT_EQ(filter_temp->state, TRACKING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 3);
}

TEST_F(ValidityFilterTest, test_shouldResetKalman_failures)
{
    ValidityFilter *filter_temp = new ValidityFilter(3, 10000, 10, 150, 5);
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    EXPECT_EQ(filter_temp->getPrevX()[0], 0);
    EXPECT_EQ(filter_temp->getPrevY()[0], 0);
    EXPECT_EQ(filter_temp->getPrevZ()[0], 0);

    // Two valid to get two into IDLING, then one to fail due to distance
    EXPECT_FALSE(filter_temp->shouldResetKalman(100, 200, 100));
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
    EXPECT_TRUE(filter_temp->shouldResetKalman(12400, 21100, 12400)); // Not in stopping, so we reset KF
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 1);
    EXPECT_FALSE(filter_temp->shouldResetKalman(22400, 21100, 12400)); // In stopping, so we don't reset KF
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);

    // From here, two valids to get two into IDLING, then one to fail due to dt
    EXPECT_FALSE(filter_temp->shouldResetKalman(100, 200, 100));
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
    // sleep for _max_dt and then invalid. Will reset lock in counter
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(filter_temp->_max_dt) + 500));
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123)); // We reset lock in counter, so ALWAYS regress to STOPPING
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);

    // From here, two valids to get two into IDLING, then one to fail due to shift
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
    // Invalid due to shift, should reset KF
    EXPECT_TRUE(filter_temp->shouldResetKalman(400, 400, 400)); // Not in stopping, so we reset KF
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 1);
    EXPECT_FALSE(filter_temp->shouldResetKalman(600, 400, 400)); // Not in stopping, so we reset KF
    EXPECT_EQ(filter_temp->state, STOPPING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 0);
    // Back to tracking and we call it a day
    EXPECT_FALSE(filter_temp->shouldResetKalman(100, 200, 100));
    EXPECT_FALSE(filter_temp->shouldResetKalman(123, 210, 123));
    EXPECT_EQ(filter_temp->state, IDLING);
    EXPECT_EQ(filter_temp->getLockInCounter(), 2);
}