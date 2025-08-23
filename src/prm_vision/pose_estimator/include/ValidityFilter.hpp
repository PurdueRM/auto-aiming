#ifndef _RESULT_FILTER_HPP_
#define _RESULT_FILTER_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>
#include <stdio.h>

enum ValidityFilterState
{
    TRACKING,
    IDLING,
    STOPPING,
};

class ValidityFilter
{
public:
    ValidityFilter(int lock_in_after = 3, float max_distance = 10000, float min_distance = 10, float max_shift_distance = 150, int prev_len = 5);

    ~ValidityFilter();

    bool shouldResetKalman(float x, float y, float z);
    bool positionValidity(float, float, float);
    bool distanceValidity(float, float, float);
    void updatePrev(float, float, float);
    int getLockInCounter();

    ValidityFilterState state = STOPPING;
    double _max_dt = 2000; // ms

    // Getters
    float *getPrevX() { return _prev_x; }
    float *getPrevY() { return _prev_y; }
    float *getPrevZ() { return _prev_z; }
    int getLockInAfter() { return _lock_in_after; }

    // Setters
    void setLockInAfter(int lock_in_after) { _lock_in_after = lock_in_after; }
    void setMaxDistance(float max_distance) { _max_distance = max_distance; }
    void setMinDistance(float min_distance) { _min_distance = min_distance; }
    void setMaxShiftDistance(float max_shift_distance) { _max_shift_distance = max_shift_distance; }
    void setPrevLen(int prev_len) { _prev_len = prev_len; }
    void setMaxDt(double max_dt) { _max_dt = max_dt; }

protected:
    int _lock_in_after;        // lock in after n frames
    float _max_distance;       // mm
    float _min_distance;       // mm
    float _max_shift_distance; // mm
    int _prev_len;             // check the back n frames for max shift distance vilolation

    int _lock_in_counter = 0;

    // zero time point
    std::chrono::steady_clock::time_point _last_valid_time = std::chrono::steady_clock::time_point::min();

    float _prev_x[20];
    float _prev_y[20];
    float _prev_z[20];
    int _prev_idx = 0;

    void incrementLockInCounter();
    void decrementLockInCounter();
    void resetLockInCounter();
};

#endif // _RESULT_FILTER_HPP_