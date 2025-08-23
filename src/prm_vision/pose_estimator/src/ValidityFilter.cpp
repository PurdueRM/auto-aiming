#include "ValidityFilter.hpp"

ValidityFilter::ValidityFilter(int lock_in_after, float max_distance, float min_distance, float max_shift_distance, int prev_len)
{
    this->_lock_in_after = lock_in_after;
    this->_max_distance = max_distance;
    this->_min_distance = min_distance;
    this->_max_shift_distance = max_shift_distance;
    this->_prev_len = std::min(prev_len, 20);

    // Initialize arrays to track previous predictions
    for (int i = 0; i < this->_prev_len; i++)
    {
        _prev_x[i] = 0;
        _prev_y[i] = 0;
        _prev_z[i] = 0;
    }
}

ValidityFilter::~ValidityFilter() {}

/**
 * @brief Check if we should reset kalman filter based on a validity filter.
 *
 * @param x The x-coordinate of the detected armor.
 * @param y The y-coordinate of the detected armor.
 * @param z The z-coordinate of the detected armor.
 * @return true If the pose is valid based on the validity filter and current aiming state.
 *
 * NOTE: The return value is ONLY used to determine if we should reset the Kalman filter inside PoseEstimator.
 * We primarily use the state variable to determine the action to take (idle, track, stop).
 */
bool ValidityFilter::shouldResetKalman(float x, float y, float z)
{
    // Calculate time difference since last valid detection
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double, std::milli>(now - _last_valid_time).count();

    // Check distance validity
    if (!distanceValidity(x, y, z))
    {
        decrementLockInCounter();
        updatePrev(x, y, z);      // We still add to prior detections, this is a design choice
        return state != STOPPING; // not in range, so invalid if we aren't stopping
    }

    // Check time since last valid detection
    if (dt > _max_dt)
    {
        resetLockInCounter();
        _last_valid_time = now; // Reset the timer even after a timeout, prevents this check from always failing if we fail after succeeding for _max_dt
        updatePrev(x, y, z);
        return state != STOPPING; // too much time has passed, so invalid if we aren't stopping
    }

    // Check position validity
    if (!positionValidity(x, y, z))
    {
        decrementLockInCounter();
        updatePrev(x, y, z);
        return state != STOPPING; // too much shift, so invalid if we aren't stopping
    }

    // Valid detection
    incrementLockInCounter();
    updatePrev(x, y, z);
    _last_valid_time = now; // Update the last valid time

    return false;
}

void ValidityFilter::updatePrev(float x, float y, float z)
{
    _prev_x[_prev_idx] = x;
    _prev_y[_prev_idx] = y;
    _prev_z[_prev_idx] = z;
    _prev_idx = (_prev_idx + 1) % _prev_len;
}

bool ValidityFilter::distanceValidity(float x, float y, float z)
{
    float dst = std::sqrt(x * x + y * y + z * z);
    return (dst <= _max_distance && dst >= _min_distance);
}

bool ValidityFilter::positionValidity(float x, float y, float z)
{
    for (int i = 0; i < _prev_len; i++)
    {
        float dx = x - _prev_x[i];
        float dy = y - _prev_y[i];
        float dz = z - _prev_z[i];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < _max_shift_distance)
        {
            return true; // Valid as soon as one match is found
        }
    }
    return false; // No valid matches found
}

void ValidityFilter::incrementLockInCounter()
{
    if (++_lock_in_counter >= _lock_in_after)
    {
        state = TRACKING;
        _lock_in_counter = _lock_in_after; // Cap at max
    }
    else
    {
        state = IDLING;
    }
}

void ValidityFilter::decrementLockInCounter()
{
    if (--_lock_in_counter <= 0)
    {
        state = STOPPING;
        _lock_in_counter = 0; // Ensure no negative values
    }
    else
    {
        state = IDLING;
    }
}

void ValidityFilter::resetLockInCounter()
{
    _lock_in_counter = 0;
    state = STOPPING;
}

int ValidityFilter::getLockInCounter()
{
    return _lock_in_counter;
}
