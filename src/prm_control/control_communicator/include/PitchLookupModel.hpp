#ifndef PITCH_LOOKUP_MODEL_HPP
#define PITCH_LOOKUP_MODEL_HPP

#include <climits>
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <string>
#include <thread>

class PitchLookupModel {
public:
    PitchLookupModel();
    explicit PitchLookupModel(std::string filename);

    void load_file();
    void write_file();
    float get_offset(int distance, int height);

private:
    int lower_y = INT_MAX;
    int lower_z = INT_MAX;
    int upper_y = INT_MIN;
    int upper_z = INT_MIN;
    std::string filename;
    std::vector<std::vector<float>> pitch_lookup_table;
    std::thread timer;

    float map(float value, float old_min, float old_max, float new_min, float new_max);
    void print_to_file(std::string text);
};

#endif // PITCH_LOOKUP_MODEL_HPP