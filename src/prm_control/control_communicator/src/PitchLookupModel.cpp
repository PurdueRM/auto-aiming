#include "PitchLookupModel.hpp"

PitchLookupModel::PitchLookupModel() {}

PitchLookupModel::PitchLookupModel(std::string filename) {
    this->filename = filename;
    this->load_file();

    // timer to read file every 2 seconds for updates
    this->timer = std::thread([this]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            this->load_file();
        }
    });
}

/**
 * @brief Loads lookup table from file
 *
 * The first line of the file contains two space-separated integers: M, N,
 * where M represents the number of z coordinates (distance from the camera)
 * and N represents the number of y coordinates (height above the ground). 
 * The next M x N lines contain three space-separated numbers:
 * z coordinate, y coordinate, and pitch. Length measurements are in
 * millimeters, and angle measurements are in degrees.
 */
void PitchLookupModel::load_file() {
    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == nullptr) {
        return;
    }

    int y_count = 0;
    int z_count = 0;
    fscanf(fp, "%d %d\n", &z_count, &y_count);

    this->pitch_lookup_table = std::vector<std::vector<float>>(
        z_count, std::vector<float>(y_count, 0));

    for (int row = 0; row < z_count; row++) {
        for (int col = 0; col < y_count; col++) {
            int z = 0, y = 0;
            float pitch = 0;
            int read_count = fscanf(fp, "%d %d %f\n", &z, &y, &pitch);

            if (read_count != 3) {
                fclose(fp);
                fp = NULL;
                return;
            }

            if (z < this->lower_z) this->lower_z = z;
            if (z > this->upper_z) this->upper_z = z;
            if (y < this->lower_y) this->lower_y = y;
            if (y > this->upper_y) this->upper_y = y;

            this->pitch_lookup_table[row][col] = pitch;
        }
    }

    fclose(fp);
    fp = NULL;
}

/**
 * @brief Writes the lookup table to a file
 * 
 * @param filename Name of the file to write the lookup table to
 */

void PitchLookupModel::write_file() {
    FILE* fp = fopen(filename.c_str(), "w");
    if (fp == nullptr) {
        return;
    }

    int z_count = this->pitch_lookup_table.size();
    int y_count = this->pitch_lookup_table[0].size();

    fprintf(fp, "%d %d\n", z_count, y_count);

    for (int row = 0; row < z_count; row++) {
        for (int col = 0; col < y_count; col++) {
            fprintf(fp, "%d %d %f\n", row, col, this->pitch_lookup_table[row][col]);
        }
    }

    fclose(fp);
    fp = NULL;
}


/**
 * @brief Lookup the corresponding pitch with distance and height, using bilinear interpolation
 *
 * @param distance Distance away from the camera in millimeters
 * @param height   Height above the ground in millimeters
 *
 * @return Pitch in degrees
 */
float PitchLookupModel::get_offset(int distance, int height) {
    // Calculate the step size of the lookup table
    int z_step_size = (this->upper_z - this->lower_z) /
                    (this->pitch_lookup_table.size() - 1);
    int y_step_size = (this->upper_y - this->lower_y) /
                    (this->pitch_lookup_table[0].size() - 1);

    // Calculate the indices of the tightest bound containing distance and height
    int lower_step_z_idx = (distance - this->lower_z) / z_step_size;
    int upper_step_z_idx = (distance - this->lower_z + z_step_size - 1) / z_step_size;
    int lower_step_y_idx = (height - this->lower_y) / y_step_size;
    int upper_step_y_idx = (height - this->lower_y + y_step_size - 1) / y_step_size;

    // Calculate the actual values of the tightest bounds
    int lower_step_z = this->lower_z + z_step_size * lower_step_z_idx;
    int upper_step_z = this->lower_z + z_step_size * upper_step_z_idx;
    int lower_step_y = this->lower_y + y_step_size * lower_step_y_idx;
    int upper_step_y = this->lower_y + y_step_size * upper_step_y_idx;

    if (lower_step_z_idx < 0 || lower_step_z_idx >= this->pitch_lookup_table.size() ||
        upper_step_z_idx < 0 || upper_step_z_idx >= this->pitch_lookup_table.size() ||
        lower_step_y_idx < 0 || lower_step_y_idx >= this->pitch_lookup_table[0].size() ||
        upper_step_y_idx < 0 || upper_step_y_idx >= this->pitch_lookup_table[0].size()) {
        return 0; // No computed pitch offset
    }

    // Calculate the pitch of the four points with tightest bounds
    float pitch_low_low = this->pitch_lookup_table[lower_step_z_idx][lower_step_y_idx];
    float pitch_low_high = this->pitch_lookup_table[lower_step_z_idx][upper_step_y_idx];
    float pitch_high_low = this->pitch_lookup_table[upper_step_z_idx][lower_step_y_idx];
    float pitch_high_high = this->pitch_lookup_table[upper_step_z_idx][upper_step_y_idx];

    // Calculate the pitch using bilinear interpolation
    float pitch_low = (pitch_low_low * (upper_step_y - height) +
                       pitch_low_high * (height - lower_step_y)) /
                      (upper_step_y - lower_step_y);
    float pitch_high = (pitch_high_low * (upper_step_y - height) +
                        pitch_high_high * (height - lower_step_y)) /
                       (upper_step_y - lower_step_y);
    float pitch = (pitch_low * (upper_step_z - distance) +
                   pitch_high * (distance - lower_step_z)) /
                  (upper_step_z - lower_step_z);

    return pitch;
}

/**
 * @brief Maps a value from a range to another range
 *
 * @param value   Value to be mapped
 * @param old_min The minimum of the old range
 * @param old_may The maximum of the old range
 * @param new_min The minimum of the new range
 * @param new_may The maximum of the new range
 *
 * @return Mapped value
 */
float PitchLookupModel::map(float value, float old_min, float old_may, float new_min, float new_may) 
{
    if (old_min > old_may || new_min > new_may) {
        return -1;
    }

    if (old_min == old_may) {
        return (new_min == new_may) ? new_min : -1;
    }

    return new_min + (value - old_min) * (new_may - new_min) / (old_may - old_min);
}

void PitchLookupModel::print_to_file(std::string text) {
    FILE* fp = fopen("/home/purduerm/Documents/output.txt", "a");
    if (fp == nullptr) {
        return;
    }

    fprintf(fp, "%s\n", text.c_str());
    fclose(fp);
}

