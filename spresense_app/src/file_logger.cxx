#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <iostream>

#include "include/file_logger.h"

std::ofstream imu_file;
std::ofstream uwb_file;

static int get_next_log_dir_number(const std::string& base_path) {
    DIR* dp = opendir(base_path.c_str());
    if (!dp) return 1;

    int max_index = 0;
    struct dirent* entry;
    while ((entry = readdir(dp)) != nullptr) {
        if (entry->d_type == DT_DIR) {
            std::string name(entry->d_name);
            if (name.size() == 3 && std::all_of(name.begin(), name.end(), ::isdigit)) {
                int index = std::stoi(name);
                if (index > max_index) max_index = index;
            }
        }
    }
    closedir(dp);
    return max_index + 1;
}

static std::string create_log_directory(const std::string& base_path) {
    int dir_number = get_next_log_dir_number(base_path);
    std::ostringstream oss;
    oss << base_path << '/' << std::setfill('0') << std::setw(3) << dir_number;
    std::string new_dir = oss.str();

    if (mkdir(new_dir.c_str(), 0777) != 0) {
        std::cerr << "mkdir failed: " << new_dir << std::endl;
        return "";
    }
    return new_dir;
}

bool init_log_files() {
    std::string log_dir = create_log_directory(SD_ROOTPATH);
    if (log_dir.empty()) return false;

    imu_file.open(log_dir + "/imu.txt");
    uwb_file.open(log_dir + "/uwb.txt");

    if (!imu_file.is_open() || !uwb_file.is_open()) {
        std::cerr << "file open failed\n";
        return false;
    }

    imu_file << "timestamp[us],temp[celsius],ax[m/s^2],ay[m/s^2],az[m/s^2],gx[rad/s],gy[rad/s],gz[rad/s]" << std::endl;
    uwb_file << "timestamp[us],anchor_id,nlos,distance[m],azimuth[deg],elevation[deg]" << std::endl;

    return true;
}

bool close_log_files() {
    if (imu_file.is_open()) {
        imu_file.close();
    }
    if (uwb_file.is_open()) {
        uwb_file.close();
    }
    return true;
}