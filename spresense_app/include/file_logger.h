#ifndef LOG_WRITER_HPP
#define LOG_WRITER_HPP

#include <string>
#include <fstream>

constexpr const char* SD_ROOTPATH = "/mnt/sd0";

extern std::ofstream imu_file;
extern std::ofstream uwb_file;

bool init_log_files();
bool close_log_files();

#endif // LOG_WRITER_HPP
