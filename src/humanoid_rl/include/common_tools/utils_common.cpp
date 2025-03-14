/**
 * Copyright (c) [2025] XinChengYang <vertax@foxmail.com> <yaphetys@gmail.com>
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "utils_common.h"
#include "log4z.h"

namespace common_tools {
// system related tools
void dump_program_info_log4z(const std::string &app_name)
{
    const time_t t = time(NULL);
    const struct tm *tmp = localtime(&t);
    char szTime[256];
    char szDate[256];
    strftime(szTime, 80, "%H:%M:%S", tmp);
    strftime(szDate, 80, "%Y/%m/%d", tmp);

    // zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    std::string cpu_info = common_tools::get_cpu_info();
    std::string ram_info = common_tools::get_RAM_info();
    std::string os_info = common_tools::get_OS_info();
    std::string home_folder = common_tools::get_home_folder();
    std::string current_folder = common_tools::get_current_folder();
    std::string current_date_time = std::string(szDate) + " " + std::string(szTime);

    LOGD("=============================================================");
    LOGFMTD("App name   : %s", app_name.c_str());
    LOGFMTD("Build date : %s  %s", __DATE__, __TIME__);
    LOGFMTD("CPU infos  : %s", cpu_info.c_str());
    LOGFMTD("RAM infos  : %s", ram_info.c_str());
    LOGFMTD("OS  infos  : %s", os_info.c_str());
    LOGFMTD("Home dir   : %s", home_folder.c_str());
    LOGFMTD("Current dir: %s", current_folder.c_str());
    LOGFMTD("Date mow   : %s", current_date_time.c_str());
    LOGD("=============================================================");

#ifdef HUMANOID_RL_VERSION_STRING // Has Humanoid_rl
    LOGFMTD("Humanoid_rl version    : %d.%d.%d.%d", HUMANOID_RL_MAJOR, HUMANOID_RL_MINOR, HUMANOID_RL_PATCH,
            HUMANOID_RL_BUILD);
#endif

#ifdef __GNUC_PATCHLEVEL__
    LOGFMTD("GCC version            : %d.%d.%d", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#endif

#ifdef QT_VERSION // Has Qt
    LOGFMTD("Qt version             : %d.%d.%d", (QT_VERSION >> 16), ((QT_VERSION >> 8) & 0xFF), (QT_VERSION & 0xFF));
#endif

#ifdef BOOST_VERSION // Has Boost
    LOGFMTD("Boost version          : %d.%d.%d", BOOST_VERSION / 100000, BOOST_VERSION / 100 % 1000,
            BOOST_VERSION % 100);
#endif

#ifdef EIGEN_MINOR_VERSION // Has Eigen
    LOGFMTD("Eigen version          : %d.%d.%d", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
#endif

#ifdef VTK_MAJOR_VERSION
    LOGFMTD("VTK version            : %d.%d.%d", VTK_MAJOR_VERSION, VTK_MINOR_VERSION, VTK_BUILD_VERSION);
#endif

#ifdef PCL_VERSION // Has PCL
    LOGFMTD("PCL version            : %d.%d.%d", PCL_MAJOR_VERSION, PCL_MINOR_VERSION, PCL_REVISION_VERSION);
#endif

#ifdef CV_VERSION_REVISION // Has OpenCV
    LOGFMTD("OpenCV version         : %d.%d.%d", CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_VERSION_REVISION);
#endif

#ifdef CERES_VERSION_PATCH // Has Ceres-solver
    LOGFMTD("Ceres-solver version   : %d.%d.%d", CERES_VERSION_MAJOR, CERES_VERSION_MINOR, CERES_VERSION_PATCH);
#endif

#ifdef CGAL_BUGFIX_VERSION // Has CGAL
    LOGFMTD("CGAL version           : %d.%d.%d", CGAL_MAJOR_VERSION, CGAL_MINOR_VERSION, CGAL_BUGFIX_VERSION);
#endif

#ifdef CGAL_VERSION // Has CGAL
    LOGFMTD("CGAL version           : %s", CGAL_VERSION);
#endif

    LOGD("=============================================================");

    // zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

void clean_log_files(int max_log_count)
{
    // 日志文件目录
    boost::filesystem::path log_dir("./log4z");

    // 检查日志文件目录是否存在
    if(!boost::filesystem::exists(log_dir) || !boost::filesystem::is_directory(log_dir))
    {
        LOGFMTE("Error: Log directory does not exist or is not a directory.\n");
        return;
    }

    // 存储文件路径和修改时间的pair
    std::vector<std::pair<boost::filesystem::path, std::time_t> > log_files;

    // 遍历日志目录，获取所有日志文件
    for(boost::filesystem::directory_iterator itr(log_dir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if(boost::filesystem::is_regular_file(itr->status()))
        {
            // 获取文件修改时间
            std::time_t mod_time = boost::filesystem::last_write_time(itr->path());
            log_files.push_back(std::make_pair(itr->path(), mod_time));
        }
    }

    // 如果日志文件数量小于最大数量，则不需要删除
    if(static_cast<int>(log_files.size()) < max_log_count)
    {
        LOGFMTI("No need to delete logs. Current log count: %lu\n", log_files.size());
        return;
    }

    // 按文件的修改时间排序，最旧的文件排在前面
    std::sort(log_files.begin(), log_files.end(), [](const auto &a, const auto &b) {
        return a.second < b.second; // 比较时间，最旧的排在前面
    });

    // 需要删除的文件数量
    size_t files_to_delete = log_files.size() - max_log_count;

    // 删除最老的文件
    for(size_t i = 0; i < files_to_delete; ++i)
    {
        try
        {
            boost::filesystem::remove(log_files[i].first); // 删除文件
            LOGFMTI("Deleted old log file: %s\n", log_files[i].first.string().c_str());
        } catch(const boost::filesystem::filesystem_error &e)
        {
            LOGFMTE("Error deleting file: %s\n", e.what());
        }
    }
}

// boost::filesystem related tools
std::string get_currentpath(const std::string &path)
{
    try
    {
        boost::filesystem::path boost_path(path);
        if(boost::filesystem::exists(boost_path))
        {
            return boost_path.parent_path().string();
        } else
        {
            LOGFMTE("Error: Image path %s does not exist", path.c_str());
            return nullptr;
        }
    } catch(const boost::filesystem::filesystem_error &e)
    {
        // std::cerr << e.what() << std::endl;
        LOGFMTE("Error: Could not load image at %s", path.c_str());
        return nullptr;
    }
}

std::string get_filename(const std::string &path)
{
    try
    {
        boost::filesystem::path boost_path(path);
        if(boost::filesystem::exists(boost_path))
        {
            return boost_path.stem().string();
        } else
        {
            LOGFMTE("Error: Image path %s does not exist", path.c_str());
            return nullptr;
        }
    } catch(const boost::filesystem::filesystem_error &e)
    {
        LOGFMTE("Error: Could not load image at %s", path.c_str());
        // std::cerr << e.what() << std::endl;
        return nullptr;
    }
}

long getFileSize(const std::string &filename)
{
    try
    {
        boost::filesystem::path boost_path(filename);
        if(boost::filesystem::exists(boost_path))
        {
            return boost::filesystem::file_size(boost_path); // return in bytes
        } else
        {
            LOGFMTE("Error: File %s does not exist", filename.c_str());
            return -1;
        }
    } catch(const boost::filesystem::filesystem_error &e)
    {
        LOGFMTE("Error: Could not read filesize at %s", filename.c_str());
        // std::cerr << e.what() << std::endl;
        return -1;
    }
}
// Eigen-related tools
std::string eigenMatrixToString(const Eigen::Matrix4f &mat)
{
    std::ostringstream oss;
    for(int i = 0; i < mat.rows(); ++i)
    {
        for(int j = 0; j < mat.cols(); ++j)
        {
            oss << mat(i, j);
            if(j < mat.cols() - 1)
            {
                oss << ", ";
            }
        }
        if(i < mat.rows() - 1)
        {
            oss << "\n";
        }
    }
    return oss.str();
}
} // namespace common_tools
