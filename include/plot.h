#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>


namespace fs = boost::filesystem;


class DataLogger {
private:
    std::ofstream file_;
    bool is_file_open_;
    double start_time_;

public:
    DataLogger(const std::string& filename) : is_file_open_(false), start_time_(0.0) 
    {

        fs::path full_path(filename);

         // 提取文件夹路径
        fs::path dir_path = full_path.parent_path();

        // 创建目录（如果不存在）
        if (!fs::exists(dir_path)) {
            if (fs::create_directories(dir_path)) {
                std::cout << "Created directory: " << dir_path << std::endl;
            } else {
                std::cerr << "Failed to create directory: " << dir_path << std::endl;
                return;
            }
        }

        // 打开文件，追加模式
        // file_.open(filename, std::ios::app);
        file_.open(filename, std::ios::trunc);
        if (!file_.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        is_file_open_ = true;      
        
    }

    ~DataLogger() {
        if (is_file_open_) {
            file_.close();
        }
    }

    void logData(double time, const Eigen::VectorXd& data)
    {
        if (!is_file_open_) return;
        // 写入标记行：运行编号或启动时间
        if(start_time_ == 0.0)
        {
            start_time_ = time;
            file_ << "# Run start: " << std::fixed << std::endl;
        }
        // 记录时间戳和数据
        file_ << std::fixed << std::setprecision(6)
              << time-start_time_;

        for (int i = 0; i < data.size(); ++i) 
        {
            file_ << "," << data(i);
        }

        file_ << std::endl;
        
    }
};
