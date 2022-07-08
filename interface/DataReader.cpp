#include "DataReader.h"

namespace interface
{
std::vector<sensorData::PoseData> GetTUMPoseFromFile(std::ifstream &pose_stream, int drop_lines_num)
{
    std::string line;

    for (int i = 0; i < drop_lines_num; i++)
        getline(pose_stream, line); // drop this line
    std::vector<sensorData::PoseData> pose_data;
    while (!pose_stream.eof())
    {
        std::string s;
        std::getline(pose_stream, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            //字符串格式:  时间戳 x y z qx qy qz qw
            sensorData::PoseData poseData;
            ss >> poseData.timestamp;
            ss >> poseData.posi[0];
            ss >> poseData.posi[1];
            ss >> poseData.posi[2];

            ss >> poseData.rot.x();
            ss >> poseData.rot.y();
            ss >> poseData.rot.z();
            ss >> poseData.rot.w();
            pose_data.push_back(poseData);
        }
    }

    return pose_data;
}

std::vector<std::string> ReadFolder(const std::string &folder)
{
    std::vector<std::string> files;
    auto dir = opendir(folder.c_str());

    if ((dir) != nullptr)
    {
        struct dirent *entry;
        entry = readdir(dir);
        while (entry)
        {
            auto temp = folder + "/" + entry->d_name;
            if (std::strcmp(entry->d_name, "") == 0 ||
                std::strcmp(entry->d_name, ".") == 0 ||
                std::strcmp(entry->d_name, "..") == 0)
            {
                entry = readdir(dir);
                continue;
            }
            files.push_back(temp);
            entry = readdir(dir);
        }
    }
    return files;
}

void LoadExtrinsic(
    const std::string &file_path, Eigen::Affine3d &extrinsic)
{
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["transform"])
    {
        if (config["transform"]["translation"])
        {
            extrinsic.translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic.translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic.translation()(2) =
                config["transform"]["translation"]["z"].as<double>();
            if (config["transform"]["rotation"])
            {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic.linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
                // AINFO << Eigen::Quaterniond(qw, qx, qy, qz).coeffs();
            }
        }
    }
}

bool LoadIntrinsic(const std::string &intrinsics_path, cv::Mat &dist_coeffs, cv::Mat &intrisic_mat)
{
#if GCC_VERSION >= 90400
    if (!(std::filesystem::exists(intrinsics_path)))
        return false;
#else
    if (!(boost::filesystem::exists(intrinsics_path)))
        return false;
#endif
    YAML::Node config = YAML::LoadFile(intrinsics_path);

    if (config["K"] && config["D"])
    {
        std::vector<double> K = config["K"].as<std::vector<double>>();
        std::vector<double> D = config["D"].as<std::vector<double>>();
        intrisic_mat = cv::Mat(3, 3, cv::DataType<double>::type);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                intrisic_mat.at<double>(i, j) = K[i * 3 + j];
            }
        }
        dist_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs.at<double>(i) = D[i];
        }
    }

    return true;
}

bool CopyFiles(const std::string &src, const std::string &dst, int mode)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
    std::error_code ec;
#else
    namespace fs = boost::filesystem;
    boost::system::error_code ec;
#endif
    const fs::path dst_path(dst);
    const fs::path src_path(src);

    // 待拷贝文件为普通文件，则直接拷贝
    if (fs::is_regular_file(src_path)){
        // case1: src is a file, dst is also a file or not exist
        if (fs::is_regular_file(dst_path) || !fs::exists(dst_path)){
            if (mode == 0){
                fs::copy_file(src_path, dst_path, ec);
            }else{
                fs::rename(src_path, dst_path, ec);
            }

            if (ec)
            {
                AERROR << "Copy File Failed: " << ec.message();
                return false;
            }
        }
        // case2: src is a file, dst is a directory
        else if (fs::is_directory(dst_path)){
            fs::path dst_file = dst_path / src_path.filename();
            if (mode == 0){
                fs::copy_file(src_path, dst_path, ec);
            }else{
                fs::rename(src_path, dst_path, ec);
            }
            if (ec)
            {
                AERROR << "Copy File Failed: " << ec.message();
                return false;
            }
        }
        // case3: src is a file, dst is neither file nor directory
        else{
            if (ec)
            {
                AERROR << "Copy File Failed: " << ec.message();
                return false;
            }
        }
        return true;
    }

    // 待拷贝文件为目录，则递归拷贝
    if (fs::is_directory(src_path)){
        if (!fs::exists(dst))
        {
            fs::create_directories(dst);
        }
        for (fs::directory_iterator it(src); it != fs::directory_iterator(); ++it)
        {
            const fs::path newSrc = src / it->path();
            const fs::path newDst = dst / it->path();
            if (fs::is_directory(newSrc))
            {
                CopyFiles(newSrc.string(), newDst.string());
            }
            else if (fs::is_regular_file(newSrc))
            {
#if GCC_VERSION >= 90400
                if (mode == 0){
                    fs::copy_file(newSrc, newDst, fs::copy_options::overwrite_existing, ec);
                }else{
                    fs::rename(newSrc, newDst, ec);
                }

#else
                if (mode == 0){
                    fs::copy_file(newSrc, newDst, fs::copy_option::overwrite_if_exists, ec);
                }else{
                    fs::rename(newSrc, newDst, ec);
                }
#endif
                if (ec)
                {
                    AERROR << "Copy File Failed: " << ec.message();
                    return false;
                }
            }
        }
        return true;
    }

    AERROR << "Error: unrecognized file." << src;
    return false;
}

bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
    std::error_code ec;
#else
    namespace fs = boost::filesystem;
    boost::system::error_code ec;
#endif
    // 设置遍历结束标志，用recursive_directory_iterator即可循环的遍历目录
    fs::recursive_directory_iterator end;

    for (fs::recursive_directory_iterator pos(strSourceDir); pos != end; ++pos)
    {
        //过滤掉目录和子目录为空的情况
        if (fs::is_directory(*pos))
            continue;
        std::string strAppPath = fs::path(*pos).string();
        std::string strRestorePath;
        // replace_first_copy: 在strAppPath中查找strSourceDir字符串
        // 找到则用strDestDir替换, 替换后的字符串保存在一个输出迭代器中
        boost::algorithm::replace_first_copy(std::back_inserter(strRestorePath), strAppPath, strSourceDir, strDestDir);
        if (!fs::exists(fs::path(strRestorePath).parent_path()))
        {
            fs::create_directories(fs::path(strRestorePath).parent_path(), ec);
        }
#if GCC_VERSION >= 90400
        fs::copy_file(strAppPath, strRestorePath, fs::copy_options::overwrite_existing, ec);
#else
        fs::copy_file(strAppPath, strRestorePath, fs::copy_option::overwrite_if_exists, ec);
#endif
    }
    if (ec)
        return false;

    return true;
}

bool GetFileNameInPath(const std::string &path, std::string &filename){
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
#else
    namespace fs = boost::filesystem;
#endif
    const fs::path _path_(path);
    if (fs::is_regular_file(_path_))
    {
        filename = _path_.filename().string();
        std::string extension = _path_.extension().string();
        filename.erase(filename.size() - extension.size());
        return true;
    }
    else
        return false;
}

} // namespace interface
