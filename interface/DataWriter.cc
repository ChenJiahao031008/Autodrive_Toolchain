#include "DataWriter.h"

namespace interface
{
void DataWriter::SaveTrajectory(const std::string &filename, const std::vector<sensorData::PoseData> &trajectory, save_options mode)
{
    AINFO << "Saving camera trajectory to " << filename;

    // sort(trajectory.begin(),trajectory.end());
    std::ofstream output(filename.c_str(), std::ios::out);
    if (!output.is_open())
    {
        AERROR << "Open File " << filename  << " Failed.";
        return;
    }
    output << std::fixed;

    for ( auto data: trajectory)
    {
        double timestamp = data.GetTimestamp();
        Eigen::Quaternionf q = data.GetRotation();
        Eigen::Vector3f t = data.GetPosition();
        if (mode == save_options::KITTI_INVERSE || mode == save_options::TUM_INVERSE)
        {
            Eigen::Matrix3f Rwc = q.matrix().transpose();
            Eigen::Vector3f twc = -Rwc * t;
            q = Eigen::Quaternionf(Rwc);
            t = twc;
        }

        if (mode == save_options::TUM || mode == save_options::TUM_INVERSE)
        {
            //字符串格式:  时间戳 x y z qx qy qz qw
            output
            << std::setprecision(6) << timestamp << " " << std::setprecision(9)
            << t(0) << " " << t(1) << " " << t(2) << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << std::endl;
        }
        else if(mode == save_options::KITTI || mode == save_options::KITTI_INVERSE)
        {
            Eigen::Matrix3f R = q.matrix();
            output << std::setprecision(9)
            << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t(0) << " "
            << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t(1) << " "
            << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t(2)
            << std::endl;
        }
        else{
            AFATAL << "ERROR save_options.";
            return;
        }
    }
    output.close();
    AINFO << "Trajectory Saved!";

}

}
