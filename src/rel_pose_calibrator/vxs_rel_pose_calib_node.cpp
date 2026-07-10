#include "vxs_rel_pose_calib_node.hpp"

namespace vxs_ros1
{
    VxsRelPoseCalib::VxsRelPoseCalib(          //
        const ros::NodeHandle &nh,             //
        const ros::NodeHandle &nhp) : nh_(nh), //                                                //
                                      nhp_(nhp)
    {
        std::string package_directory = ros::package::getPath("vxs_rel_pose_calib");

        // Declare & Get parameters
        nhp.param<bool>("calibration_mode", calibrator_mode_, true);
        nhp.param<std::string>("camera_calib_filename", camera_calib_filename_, "");
        nhp.param<std::string>("sensor_calib_json", sensor_calib_filename_, "");
        nhp.param<std::string>("sensor_config_json", camera_calib_filename_, "");
        nhp.param<std::string>("rel_pose_filename", rel_pose_filename_, posecalib::RelPoseCalibrator::DEFAULT_CALIB_FILE);
        nhp.param<std::string>("points_filename", points_filename_, posecalib::RelPoseCalibrator::DEFAULT_DATA_FILE);
    }
};
} // namespace vxs_ros1