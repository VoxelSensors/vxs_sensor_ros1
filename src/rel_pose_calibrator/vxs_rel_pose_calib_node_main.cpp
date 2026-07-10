#include "vxs_rel_pose_calib_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vxs_rel_pose_calib");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    vxs_ros1::VxsRelPoseCalib vxs_rel_pose_calib(nh, nhp);

    // Now spin ...
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}