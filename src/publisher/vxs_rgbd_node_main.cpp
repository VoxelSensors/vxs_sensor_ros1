#include "vxs_rgbd_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vxs_rgb_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    vxs_ros1::VxsRGBDPublisher vxs_rgbpublisher(nh, nhp);

    // Now spin ...
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}