#include "subscriber/vxs_subscriber.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vxs_subscriber");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    vxs_ros1::VxsSensorSubscriber vxs_subscriber(nh, nhp);

    // Now spin ...
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}