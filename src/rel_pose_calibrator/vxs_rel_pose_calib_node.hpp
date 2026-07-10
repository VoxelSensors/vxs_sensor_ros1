/**
 * relative pose calibration node
 *
 * George Terzakis, VoxelSensors, July 2026
 */

#ifndef VXS_RELPOSE_CALIB_NODE_HPP__
#define VXS_RELPOSE_CALIB_NODE_HPP__

#include <memory>
#include <thread>
#include <string>

#include <thread>
#include <shared_mutex>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "RelPoseCalibrator.hpp"

#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_ros1
{
    class VxsRelPoseCalib
    {

    public:
        VxsRelPoseCalib(               //
            const ros::NodeHandle &nh, //
            const ros::NodeHandle &nhp);
    };
} // namespace vxs_ros1

#endif