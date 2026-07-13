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

#include "rel_pose_calibrator/RelPoseCalibrator.hpp"

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

    private:
        ros::NodeHandle nh_, nhp_;
        //! Calibration mode (true). If false, cerification (diplay) mode.
        bool calib_mode_ = true;
        //! Pose calibration filename
        std::string rel_pose_filename_;
        //! Camera calibration filename
        std::string rgb_calib_filename_;
        //! RGB camera index
        int rgb_index_;
        //! Sensor calibration json
        std::string sensor_calib_json_;
        //! Sensor config json
        std::string sensor_config_json_;
        //! Points filename
        std::string points_filename_;

        //! Camera calibration
        posecalib::RelPoseCalibrator::CalibrationData rgb_calib_;
        //! Sensor calibration
        posecalib::RelPoseCalibrator::CalibrationData sensor_calib_;
        //! Relative pose calibration (wither loaded or estimated)
        posecalib::RelPoseCalibrator::CalibrationData rel_pose_;
        //! rgb image size
        cv::Size_<int> rgb_size_;
        //! FPS
        int fps_;
        //! Streaming duration
        int streaming_duration_;

        //! Capture object
        std::shared_ptr<cv::VideoCapture> cap_;

        //! Main processing loop
        void MainLoop();
    };
} // namespace vxs_ros1

#endif