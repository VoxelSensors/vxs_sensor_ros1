/**
 * Relative pose calibrator
 * RelPoseCalibrator.hpp
 *
 * George Terzakis, VoxelSensors, July 2026
 */

#ifndef RELAPOSECALIB_HPP_
#define RELAPOSECALIB_HPP_

#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <SDK2.h> // for vxsdk::XYZT and vxsdk::XYT

namespace vxs_ros1
{
    namespace posecalib
    {
        class RelPoseCalibrator
        {

        public:
            // !==========================================
            // !CONFIGURATION
            // !==========================================
            // !Set to 1 for Data Collection & Calibration
            // !Set to 2 for Verification / Projection
            enum class Mode
            {
                DataCollection,
                Verification
            };

            struct CalibrationData
            {
                cv::Matx<float, 3, 3> K;
                cv::Vec<float, 5> dist;
                cv::Vec3f rvec;
                cv::Vec3f tvec;
            };

            static const std::string DEFAULT_CALIB_FILE;
            static const std::string DEFAULT_DATA_FILE;

            //! Constructor
            RelPoseCalibrator(                                              //
                const std::string &cam_calib_yaml,                          //
                const std::string &sensor_config_json,                      //
                const std::string &out_calib_filename = DEFAULT_CALIB_FILE, //
                const std::string &out_data_filename = DEFAULT_DATA_FILE);

            //! Find largest white blob center in RGB image
            bool FindRGBTarget(       //
                const cv::Mat &input, //
                cv::Point2f &center,  //
                cv::Mat &debug_img);

            //! Process SDK XYXYT Data
            void ProcessXYT(                           //
                const std::vector<vxsdk::vxXYT> &data, //
                cv::Point2f &mean_val,                 //
                cv::Mat &display_img);
            //! Process the 3D points (XYZT)
            bool ProcessXYZT(                           //
                const std::vector<vxsdk::vxXYZT> &data, //
                cv::Point3f &mean_pt);

        private:
            //! Camera calibration yaml filename
            std::string cam_calib_yaml_;
            //! Sensor config json filename
            std::string sensor_config_json_;
            //! Calibration (csv) output filename
            std::string out_calib_filename_;
            //! Data point output filename
            std::string out_data_filename_;

        public:
            //! Helper to save calibration to CSV
            static void SavePoseCalibrationData( //
                const std::string &filename,     //
                const cv::Vec3f &rvec,           //
                const cv::Vec3f &tvec,           //
                const cv::Matx<float, 3, 3> &K,  //
                const cv::Vec<float, 5> &distCoefs);

            //! Helper to load calibration from CSV
            static bool LoadPoseCalibrationData(const std::string &filename, CalibrationData &data);
            //! Helper to load calibration from CSV
            static bool LoadIntrinsicCalibrationData( //
                const std::string &filename,          //
                RelPoseCalibrator::CalibrationData &data);

            //! Helper to load sensor calibration
            static bool LoadSensorCalibrationFromJson( //
                const std::string &calib_json,         //
                cv::Matx<float, 3, 3> &K,              //
                cv::Vec<float, 5> &dist_coefs);

            //! Helper to load rgb calibration from yaml (uses cv::fileStorage)
            static bool LoadRGBCalibration(        //
                const std::string &rgb_calib_yaml, //
                const cv::Size_<int> &image_size,  //
                cv::Matx<float, 3, 3> &K,          //
                cv::Vec<float, 5> &D);
        };
    } // posecalib
} // namespace vxs_ros1

#endif
