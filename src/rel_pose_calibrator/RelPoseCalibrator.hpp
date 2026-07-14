/**
 * Relative pose calibrator
 * RelPoseCalibrator.hpp
 *
 * George Terzakis, VoxelSensors, July 2026
 *
 * Adapted from code by Laurie Van Bogaert
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
                const cv::Size_<int> &rgb_size,                             //
                const std::string &rgb_calib_filename,                      //
                const std::string &sensor_calib_json,                       //
                const bool calib_mode,                                      //
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

            //! Get rgb size
            inline cv::Size_<int> GetRgbSize() const { return rgb_size_; }
            //! Get rgb calibration
            inline CalibrationData GetRgbCalibration() const { return rgb_calib_; }
            //! Get sensor calibration
            inline CalibrationData GetSensorCalibration() const { return sensor_calib_; }
            //! Get (relative) pose calibration
            inline CalibrationData GetPoseCalibration() const { return rel_pose_; }
            //! In validation mode?
            inline bool ValidationMode() const { return !calib_mode_; }
            //! Get the points /data filename
            inline std::string GetDataFilename() const { return out_data_filename_; }
            //! Get calibration filename
            inline std::string GetPoseCalibrationFilename() const { return out_calib_filename_; }

            //! Camera calibration
            CalibrationData rgb_calib_;

        private:
            // RGB size
            cv::Size_<int> rgb_size_;
            //! Camera calibration yaml filename
            std::string rgb_calib_filename_;
            //! Sensor calibration json filename
            std::string sensor_calib_json_;
            //! Calibration (csv) output filename
            std::string out_calib_filename_;
            //! Data point output filename
            std::string out_data_filename_;

            //! Sensor calibration
            CalibrationData sensor_calib_;
            //! Relative pose calibration (wither loaded or estimated)
            CalibrationData rel_pose_;

            //! Calibration mode (true). If false, cerification (diplay) mode.
            bool calib_mode_;

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
