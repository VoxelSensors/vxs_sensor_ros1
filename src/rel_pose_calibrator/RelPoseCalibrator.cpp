#include "rel_pose_calibrator/RelPoseCalibrator.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace vxs_ros1
{
    namespace posecalib
    {
        const std::string RelPoseCalibrator::DEFAULT_CALIB_FILE = "rgb_calibration.csv";
        const std::string RelPoseCalibrator::DEFAULT_DATA_FILE = "collected_points.csv";

        RelPoseCalibrator::RelPoseCalibrator(                                                //
            const std::string &cam_calib_yaml,                                               //
            const std::string &sensor_config_json,                                           //
            const std::string &out_calib_filename,                                           //
            const std::string &out_data_filename) : cam_calib_yaml_(cam_calib_yaml),         //
                                                    sensor_config_json_(sensor_config_json), //
                                                    out_calib_filename_(out_calib_filename), //
                                                    out_data_filename_(out_data_filename)
        {
        }

        void RelPoseCalibrator::SavePoseCalibrationData( //
            const std::string &filename,                 //
            const cv::Vec3f &rvec,                       //
            const cv::Vec3f &tvec,                       //
            const cv::Matx<float, 3, 3> &K,              //
            const cv::Vec<float, 5> &dist_coefs)
        {
            std::ofstream file(filename);
            if (file.is_open())
            {
                // Save rvec
                for (int i = 0; i < 3; i++)
                {
                    file << rvec[i] << (i == 2 ? "" : ",");
                }
                file << "\n";
                // Save tvec
                for (int i = 0; i < 3; i++)
                {
                    file << tvec[i] << (i == 2 ? "" : ",");
                }
                file << "\n";
                // Save Camera Matrix (3x3)
                for (int r = 0; r < 3; r++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        file << K(r, c) << (c == 2 ? "" : ",");
                    }
                    file << "\n";
                }
                for (int i = 0; i < 5; i++)
                {
                    file << dist_coefs[i] << (i == 4 ? "" : ",");
                }
                file << "\n";
                file.close();
                std::cout << "[Info] Calibration saved to " << filename << std::endl;
            }
        }

        bool RelPoseCalibrator::LoadPoseCalibrationData( //
            const std::string &filename,                 //
            RelPoseCalibrator::CalibrationData &data)
        {
            std::ifstream file(filename);
            if (!file.is_open())
            {
                std::cerr << "[Error] Could not open calibration file." << std::endl;
                return false;
            }

            std::string line, val;

            // Read rvec
            if (std::getline(file, line))
            {
                std::stringstream ss(line);
                for (int i = 0; i < 3; i++)
                {
                    std::getline(ss, val, ',');
                    data.rvec[i] = std::stod(val);
                }
            }
            // Read tvec
            if (std::getline(file, line))
            {
                std::stringstream ss(line);
                for (int i = 0; i < 3; i++)
                {
                    std::getline(ss, val, ',');
                    data.tvec[i] = std::stod(val);
                }
            }
            // Read Camera Matrix
            for (int r = 0; r < 3; r++)
            {
                if (std::getline(file, line))
                {
                    std::stringstream ss(line);
                    for (int c = 0; c < 3; c++)
                    {
                        std::getline(ss, val, ',');
                        data.K(r, c) = std::stod(val);
                    }
                }
            }

            // Read dist
            if (std::getline(file, line))
            {
                std::stringstream ss(line);
                for (int i = 0; i < 5; i++)
                {
                    std::getline(ss, val, ',');
                    data.dist[i] = std::stod(val);
                }
            }
            return true;
        }

        //! Helper to load calibration from CSV
        bool RelPoseCalibrator::LoadIntrinsicCalibrationData( //
            const std::string &filename,                      //
            RelPoseCalibrator::CalibrationData &data)
        {
            std::ifstream file(filename);
            if (!file.is_open())
            {
                std::cerr << "[Error] Could not open intrasics calibration file." << std::endl;
                return false;
            }

            std::string line, val;

            // Read Camera Matrix
            for (int r = 0; r < 3; r++)
            {
                if (std::getline(file, line))
                {
                    std::stringstream ss(line);
                    for (int c = 0; c < 3; c++)
                    {
                        std::getline(ss, val, ',');
                        data.K(r, c) = std::stod(val);
                    }
                }
            }

            // Read dist
            if (std::getline(file, line))
            {
                std::stringstream ss(line);
                for (int i = 0; i < 5; i++)
                {
                    std::getline(ss, val, ',');
                    data.dist[i] = std::stod(val);
                }
            }
            return true;
        }

        bool RelPoseCalibrator::FindRGBTarget( //
            const cv::Mat &input,              //
            cv::Point2f &center,               //
            cv::Mat &debug_img)
        {
            cv::Mat mask;
            // Strict white detection (adjust range if camera is noisy, e.g., 240-255)
            cv::inRange(input, cv::Scalar(240, 240, 240), cv::Scalar(255, 255, 255), mask);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            int maxIdx = -1;

            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if (area > maxArea && area > 50)
                { // Filter small noise
                    maxArea = area;
                    maxIdx = i;
                }
            }

            debug_img = input.clone();

            if (maxIdx >= 0)
            {
                cv::Moments mu = cv::moments(contours[maxIdx]);
                center = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
                                     static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));

                // Draw green cross
                cv::drawMarker(debug_img, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
                return true;
            }
            return false;
        }

        void RelPoseCalibrator::ProcessXYT(        //
            const std::vector<vxsdk::vxXYT> &data, //
            cv::Point2f &mean_pt,                  //
            cv::Mat &display_img)
        {
            display_img = cv::Mat::zeros(300, 300, CV_8UC3);

            if (data.empty())
                return;

            double sumX = 0, sumY = 0;
            int validPoints = 0;

            for (uint32_t i = 0; i < data.size(); ++i)
            {
                // Taking first pair x0, y0 as requested
                int x = static_cast<int>(data[i].x);
                int y = static_cast<int>(data[i].y);

                if (x >= 0 && x < 300 && y >= 0 && y < 300)
                {
                    display_img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // White dots
                    sumX += x;
                    sumY += y;
                    validPoints++;
                }
            }

            if (validPoints > 0)
            {
                mean_pt.x = static_cast<float>(sumX / validPoints);
                mean_pt.y = static_cast<float>(sumY / validPoints);
                // Draw green cross on mean
                cv::drawMarker(display_img, mean_pt, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
            }
        }

        bool RelPoseCalibrator::ProcessXYZT(        //
            const std::vector<vxsdk::vxXYZT> &data, //
            cv::Point3f &mean_pt)
        {
            if (data.empty())
                return false;

            double sumX = 0, sumY = 0, sumZ = 0;
            for (uint32_t i = 0; i < data.size(); ++i)
            {
                sumX += data[i].x;
                sumY += data[i].y;
                sumZ += data[i].z;
            }

            mean_pt.x = static_cast<float>(sumX / data.size());
            mean_pt.y = static_cast<float>(sumY / data.size());
            mean_pt.z = static_cast<float>(sumZ / data.size());
            return true;
        }

        //! Load sensor calibration
        bool RelPoseCalibrator::LoadSensorCalibrationFromJson( //
            const std::string &calib_json,                     //
            cv::Matx<float, 3, 3> &K,                          //
            cv::Vec<float, 5> &dist)
        {
            // @TODO: Read the config to acquire number of cameras! Assuming stereo for now....
            cv::FileStorage fs(calib_json, 0);
            if (!fs.isOpened())
            {
                return false;
            }
            cv::FileNode root = fs["Cameras"];
            cv::FileNode cam1 = root[0];
            dist = cv::Vec<float, 5>({cam1["Distortion"][0], cam1["Distortion"][1], cam1["Distortion"][2], cam1["Distortion"][3], cam1["Distortion"][4]});
            K = cv::Matx<float, 3, 3>({cam1["Intrinsic"][0][0], cam1["Intrinsic"][0][1], cam1["Intrinsic"][0][2], //
                                       cam1["Intrinsic"][1][0], cam1["Intrinsic"][1][1], cam1["Intrinsic"][1][2], //
                                       cam1["Intrinsic"][2][0], cam1["Intrinsic"][2][1], cam1["Intrinsic"][2][2]});

            /*
            cams_[0].t = cv::Vec3f({cam1["Translation"][0], cam1["Translation"][1], cam1["Translation"][2]});           //
            cams_[0].R = cv::Matx<float, 3, 3>({cam1["Rotation"][0][0], cam1["Rotation"][0][1], cam1["Rotation"][0][2], //
                                                cam1["Rotation"][1][0], cam1["Rotation"][1][1], cam1["Rotation"][1][2], //
                                                cam1["Rotation"][2][0], cam1["Rotation"][2][1], cam1["Rotation"][2][2]});
            cams_[0].dist = cv::Vec<float, 5>({cam1["Distortion"][0], cam1["Distortion"][1], cam1["Distortion"][2], cam1["Distortion"][3], cam1["Distortion"][4]});
            cams_[0].K = cv::Matx<float, 3, 3>({cam1["Intrinsic"][0][0], cam1["Intrinsic"][0][1], cam1["Intrinsic"][0][2], //
                                                cam1["Intrinsic"][1][0], cam1["Intrinsic"][1][1], cam1["Intrinsic"][1][2], //
                                                cam1["Intrinsic"][2][0], cam1["Intrinsic"][2][1], cam1["Intrinsic"][2][2]});
            cams_[0].image_size = cv::Size_<int>(cam1["SensorSize"]["Width"], cam1["SensorSize"]["Height"]);

            cv::FileNode cam2 = root[1];
            cams_[1].t = cv::Vec3f({cam2["Translation"][0], cam2["Translation"][1], cam2["Translation"][2]});           //
            cams_[1].R = cv::Matx<float, 3, 3>({cam2["Rotation"][0][0], cam2["Rotation"][0][1], cam2["Rotation"][0][2], //
                                                cam2["Rotation"][1][0], cam2["Rotation"][1][1], cam2["Rotation"][1][2], //
                                                cam2["Rotation"][2][0], cam2["Rotation"][2][1], cam2["Rotation"][2][2]});
            cams_[1].dist = cv::Vec<float, 5>({cam2["Distortion"][0], cam2["Distortion"][1], cam2["Distortion"][2], cam2["Distortion"][3], cam1["Distortion"][4]});
            cams_[1].K = cv::Matx<float, 3, 3>({cam2["Intrinsic"][0][0], cam2["Intrinsic"][0][1], cam2["Intrinsic"][0][2], //
                                                cam2["Intrinsic"][1][0], cam2["Intrinsic"][1][1], cam2["Intrinsic"][1][2], //
                                                cam2["Intrinsic"][2][0], cam2["Intrinsic"][2][1], cam2["Intrinsic"][2][2]});
            cams_[1].image_size = cv::Size_<int>(cam2["SensorSize"]["Width"], cam2["SensorSize"]["Height"]);
            */
            return true;
        }

        //! Load RGB calibration from **yaml**
        bool RelPoseCalibrator::LoadRGBCalibration( //
            const std::string &rgb_calib_yaml,      //
            const cv::Size_<int> &image_size,       //
            cv::Matx<float, 3, 3> &K,               //
            cv::Vec<float, 5> &D)
        {
            cv::FileStorage fs(rgb_calib_yaml, cv::FileStorage::READ);

            if (!fs.isOpened())
            {
                return false;
            }

            int rgb_calib_width, rgb_calib_height;
            fs["width"] >> rgb_calib_width;
            fs["height"] >> rgb_calib_height;

            float scaler_x = 1;
            float scaler_y = 1;
            if (image_size.width != rgb_calib_width || image_size.height != rgb_calib_height)
            {
                scaler_x = (1.0 * image_size.width) / rgb_calib_width;
                scaler_y = (1.0 * image_size.height / rgb_calib_height);
            }

            cv::Matx<float, 1, 5> mxD;
            fs["D"] >> mxD;
            for (int i = 0; i < 5; i++)
            {
                D[i] = mxD(0, i);
            }

            fs["K"] >> K;
            K(0, 0) *= scaler_x;
            K(0, 2) *= scaler_x;
            K(1, 1) *= scaler_y;
            K(1, 2) *= scaler_y;

            // fs["P"] >> P;
            /// P(0, 0) *= scaler_x;
            // P(0, 2) *= scaler_x;
            // P(1, 1) *= scaler_y;
            /// P(1, 2) *= scaler_y;

            // fs["R"] >> rgbR;

            fs.release();
            return true;
        }

    } // namespace posecalib
} // namespace vxs_ros1