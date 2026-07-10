#include "RelPoseCalibrator.hpp"

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
        void RelPoseCalibrator::LoadSensorCalibrationFromJson( //
            const std::string &calib_json,                     //
            cv::Matx<float, 3, 3> &K,                          //
            cv::Vec<float, 5> &dist)
        {
            // @TODO: Read the config to acquire number of cameras! Assuming stereo for now....
            cv::FileStorage fs(calib_json, 0);
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
        }

        // ==========================================
        // MAIN
        // ==========================================
        int main(int argc, char *argv[])
        {
            std::string calib_filename = "./new_new_sensor_calib.json";
            std::string config_filename = "./and2_noMedian_golden_imu_1.json";

            bool skip = false;
            if (argc > 1)
            {
                for (int i = 0; i < argc; i++)
                {
                    if (i == 0)
                    {
                        std::cout << "Argument: " << std::endl;
                    }
                    else
                    {
                        std::cout << argv[i] << std::endl;
                    }
                    if (i == 0 || skip)
                    {
                        // does nothing
                        skip = false;
                    }
                    /*
                    else if (strcmp(argv[i], "--color_intrinsics") == 0)
                    {
                        skip = true;
                        assert(i + 1 < argc);
                        calibRGBIntraName = argv[i + 1];
                    }
                    */
                    else if (strcmp(argv[i], "--config") == 0)
                    {
                        skip = true;
                        assert(i + 1 < argc);
                        config_filename = argv[i + 1];
                    }
                    else if (strcmp(argv[i], "--sensor_intrinsics") == 0)
                    {
                        skip = true;
                        assert(i + 1 < argc);
                        calib_filename = argv[i + 1];
                    }
                }
            }

            CalibrationData intraData;

            const cv::Matx<double, 3, 3> logi4K_K = {//
                                                     6.6225035001683091e+02, 0., 3.4585341641799397e+02, 0.,
                                                     6.5844040381453078e+02, 2.2950251102436189e+02, 0., 0., 1.};

            const cv::Vec<double, 5> logi4K_dist_coefs = {2.1370439882736239e-01, -3.9585898806283154e-01,
                                                          -6.2616916163493411e-03, 9.2965071775274813e-03,
                                                          -1.4630680298126895e+00};

            cv::Matx<float, 3, 3> sK;
            cv::Vec<float, 5> sdist;
            LoadSensorCalibrationFromJson(calib_filename, sK, sdist);

            std::cout << "Sensor Intrinsics: " << sK << std::endl;
            std::cout << "Sensor dist. coefficients: " << sdist << std::endl;

            // 1. Initialize SDK
            int FPS = 30;
            uint32_t streamingDuration = 10;

            //  Detect sensor
            vxsdk::vxCameraType cam_type = vxsdk::vxDetectCameras();
            //
            vxsdk::vxFlag init_flags = vxsdk::vxFlag::XYZT;
            int nbThreads = 4;
            int binningAmount = 1;

            vxsdk::vxSetFPS(FPS);
            vxsdk::vxSetStreamingDuration(streamingDuration);
            vxsdk::vxSetFilteringParameters(2, 5, 5, 0.2, 0.1, 4, 10);

            int camNum = vxStartSystem(config_filename.c_str(), calib_filename.c_str(), init_flags, cam_type);

            if (camNum < 0)
            {
                std::cerr << "Failed to start SDK." << std::endl;
                return -1;
            }

            // 2. Initialize RGB Camera (OpenCV)
            cv::VideoCapture cap(4); // Open default camera
            if (!cap.isOpened())
            {
                std::cerr << "Failed to open RGB Camera." << std::endl;
                return -1;
            }

            cv::Size image_size;
            image_size.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            image_size.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

            std::cout << "Capture size: " << image_size << std::endl;

            // Sensor webcam (logitech 4K Stream Edition). Calibration at max. resolution (1080 x 1920)
            cv::Size logi4K_max_size(640, 480);
            const double scaler_x = (image_size.width * 1.0) / logi4K_max_size.width;
            const double scaler_y = (image_size.height * 1.0) / logi4K_max_size.height;

            intraData.K = cv::Mat(                                           //
                cv::Matx<double, 3, 3>(                                      //
                    logi4K_K(0, 0) * scaler_x, 0, logi4K_K(0, 2) * scaler_x, //
                    0, logi4K_K(1, 1) * scaler_y, logi4K_K(1, 2) * scaler_y, //
                    0, 0, 1));
            intraData.dist = cv::Mat(logi4K_dist_coefs);

            // Prepare structures
            cv::Mat rgbFrame, rgbVis;
            cv::Mat sdkVis;

            std::vector<cv::Point3f> objectPoints; // XYZ from SDK
            std::vector<cv::Point2f> imagePoints;  // XY from RGB
            std::vector<cv::Point2f> sdk2DPoints;  // XY from SDK (for CSV record)

            CalibrationData calibData;

            // For Mode 2, pre-load data
            if (CURRENT_MODE == 2)
            {
                if (!loadCalibrationData(CALIB_FILE, calibData))
                {
                    std::cout << "Calibration file missing, cannot run Mode 2." << std::endl;
                    return -1;
                }
                std::cout << "Mode 2: Calibration loaded." << std::endl;
            }
            else
            {
                std::cout << "Mode 1: Collect data (Spacebar) -> Calibrate & Save (C)." << std::endl;
            }

            int iter = 0;

            // 3. Infinite Loop
            while (true)
            {
                // Grab RGB Frame
                cap >> rgbFrame;
                if (rgbFrame.empty())
                    continue;

                // SDK Data Check
                while (vxsdk::vxCheckForData())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                // Get Data Pointers

                // NOTE: vxGetXYXYT/XYZT signatures depend on specific SDK version.
                // Assuming getting pointer and count logic based on provided context.
                // Usually SDK returns a pointer, we need to know how many points (often fixed or returned).
                // Based on previous code, looks like `nbPoints` needs to be fetched or is implicit.
                // Assuming we fetch a fixed buffer or SDK manages size internally, but usually we need a count.
                // Let's assume vxCheckForData ensures current buffer is valid.
                // We use a safe arbitrary number if not returned, or assume SDK provides 'nbPoints' reference.
                // *Correction based on snippet*: `vxGetXYXYT(nbPoints)` takes nbPoints as arg?
                // In snippet: `vxsdk::vxXYXYT* xyt0 = vxsdk::vxGetXYT(nbPoints, 0);` -> nbPoints is passed by reference likely?
                // Let's assume the previous code pattern:

                // Re-fetching nbPoints logic from SDK based on pipeline type 'all'
                // This part depends heavily on SDK impl, assuming vxGetXYZT returns count or updates ref.
                // Let's assume we call a function that populates nbPoints or we use a fixed size logic from prior code.
                // In the snippet, `vxsdk::vxGetXYXYT(nbPoints)` passes nbPoints.

                // IMPORTANT: In the provided snippet, nbPoints was initialized to 0 inside the `else` block.
                // SDK functions likely update it.

                int num_points = 0;
                vxsdk::vxXYZT *xyztData = vxsdk::vxGetXYZT(num_points);
                // std::cout << " XYZT nbPoints: " << num_points << std::endl;
                std::vector<vxsdk::vxXYZT> points3d;
                points3d.reserve(num_points);
                std::vector<vxsdk::vxXYT> img_points;
                img_points.reserve(num_points);
                for (int i = 0; i < num_points; i++)
                {
                    points3d.emplace_back(xyztData[i]);
                    const float X = xyztData[i].x;
                    const float Y = xyztData[i].y;
                    const float Z = xyztData[i].z;
                    if (Z < 1e-3)
                        continue;
                    // Project on image
                    const int x = std::lround(sK(0, 0) * X / Z + sK(0, 2));
                    const int y = std::lround(sK(1, 1) * Y / Z + sK(1, 2));
                    if (x < 0 || x > 299 || y < 0 || y > 299)
                        continue;
                    vxsdk::vxXYT xyt;
                    xyt.x = x;
                    xyt.y = y;
                    xyt.timestamp = xyztData[i].timestamp;
                    img_points.emplace_back(xyt);
                }
                // std::cout << " XYT nbPoints: " << img_points.size() << std::endl;

                // --- DATA PROCESSING ---

                cv::Point2f rgbCenter(0, 0);
                bool rgbFound = findRGBTarget(rgbFrame, rgbCenter, rgbVis);

                cv::Point2f sdkMean2D(0, 0);
                processXYT(img_points, sdkMean2D, sdkVis);

                cv::Point3f sdkMean3D(0, 0, 0);
                bool sdk3DFound = processXYZT(points3d, sdkMean3D);

                // --- MODE 1: DATA COLLECTION ---
                if (CURRENT_MODE == 1)
                {
                    // Visualization
                    cv::imshow("RGB Source", rgbVis);
                    cv::imshow("SDK Event Accumulation", sdkVis);

                    int key = cv::waitKey(1);

                    // Spacebar: Record
                    if (key == 32)
                    {
                        if (rgbFound && sdk3DFound)
                        {
                            objectPoints.push_back(sdkMean3D);
                            imagePoints.push_back(rgbCenter);
                            sdk2DPoints.push_back(sdkMean2D);

                            std::cout << "Recorded: RGB(" << rgbCenter.x << "," << rgbCenter.y << ") "
                                      << "SDK3D(" << sdkMean3D.x << "," << sdkMean3D.y << "," << sdkMean3D.z << ")" << std::endl;
                        }
                        else
                        {
                            std::cout << "Skipped: Targets not found in both cameras." << std::endl;
                        }
                    }

                    // 'c': Compute and Save
                    if (key == 'c' || key == 'C')
                    {
                        if (objectPoints.size() < 4)
                        {
                            std::cout << "Need at least 4 points to calibrate." << std::endl;
                        }
                        else
                        {
                            // Save Raw Points CSV
                            std::ofstream outFile(DATA_FILE);
                            outFile << "rgb_x,rgb_y,sdk_mean_x,sdk_mean_y,sdk_z_x,sdk_z_y,sdk_z_z\n";
                            for (size_t i = 0; i < objectPoints.size(); ++i)
                            {
                                outFile << imagePoints[i].x << "," << imagePoints[i].y << ","
                                        << sdk2DPoints[i].x << "," << sdk2DPoints[i].y << ","
                                        << objectPoints[i].x << "," << objectPoints[i].y << "," << objectPoints[i].z << "\n";
                            }
                            outFile.close();
                            std::cout << "Points saved to " << DATA_FILE << std::endl;

                            // Calibrate (SolvePnP)
                            // Heuristic Camera Matrix (Assumes generic webcam if not calibrated)
                            // fx, fy ~ width, cx, cy ~ center
                            double w = rgbFrame.cols;
                            double h = rgbFrame.rows;

                            cv::Mat rvec, tvec;

                            std::cout << "Calibration before: " << intraData.K << std::endl;

                            cv::solvePnP(objectPoints, imagePoints, intraData.K, intraData.dist, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
                            std::cout << "Calibration after: " << intraData.K << std::endl;

                            SaveCalibrationData(CALIB_FILE, rvec, tvec, intraData.K, intraData.dist);

                            std::cout << "Exiting..." << std::endl;
                            break;
                        }
                    }
                }
                // --- MODE 2: VERIFICATION ---
                else if (CURRENT_MODE == 2)
                {
                    // Project 3D Point to RGB
                    if (sdk3DFound)
                    {
                        std::vector<cv::Point3f> pts3d;
                        pts3d.push_back(sdkMean3D);
                        std::vector<cv::Point2f> projectedPts;

                        cv::projectPoints(pts3d, calibData.rvec, calibData.tvec, calibData.K, calibData.dist, projectedPts);

                        // Draw Red Cross at projected location
                        if (projectedPts.size() > 0)
                        {
                            cv::drawMarker(rgbVis, projectedPts[0], cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 25, 3);
                        }
                    }

                    cv::imshow("RGB Verification (Red=Projected, Green=Blob)", rgbVis);
                    cv::imshow("SDK Event Accumulation", sdkVis);
                    if (cv::waitKey(1) == 27)
                        break; // ESC to exit
                }
            }

            vxsdk::vxStopSystem();
            return 0;
        }

    } // [psecalib]
} // namespace vxs_ros1