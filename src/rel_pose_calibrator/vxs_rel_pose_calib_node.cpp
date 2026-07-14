#include "vxs_rel_pose_calib_node.hpp"

namespace vxs_ros1
{
    VxsRelPoseCalib::VxsRelPoseCalib(          //
        const ros::NodeHandle &nh,             //
        const ros::NodeHandle &nhp) : nh_(nh), //
                                      nhp_(nhp)
    {
        std::string package_directory = ros::package::getPath("vxs_rel_pose_calib");

        // Declare & Get parameters
        bool calib_mode;
        nhp.param<bool>("calib_mode", calib_mode, true);
        ROS_INFO_STREAM("Mode: " << calib_mode ? "Calibration" : "Validation.");
        std::string rgb_calib_filename;
        nhp.param<std::string>("rgb_calib_filename", rgb_calib_filename, "");
        ROS_INFO_STREAM("RGB calibration filename: " << rgb_calib_filename);
        nhp.param<std::string>("sensor_calib_json", sensor_calib_json_, "");
        ROS_INFO_STREAM("RGB calibration json: " << sensor_calib_json_);
        nhp.param<std::string>("sensor_config_json", sensor_config_json_, "");
        ROS_INFO_STREAM("SDK config json: " << sensor_config_json_);
        std::string rel_pose_filename;
        nhp.param<std::string>("rel_pose_filename", rel_pose_filename, posecalib::RelPoseCalibrator::DEFAULT_CALIB_FILE);
        ROS_INFO_STREAM("Output (relative pose) calibration filename: " << rel_pose_filename);
        std::string points_filename;
        nhp.param<std::string>("points_filename", points_filename, posecalib::RelPoseCalibrator::DEFAULT_DATA_FILE);
        ROS_INFO_STREAM("Output (points) data filename: " << points_filename);
        nhp.param<int>("rgb_index", rgb_index_, -1);
        ROS_INFO_STREAM("RGB camera index: " << rgb_index_);
        cv::Size_<int> rgb_size;
        nhp.param<int>("rgb_width", rgb_size.width, 640);
        nhp.param<int>("rgb_height", rgb_size.height, 480);
        ROS_INFO_STREAM("RGB camera size: " << rgb_size);
        nhp.param<int>("fps", fps_, 30);
        ROS_INFO_STREAM("Fps: " << fps_);
        nhp.param<int>("streaming_duration", streaming_duration_, 30);
        ROS_INFO_STREAM("Streaming duration: " << streaming_duration_);

        calibrator_ = std::make_shared<posecalib::RelPoseCalibrator>(rgb_size,           //
                                                                     rgb_calib_filename, //
                                                                     sensor_calib_json_, //
                                                                     calib_mode,         //
                                                                     rel_pose_filename,  //
                                                                     points_filename);
        // 1. Initialize SDK
        //
        //  Detect sensor
        vxsdk::vxCameraType cam_type = vxsdk::vxDetectCameras();
        // Setup SDK
        vxsdk::vxFlag init_flags = vxsdk::vxFlag::XYZT;
        vxsdk::vxSetFPS(fps_);
        vxsdk::vxSetStreamingDuration(streaming_duration_);
        vxsdk::vxSetFilteringParameters(2, 5, 5, 0.2, 0.1, 4, 10);
        // Start SDK
        int num_cams = vxStartSystem(sensor_config_json_.c_str(), sensor_calib_json_.c_str(), init_flags, cam_type);

        if (num_cams < 0)
        {
            ROS_ERROR_STREAM("Failed to start SDK. Number of cameras: " << num_cams << ". Exiting...");
            exit(-1);
        }

        // 2. Initialize RGB Camera (OpenCV)
        cap_ = std::make_shared<cv::VideoCapture>(rgb_index_);
        if (!cap_->isOpened())
        {
            ROS_ERROR_STREAM("Failed to open RGB Camera.");
            exit(-1);
        }

        // Configure the capture
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, calibrator_->GetRgbSize().width);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, calibrator_->GetRgbSize().height);

        cv::Size_<int> test_size;
        test_size.width = cap_->get(cv::CAP_PROP_FRAME_WIDTH);
        test_size.height = cap_->get(cv::CAP_PROP_FRAME_HEIGHT);

        ROS_INFO_STREAM("Capture size set: " << test_size);

        if (!calibrator_->ValidationMode())
        {
            ROS_INFO_STREAM("CALIBRATION MODE: Collect data (Spacebar) -> Calibrate & Save (C).");
        }
        else
        {
            ROS_INFO_STREAM("Validation mode: Relative pose calibration loaded.");
        }

        // 2. Start the calibration loop
        MainLoop();

        // 3. Done. Shutdown the node
        ros::shutdown();
    }

    void VxsRelPoseCalib::MainLoop()
    {

        // Prepare structures
        cv::Mat rgb_frame, display_img;
        cv::Mat display_depth;

        std::vector<cv::Point3f> object_points;  // XYZ from SDK
        std::vector<cv::Point2f> image_points;   // XY from RGB
        std::vector<cv::Point2f> depth_points2D; // XY from SDK (for CSV record)

        const cv::Matx<float, 3, 3> &sK = calibrator_->GetSensorCalibration().K;
        const cv::Vec<float, 5> &dist = calibrator_->GetSensorCalibration().dist;

        while (ros::ok())
        {
            // Grab RGB Frame
            *cap_ >> rgb_frame;
            if (rgb_frame.empty())
                continue;

            // SDK Data Check
            while (vxsdk::vxCheckForData())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            int num_points = 0;
            vxsdk::vxXYZT *xyztData = vxsdk::vxGetXYZT(num_points);
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

            // --- DATA PROCESSING ---

            cv::Point2f rgb_mean(0, 0);
            bool rgb_found = calibrator_->FindRGBTarget(rgb_frame, rgb_mean, display_img);

            cv::Point2f depth_mean2D(0, 0);
            calibrator_->ProcessXYT(img_points, depth_mean2D, display_depth);

            cv::Point3f depth_mean3D(0, 0, 0);
            bool depth3D_found = calibrator_->ProcessXYZT(points3d, depth_mean3D);

            // --- DATA COLLECTION MODE ---
            if (!calibrator_->ValidationMode())
            {
                // Visualization
                cv::imshow("RGB Source", display_img);
                cv::imshow("SDK Event Accumulation", display_depth);

                int key = cv::waitKey(1);

                // Spacebar: Record
                if (key == 32)
                {
                    if (rgb_found && depth3D_found)
                    {
                        object_points.push_back(depth_mean3D);
                        image_points.push_back(rgb_mean);
                        depth_points2D.push_back(depth_mean2D);

                        ROS_INFO_STREAM("Recorded: RGB(" << rgb_mean.x << "," << rgb_mean.y << ") "
                                                         << "SDK3D(" << depth_mean3D.x << "," << depth_mean3D.y << "," << depth_mean3D.z << ")");
                    }
                    else
                    {
                        ROS_WARN_STREAM("Skipped: Targets not found in both cameras.");
                    }
                }

                // 'c': Compute and Save
                if (key == 'c' || key == 'C')
                {
                    if (object_points.size() < 4)
                    {
                        ROS_ERROR_STREAM("Need at least 4 points to calibrate.");
                    }
                    else
                    {
                        // Save Raw Points CSV
                        std::ofstream outFile(calibrator_->GetDataFilename());
                        outFile << "rgb_x,rgb_y,sdk_mean_x,sdk_mean_y,sdk_z_x,sdk_z_y,sdk_z_z\n";
                        for (size_t i = 0; i < object_points.size(); ++i)
                        {
                            outFile << image_points[i].x << "," << image_points[i].y << ","
                                    << depth_points2D[i].x << "," << depth_points2D[i].y << ","
                                    << object_points[i].x << "," << object_points[i].y << "," << object_points[i].z << "\n";
                        }
                        outFile.close();
                        ROS_INFO_STREAM("Points saved to " << calibrator_->GetDataFilename());

                        // Calibrate (SolvePnP)
                        // Heuristic Camera Matrix (Assumes generic webcam if not calibrated)
                        // fx, fy ~ width, cx, cy ~ center
                        double w = rgb_frame.cols;
                        double h = rgb_frame.rows;

                        cv::Mat rvec, tvec;

                        std::cout << "Calibration before: " << calibrator_->rgb_calib_.K << std::endl;

                        cv::solvePnP(                     //
                            object_points,                //
                            image_points,                 //
                            calibrator_->rgb_calib_.K,    //
                            calibrator_->rgb_calib_.dist, //
                            rvec, tvec,                   //
                            false,                        //
                            cv::SOLVEPNP_SQPNP);
                        std::cout << "Calibration after: " << calibrator_->rgb_calib_.K << std::endl;

                        posecalib::RelPoseCalibrator::SavePoseCalibrationData( //
                            calibrator_->GetPoseCalibrationFilename(),         //
                            rvec,                                              //
                            tvec,                                              //
                            calibrator_->rgb_calib_.K,                         //
                            calibrator_->rgb_calib_.dist);

                        ROS_INFO_STREAM("Exiting...");
                        break;
                    }
                }
            }
            // --- MODE 2: VERIFICATION ---
            else
            {
                // Project 3D Point to RGB
                if (depth3D_found)
                {
                    std::vector<cv::Point3f> pts3d;
                    pts3d.push_back(depth_mean3D);
                    std::vector<cv::Point2f> projected_pts;

                    cv::projectPoints(                          //
                        pts3d,                                  //
                        calibrator_->GetPoseCalibration().rvec, //
                        calibrator_->GetPoseCalibration().tvec, //
                        calibrator_->GetPoseCalibration().K,    //
                        calibrator_->GetPoseCalibration().dist, //
                        projected_pts);

                    // Draw Red Cross at projected location
                    if (projected_pts.size() > 0)
                    {
                        cv::drawMarker(display_img, projected_pts[0], cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 25, 3);
                    }
                }

                cv::imshow("RGB Verification (Red=Projected, Green=Blob)", display_img);
                cv::imshow("SDK Event Accumulation", display_depth);
                if (cv::waitKey(1) == 27)
                    break; // ESC to exit
            }
        }

        vxsdk::vxStopSystem();
    }

} // namespace vxs_ros1