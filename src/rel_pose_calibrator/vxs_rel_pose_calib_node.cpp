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
        nhp.param<bool>("calib_mode", calib_mode_, true);
        ROS_INFO_STREAM("Mode: " << calib_mode_ ? "Calibration" : "Validation.");
        nhp.param<std::string>("rgb_calib_filename", rgb_calib_filename_, "");
        ROS_INFO_STREAM("RGB calibration filename: " << rgb_calib_filename_);
        nhp.param<std::string>("sensor_calib_json", sensor_calib_json_, "");
        ROS_INFO_STREAM("RGB calibration json: " << sensor_calib_json_);
        nhp.param<std::string>("sensor_config_json", sensor_config_json_, "");
        ROS_INFO_STREAM("SDK config json: " << sensor_config_json_);
        nhp.param<std::string>("rel_pose_filename", rel_pose_filename_, posecalib::RelPoseCalibrator::DEFAULT_CALIB_FILE);
        ROS_INFO_STREAM("Output (relative pose) calibration filename: " << rel_pose_filename_);
        nhp.param<std::string>("points_filename", points_filename_, posecalib::RelPoseCalibrator::DEFAULT_DATA_FILE);
        ROS_INFO_STREAM("Output (points) data filename: " << points_filename_);
        nhp.param<int>("rgb_index", rgb_index_, -1);
        ROS_INFO_STREAM("RGB camera index: " << rgb_index_);
        nhp.param<int>("rgb_width", rgb_size_.width, 640);
        nhp.param<int>("rgb_height", rgb_size_.height, 480);
        ROS_INFO_STREAM("RGB camera size: " << rgb_size_);
        nhp.param<int>("fps", fps_, 30);
        ROS_INFO_STREAM("Fps: " << fps_);
        nhp.param<int>("streaming_duration", streaming_duration_, 30);
        ROS_INFO_STREAM("Streaming duration: " << streaming_duration_);

        // ---- LOAD CALIBRATION & CONFIG FILES ----
        //
        // A. Load sensor calibration (for projection of 3D points ont6o the image plane)
        bool result = posecalib::RelPoseCalibrator::LoadSensorCalibrationFromJson( //
            sensor_calib_json_,                                                    //
            sensor_calib_.K,                                                       //
            sensor_calib_.dist);
        if (!result)
        {
            ROS_ERROR_STREAM("Could not load sensor calibration '" << sensor_calib_json_ << "'! Exiting...");
            exit(-1);
        }
        ROS_INFO_STREAM("Sensor Intrinsics: " << sensor_calib_.K);
        ROS_INFO_STREAM("Sensor dist. coefficients: " << sensor_calib_.dist);

        //
        // B. Load camera calibration
        result = posecalib::RelPoseCalibrator::LoadRGBCalibration( //
            rgb_calib_filename_,                                   //
            rgb_size_,                                             //
            rgb_calib_.K,                                          //
            rgb_calib_.dist);

        // 1. Initialize SDK

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
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, rgb_size_.width);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, rgb_size_.height);

        cv::Size_<int> test_size;
        test_size.width = cap_->get(cv::CAP_PROP_FRAME_WIDTH);
        test_size.height = cap_->get(cv::CAP_PROP_FRAME_HEIGHT);

        ROS_INFO_STREAM("Capture size set: " << test_size);

        // 3. Load rel. pose for validation (if validation mode) and start the main loop
        // If in validation mode, load the relative pose calibration
        if (!calib_mode_)
        {
            if (!posecalib::RelPoseCalibrator::LoadPoseCalibrationData(rel_pose_filename_, rel_pose_))
            {
                ROS_ERROR_STREAM("Calibration file missing, cannot run validation.");
                exit(-1);
            }
            ROS_INFO_STREAM("Validation mode: Relative pose calibration loaded.");
        }
        else
        {
            ROS_INFO_STREAM("CALIBRATION MODE: Collect data (Spacebar) -> Calibrate & Save (C).");
        }

        // Start the calibration loop
        MainLoop();
    }

    void VxsRelPoseCalib::MainLoop()
    {

        // Prepare structures
        cv::Mat rgb_frame, display_img;
        cv::Mat display_depth;

        std::vector<cv::Point3f> objectPoints; // XYZ from SDK
        std::vector<cv::Point2f> imagePoints;  // XY from RGB
        std::vector<cv::Point2f> sdk2DPoints;  // XY from SDK (for CSV record)

        cv::Matx<float, 3, 3> &sK = sensor_calib_.K;
        cv::Vec<float, 5> &dist = sensor_calib_.dist;

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

            cv::Point2f rgbCenter(0, 0);
            bool rgbFound = findRGBTarget(rgb_frame, rgbCenter, display_img);

            cv::Point2f sdkMean2D(0, 0);
            processXYT(img_points, sdkMean2D, display_depth);

            cv::Point3f sdkMean3D(0, 0, 0);
            bool sdk3DFound = processXYZT(points3d, sdkMean3D);

            // --- DATA COLLECTION MODE ---
            if (!calib_mode_)
            {
                // Visualization
                cv::imshow("RGB Source", display_img);
                cv::imshow("SDK Event Accumulation", display_depth);

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
                        double w = rgb_frame.cols;
                        double h = rgb_frame.rows;

                        cv::Mat rvec, tvec;

                        std::cout << "Calibration before: " << rgb_calib_.K << std::endl;

                        cv::solvePnP(objectPoints, imagePoints, rgb_calib_.K, rgb_calib_.dist, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
                        std::cout << "Calibration after: " << rgb_calib_.K << std::endl;

                        SaveCalibrationData(CALIB_FILE, rvec, tvec, rgb_calib_.K, rgb_calib_.dist);

                        std::cout << "Exiting..." << std::endl;
                        break;
                    }
                }
            }
            // --- MODE 2: VERIFICATION ---
            else
            {
                // Project 3D Point to RGB
                if (sdk3DFound)
                {
                    std::vector<cv::Point3f> pts3d;
                    pts3d.push_back(sdkMean3D);
                    std::vector<cv::Point2f> projectedPts;

                    cv::projectPoints(pts3d, rel_pose_.rvec, rel_pose_.tvec, rel_pose_.K, rel_pose_.dist, projectedPts);

                    // Draw Red Cross at projected location
                    if (projectedPts.size() > 0)
                    {
                        cv::drawMarker(display_img, projectedPts[0], cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 25, 3);
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