#include "depth_filters.hpp"
#include <iostream>
namespace vxs_sensor_ros1
{
    namespace pcl_filters
    {
        cv::Mat DownsizeDepthImage(               //
            const int new_width,                  //
            const int new_height,                 //
            const cv::Mat &depth,                 //
            const cv::Matx<float, 3, 3> &K,       //
            std::vector<cv::Vec3f> &new_points3d, //
            const float &depth_scaler)
        {
            CV_Assert(depth.type() == CV_16U || depth.type() == CV_16S || depth.type() == CV_16UC1);
            cv::Mat small_depth(new_height, new_width, CV_16UC1);
            small_depth *= 0;
            new_points3d.clear();
            new_points3d.reserve(new_width * new_height);
            const float scaler_orgX_to_smallX = (1.0f * new_width) / depth.cols;
            const float scaler_orgY_to_smallY = (1.0f * new_height) / depth.rows;

            const float newfx = K(0, 0) * scaler_orgX_to_smallX;
            const float newcx = K(0, 2) * scaler_orgX_to_smallX;
            const float newfy = K(1, 1) * scaler_orgY_to_smallY;
            const float newcy = K(1, 2) * scaler_orgY_to_smallY;

            // Scale with filling
            cv::resize(depth, small_depth, cv::Size_<int>(new_width, new_height));
            int depth_counter = 0;

            const int step_x = depth.cols / new_width;
            const int step_y = depth.rows / new_height;
            for (int y0 = 0; y0 < depth.rows; y0 += step_y)
            {
                const int span_y = depth.rows - y0 <= step_y ? depth.rows - y0 - 1 : step_y;
                for (int x0 = 0; x0 < depth.cols; x0 += step_x)
                {
                    const int span_x = depth.cols - x0 <= step_x ? depth.cols - x0 - 1 : step_x;
                    float avgZ = 0;
                    int Z_count = 0;
                    for (int r = 0; r < span_y; r++)
                    {
                        for (int c = 0; c < span_x; c++)
                        {
                            const uint16_t Z0 = depth.at<uint16_t>(y0 + r, x0 + c);
                            if (Z0 > 0)
                            {
                                avgZ += 1.0f * Z0;
                                Z_count++;
                            }
                        }
                    }
                    if (Z_count == 0)
                    {
                        continue;
                    }

                    depth_counter++;

                    avgZ /= Z_count;
                    const int x = std::lround((x0 + 0.5f * span_x) * scaler_orgX_to_smallX);
                    const int y = std::lround((y0 + 0.5f * span_y) * scaler_orgY_to_smallY);

                    const uint16_t Z = std::lround(avgZ);
                    small_depth.at<uint16_t>(y, x) = Z;

                    const float X = (x - newcx) / newfx * Z;
                    const float Y = (y - newcy) / newfy * Z;

                    new_points3d.emplace_back(X * depth_scaler, Y * depth_scaler, Z * depth_scaler);
                }
            }
            return small_depth;
        }

        cv::Mat DownsizeDepthImage( //
            const int new_width,    //
            const int new_height,   //
            const cv::Mat &depth,   //
            const cv::Matx<float, 3, 3> &K)
        {
            CV_Assert(depth.type() == CV_16U || depth.type() == CV_16S || depth.type() == CV_16UC1);
            cv::Mat small_depth(new_height, new_width, CV_16UC1);
            small_depth *= 0;
            const float scaler_orgX_to_smallX = (1.0f * new_width) / depth.cols;
            const float scaler_orgY_to_smallY = (1.0f * new_height) / depth.rows;

            const float newfx = K(0, 0) * scaler_orgX_to_smallX;
            const float newcx = K(0, 2) * scaler_orgX_to_smallX;
            const float newfy = K(1, 1) * scaler_orgY_to_smallY;
            const float newcy = K(1, 2) * scaler_orgY_to_smallY;

            // Scale with filling
            // cv::resize(depth, small_depth, cv::Size_<int>(new_width, new_height));
            int depth_counter = 0;

            const int step_x = depth.cols / new_width;
            const int step_y = depth.rows / new_height;
            for (int y0 = 0; y0 < depth.rows; y0 += step_y)
            {
                const int span_y = depth.rows - y0 <= step_y ? depth.rows - y0 - 1 : step_y;
                for (int x0 = 0; x0 < depth.cols; x0 += step_x)
                {
                    const int span_x = depth.cols - x0 <= step_x ? depth.cols - x0 - 1 : step_x;
                    float avgZ = 0;
                    int Z_count = 0;
                    for (int r = 0; r < span_y; r++)
                    {
                        for (int c = 0; c < span_x; c++)
                        {
                            const uint16_t Z0 = depth.at<uint16_t>(y0 + r, x0 + c);
                            if (Z0 > 0)
                            {
                                avgZ += 1.0f * Z0;
                                Z_count++;
                            }
                        }
                    }
                    if (Z_count == 0)
                    {
                        continue;
                    }

                    depth_counter++;

                    avgZ /= Z_count;
                    const int x = std::lround((x0 + 0.5f * span_x) * scaler_orgX_to_smallX);
                    const int y = std::lround((y0 + 0.5f * span_y) * scaler_orgY_to_smallY);

                    const uint16_t Z = std::lround(avgZ);
                    small_depth.at<uint16_t>(y, x) = Z;

                    const float X = (x - newcx) / newfx * Z;
                    const float Y = (y - newcy) / newfy * Z;
                }
            }
            return small_depth;
        }

        cv::Mat UpsizeDepthImage( //
            const int new_width,  //
            const int new_height, //
            const cv::Mat &depth, //
            const cv::Matx<float, 3, 3> &K)
        {
            CV_Assert((depth.type() == CV_16U || depth.type() == CV_16S || depth.type() == CV_16UC1) && depth.rows <= new_height && depth.cols <= new_width);
            cv::Mat big_depth(new_height, new_width, CV_16UC1);
            big_depth *= 0;
            const float scaler_orgX_to_bigX = (1.0f * new_width) / depth.cols;
            const float scaler_orgY_to_bigY = (1.0f * new_height) / depth.rows;
            const float scaler_bigX_to_orgX = 1.0f / scaler_orgX_to_bigX;
            const float scaler_bigY_to_orgY = 1.0f / scaler_orgY_to_bigY;

            const float newfx = K(0, 0) * scaler_orgX_to_bigX;
            const float newcx = K(0, 2) * scaler_orgX_to_bigX;
            const float newfy = K(1, 1) * scaler_orgY_to_bigY;
            const float newcy = K(1, 2) * scaler_orgY_to_bigY;

            int depth_counter = 0;

            for (int y = 0; y < big_depth.rows; y++)
            {
                for (int x = 0; x < big_depth.cols; x++)
                {
                    const float x0 = scaler_bigX_to_orgX * x;
                    const float y0 = scaler_bigX_to_orgX * y;

                    int xx0 = std::lround(x0);
                    int yy0 = std::lround(y0);
                    int positive_depth_count = 0;
                    float depth_avg = 0;
                    for (int dx = -1; dx < 2; dx++)
                    {
                        for (int dy = -1; dy < 2; dy++)
                        {
                            const int nx = dx + xx0;
                            const int ny = dy + yy0;
                            if (nx < 0 || nx >= depth.cols ||
                                ny < 0 || ny >= depth.rows ||
                                depth.at<uint16_t>(ny, nx) == 0)
                            {
                                continue;
                            }
                            depth_avg += 1.0f * depth.at<uint16_t>(ny, nx);
                            positive_depth_count++;
                        }
                    }
                    if (positive_depth_count > 0)
                    {
                        big_depth.at<uint16_t>(y, x) = std::lround(depth_avg / positive_depth_count);
                    }
                }
            }
            return big_depth;
        }

    } // namespace pcl_filters
} // namespace vxs_sensor_ros1