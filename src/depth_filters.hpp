/**
 * @file depth_filters.hpp
 * @author George terzakis (george.terzakis@voxelsensors.com)
 * @brief Depth image filetrs for densofication, resizing and cleaning
 * @date 2026-01-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef DEPTH_FILTERS_HPP_
#define DEPTH_FILTERS_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <math.h>

namespace vxs_ros1
{
    namespace pcl_filters
    {
        //! Resize depth image to smaller scale (with a pointcloud)
        cv::Mat DownsizeDepthImage(               //
            const int new_width,                  //
            const int new_height,                 //
            const cv::Mat &depth,                 //
            const cv::Matx<float, 3, 3> &K,       //
            std::vector<cv::Vec3f> &new_points3d, //
            const float &depth_scaler = 0.001     // default is 1 mm
        );

        //! Resize depth image to smaller scale (without returning a pointcloud)
        cv::Mat DownsizeDepthImage( //
            const int new_width,    //
            const int new_height,   //
            const cv::Mat &depth,   //
            const cv::Matx<float, 3, 3> &K);

        //! Resize depth image to a bigger one using nearest **no-zero-depth** neighbor
        cv::Mat UpsizeDepthImage( //
            const int new_width,  //
            const int new_height, //
            const cv::Mat &depth, //
            const cv::Matx<float, 3, 3> &K);

        /**
         * @brief Align depth to RGB. NOTE: Assumes undistorted input (on the RGB side mainly)
         *
         * @tparam P Precision
         * @param depth Depth image
         * @param rgb RGB image
         * @param vxK depth intrinsics
         * @param rgbK RGB intrinsics
         * @param R_cs Rotation depth-to-rgb
         * @param t_cs Translation depth-to-rgb
         * @return cv::Mat Aligned depth image
         */
        template <typename P>
        cv::Mat AlignDepthToRGB(           //
            const cv::Mat &depth,          //
            const cv::Matx<P, 3, 3> &vxK,  //
            const cv::Matx<P, 3, 3> &rgbK, //

            const cv::Matx<P, 3, 3> &R_cs, const cv::Vec<P, 3> &t_cs)
        {
            cv::Mat aligned_depth(depth.size(), CV_16U);

            aligned_depth *= 0;
            const cv::Matx<double, 3, 3> R = cv::Matx<double, 3, 3>(R_cs);
            const cv::Vec<double, 3> t = cv::Vec<double, 3>(t_cs);

            for (size_t r = 0; r < depth.rows; r++)
            {
                for (size_t c = 0; c < depth.cols; c++)
                {
                    const double x0 = c, y0 = r;
                    cv::Vec<double, 3> M = cv::Vec<double, 3>{(x0 - vxK(0, 2)) / vxK(0, 0), (y0 - vxK(1, 2)) / vxK(1, 1), 1};
                    const uint16_t d = depth.at<uint16_t>(r, c);
                    M *= static_cast<double>(d);
                    // Now project onto the RGB image
                    const cv::Vec<double, 3> Mc = R * M + t;

                    if (Mc[2] < 1e-4)
                    {
                        continue;
                    }
                    const double iZc = 1.0 / Mc[2];
                    const int x = std::lround(rgbK(0, 0) * Mc[0] * iZc + rgbK(0, 2));
                    const int y = std::lround(rgbK(1, 1) * Mc[1] * iZc + rgbK(1, 2));
                    if (x < 0 || x > aligned_depth.cols - 1 || y < 0 || y > aligned_depth.rows - 1)
                    {
                        continue;
                    }
                    aligned_depth.at<uint16_t>(y, x) = d;
                }
            }
            return aligned_depth;
        }
    } // namespace pcl_filters
} // namespace vxs_ros1

#endif
