#pragma once

#include "../Application_OrbbecSDK/tools.h"
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

class PointCloudConstructor
{
public:
    PointCloudConstructor();
    ~PointCloudConstructor() = default;

    void AddImage(const std::shared_ptr<ob::ColorFrame> & rgbImage, const std::shared_ptr<ob::DepthFrame> & depthImage);
    void AddImageAndBuildPointCloud(const std::shared_ptr<ob::ColorFrame>& rgbImage, const std::shared_ptr<ob::DepthFrame>& depthImage);

    open3d::geometry::PointCloud getPointCloud() const;
    void SetCameraParameters(const OBCameraParam& params, int colorWidth, int colorHeight, int depthWidth, int depthHeight);

    open3d::geometry::PointCloud CreatePointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& camera_matrix, float scale);

private:
    cv::Mat UndistortRGBImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion);
    cv::Mat UndistortDepthImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion);
    std::vector<cv::DMatch> matchFeatures(const cv::Mat& img1, const cv::Mat& img2,
        std::vector<cv::KeyPoint>& keypoints1,
        std::vector<cv::KeyPoint>& keypoints2);
    Eigen::Matrix4f estimatePose(const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        const std::vector<cv::DMatch>& matches,
        const cv::Mat& depth1, const cv::Mat& depth2,
        const cv::Mat& camera_matrix);
    open3d::geometry::PointCloud mergePointClouds(
        const std::vector<open3d::geometry::PointCloud>& clouds,
        const std::vector<Eigen::Matrix4d>& transforms);

private:
    open3d::geometry::PointCloud __pointCloud;
    Eigen::Matrix4d __transformMatrix;

    std::vector<cv::Mat> __rgbImages, __depthImages, __depthImagesFiltered;
    cv::Mat __rgbIntrinsics, __depthIntrinsics;
    cv::Mat __rgbDistortion, __depthDistortion;
    cv::Mat __rgbToDepthExtrinsics;
    bool __isMirrored;

    cv::Mat __undistortRGBMap1, __undistortRGBMap2;
    cv::Mat __undistortDepthMap1, __undistortDepthMap2;

    cv::Mat __add_image_undistortedRGB, __add_image_undistortedDepth, __add_image_filteredDepth;
    float __scale;

    std::vector<open3d::geometry::PointCloud> __pointClouds;
    std::vector<Eigen::Matrix4d> __transforms;
};

void PointCloudFromFrame(std::shared_ptr<open3d::geometry::PointCloud>& ptr, const std::shared_ptr<ob::Frame>& frame);