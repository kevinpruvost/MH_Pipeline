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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>

#include <thread>

class PointCloudConstructor
{
public:
    PointCloudConstructor();
    ~PointCloudConstructor();

    void AddImage(const std::shared_ptr<ob::ColorFrame> & rgbImage, const std::shared_ptr<ob::DepthFrame> & depthImage);
    void AddImageMultiThread(const std::shared_ptr<ob::ColorFrame> & rgbImage, const std::shared_ptr<ob::DepthFrame> & depthImage);

    void SaveImageMultiThread(std::shared_ptr<cv::Mat> undistortedRGB, std::shared_ptr<cv::Mat> undistortedDepth, std::shared_ptr<cv::Mat> filteredDepth);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() const;
    void SetCameraParameters(const OBCameraParam& params, int colorWidth, int colorHeight, int depthWidth, int depthHeight);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CreatePointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& camera_matrix, float scale);

    void ProcessFrames(const std::shared_ptr<ob::ColorFrame>& rgbImage, const std::shared_ptr<ob::DepthFrame>& depthImage,
        cv::Mat & undistortedRGB, cv::Mat & undistortedDepth, cv::Mat & filteredDepth);

    const cv::Mat & GetRGBCameraMatrix() const { return __rgbIntrinsics; }
    const cv::Mat & GetDepthCameraMatrix() const { return __depthIntrinsics; }
    void FinishAndSave(const std::string& string);
    void lockPointCloud();
    void unlockPointCloud();
    void KillThreads();
    void SetDirectory(const std::string& string);

    void RenderFromSavedImages();

private:
    cv::Mat UndistortRGBImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion);
    cv::Mat UndistortDepthImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion);
    std::vector<cv::DMatch> matchFeatures(const cv::Mat& img1, const cv::Mat& img2,
        std::vector<cv::KeyPoint>& keypoints1,
        std::vector<cv::KeyPoint>& keypoints2);
    Eigen::Matrix4d estimatePose(const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        const std::vector<cv::DMatch>& matches,
        const cv::Mat& depth1, const cv::Mat& depth2,
        const cv::Mat& camera_matrix);

private:
    std::mutex __mergedPointCloudMutex, __transformMutex;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr __mergedPointCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> __pointClouds;
    Eigen::Matrix4d __transformMatrix;

    std::vector<std::shared_ptr<cv::Mat>> __rgbImages, __depthImages, __filteredDepthImages;
    cv::Mat __rgbIntrinsics, __depthIntrinsics;
    cv::Mat __rgbDistortion, __depthDistortion;
    cv::Mat __rgbToDepthExtrinsics;
    bool __isMirrored;

    cv::Mat __undistortRGBMap1, __undistortRGBMap2;
    cv::Mat __undistortDepthMap1, __undistortDepthMap2;

    cv::Mat __add_image_undistortedRGB, __add_image_undistortedDepth, __add_image_filteredDepth;
    float __scale;

    pcl::visualization::PCLVisualizer::Ptr __cameraViewer;

    std::vector<std::unique_ptr<std::thread>> __threads;
    std::unique_ptr<std::thread> __finishingThread;

    double __depthToRgbRatioX;
    double __depthToRgbRatioY;

    bool killThreads = false;

    std::unique_ptr<std::thread> __queuingThread = nullptr;
    std::queue<std::shared_ptr<ob::ColorFrame>> __rgbFrames;
    std::queue<std::shared_ptr<ob::DepthFrame>> __depthFrames;
    int __lastThreadFinished = 0;

    std::string __dirName;
};