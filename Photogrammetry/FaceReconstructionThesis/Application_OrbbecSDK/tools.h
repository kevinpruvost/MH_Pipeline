#pragma once

#include <iostream>
#include <iomanip>
#include <filesystem>
#include <chrono>
#include <sstream>

#include "libobsensor/ObSensor.hpp"
// #include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#include <feather/utils.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// #include <windows.h>
// #include <shlobj.h>
#include <iostream>
#include <string>

std::string getCurrentDateTime();
std::vector<OBColorPoint> getRGBPointsFromFrame(std::shared_ptr<ob::Frame> frame);
void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string dirName, std::string fileName);
void saveRGBPointsToPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud, std::string dirName, std::string fileName);
std::string askForDirectory(const std::string& title);
std::vector<std::string> getFilesInDirectory(const std::string& directory, const std::string& extension);

class IMU_PoseEstimator
{
public:
    IMU_PoseEstimator();

    void Start();
    void GiveGyroscopeData(const OBGyroValue& gyro, uint64_t timestamp);
    void GiveAccelerometerData(const OBAccelValue& accel, uint64_t timestamp);
    Eigen::Quaternionf GetOrientation() const { return currentOrientation; }
    Eigen::Vector3f GetPosition() const { return currentPosition; }
    void SetGyroBias(const Eigen::Vector3f& bias) { gyroBias = bias; }

private:
    void UpdateOrientationFromGyro(const OBGyroValue& gyro, uint64_t timestamp);
    void UpdateOrientationFromAccel(const OBAccelValue& accel);
    Eigen::Quaternionf GetAccelOrientation(const Eigen::Vector3f& accel);
    void UpdatePosition(const OBAccelValue& accel, uint64_t timestamp);

    Eigen::Quaternionf currentOrientation;
    Eigen::Vector3f currentPosition;
    Eigen::Vector3f currentVelocity;
    Eigen::Vector3f gyroBias;
    Eigen::Vector3f gravity;
    bool __started;
    uint64_t lastGyroTimestamp;
    uint64_t lastAccelTimestamp;
};

class PointCloudVisualizer
{
public:
    PointCloudVisualizer();

    void UpdatePointCloudRealtime(const std::shared_ptr<ob::Frame> & frame);
    void UpdatePointCloudRealtime(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void UpdatePointCloudReconstruction(const std::shared_ptr<ob::Frame>& frame);
    void UpdatePointCloudReconstruction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    void SetKeyboardCallback(std::function<void(const pcl::visualization::KeyboardEvent&)> callback);
    void SetMouseCallback(std::function<void(const pcl::visualization::MouseEvent&)> callback);
    void SetIMUPoseEstimator(IMU_PoseEstimator* imuPoseEstimator);

    void UpdatePositionText(const Eigen::Vector3f& position);
    void UpdateOrientationText(const Eigen::Quaternionf& orientation);
    void UpdateGyroscopeText(const OBGyroValue& gyro);
    void UpdateAccelerometerText(const OBAccelValue& accel);

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr __cloudRealtime;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr __cloudReconstruction;
    pcl::visualization::PCLVisualizer::Ptr __viewer;

    IMU_PoseEstimator* __imuPoseEstimator;

    int __realtimeViewport;
    int __reconstructionViewport;

    bool __shouldQuit;
};

#include <chrono>

class Timer {
public:
    Timer(const std::string & name);

    void reset();

    double elapsedMilliseconds() const;
    double elapsedSeconds() const;
    void PrintTimePassed();

public:
    std::string __name;
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    int checkpoint;
};