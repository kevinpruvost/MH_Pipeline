#include "tools.h"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

std::mutex printerMutex;

int main(int argc, char** argv)
try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    // create pipeline
    ob::Pipeline pipeline;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
            colorProfile = profile->as<ob::VideoStreamProfile>();
        }
        config->enableStream(colorProfile);
    }
    catch (ob::Error& e) {
        config->setAlignMode(ALIGN_DISABLE);
        std::cerr << "Current device is not support color sensor!" << std::endl;
    }

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode                            alignMode = ALIGN_DISABLE;
    if (colorProfile) {
        // Try find supported depth to color align hardware mode profile
        depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
        if (depthProfileList->count() > 0) {
            alignMode = ALIGN_D2C_HW_MODE;
        }
        else {
            // Try find supported depth to color align software mode profile
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
            if (depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_SW_MODE;
            }
        }
    }
    else {
        depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    }

    if (depthProfileList->count() > 0) {
        std::shared_ptr<ob::StreamProfile> depthProfile;
        try {
            // Select the profile with the same frame rate as color.
            if (colorProfile) {
                depthProfile = depthProfileList->getVideoStreamProfile(1280, OB_HEIGHT_ANY, OB_FORMAT_ANY, 0);
            }
        }
        catch (...) {
            depthProfile = nullptr;
        }

        if (!depthProfile) {
            // If no matching profile is found, select the default profile.
            depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
        }
        config->enableStream(depthProfile);
    }
    config->setAlignMode(alignMode);

    auto device = pipeline.getDevice();

    device->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 2000);
    device->setIntProperty(OB_PROP_MIN_DEPTH_INT, 100);
    device->setIntProperty(OB_PROP_MAX_DEPTH_INT, 500);

    device->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PRECISION_0MM05);

    // start pipeline with config
    pipeline.start(config);

    IMU_PoseEstimator imuPoseEstimator;
    PointCloudVisualizer visualizer;
    std::shared_ptr<ob::Sensor> gyroSensor = nullptr;
    std::shared_ptr<ob::Sensor> accelSensor = nullptr;
    int imuFrequency = 25;
    try {
        // Get Gyroscope Sensor
        gyroSensor = device->getSensorList()->getSensor(OB_SENSOR_GYRO);
        if (gyroSensor) {
            // Get configuration list
            auto profiles = gyroSensor->getStreamProfileList();
            // Select the first profile to open stream
            auto profile = profiles->getGyroStreamProfile(OBGyroFullScaleRange::OB_GYRO_FS_1000dps, OBGyroSampleRate::OB_SAMPLE_RATE_100_HZ);
            gyroSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(printerMutex);
                auto                         timeStamp = frame->timeStamp();
                auto                         index = frame->index();
                auto                         gyroFrame = frame->as<ob::GyroFrame>();
                auto value = gyroFrame->value();
                // if (abs(value.x) < 0.02) value.x = 0.0;
                // if (abs(value.y) < 0.02) value.y = 0.0;
                // if (abs(value.z) < 0.02) value.z = 0.0;
                imuPoseEstimator.GiveGyroscopeData(value, timeStamp);
                if (gyroFrame != nullptr && (index % 50) == 2) {  //( timeStamp % 500 ) < 2: Reduce printing frequency
                    visualizer.UpdateGyroscopeText(value);
                }
                });
        }
        else {
            std::cout << "get gyro Sensor failed ! " << std::endl;
        }
    }
    catch (ob::Error& e) {
        std::cerr << "current device does not support imu!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get Acceleration Sensor
    accelSensor = device->getSensorList()->getSensor(OB_SENSOR_ACCEL);
    if (accelSensor) {
        // Get configuration list
        auto profiles = accelSensor->getStreamProfileList();
        // Select the first profile to open stream
        auto profile = profiles->getAccelStreamProfile(OBAccelFullScaleRange::OB_ACCEL_FS_4g, OBAccelSampleRate::OB_SAMPLE_RATE_100_HZ);
        accelSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame) {
            std::unique_lock<std::mutex> lk(printerMutex);
            auto                         timeStamp = frame->timeStamp();
            auto                         index = frame->index();
            auto                         accelFrame = frame->as<ob::AccelFrame>();
            auto value = accelFrame->value();
            imuPoseEstimator.GiveAccelerometerData(value, timeStamp);
            if (accelFrame != nullptr && (index % 50) == 0) {
                visualizer.UpdateAccelerometerText(value);
                visualizer.UpdatePositionText(imuPoseEstimator.GetPosition());
                visualizer.UpdateOrientationText(imuPoseEstimator.GetOrientation());
                auto pos = imuPoseEstimator.GetPosition();
                auto orientation = imuPoseEstimator.GetOrientation();
                //printf("Position: [%.3f,%.3f,%.3f]\n", pos.x(), pos.y(), pos.z());
                //printf("Orientation: [%.3f,%.3f,%.3f,%.3f]\n", orientation.x(), orientation.y(), orientation.z(), orientation.w());
            }
            });
    }
    else {
        std::cout << "get Accel Sensor failed ! " << std::endl;
    }

    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
    auto cameraParam = pipeline.getCameraParam();

    pointCloud.setCameraParam(cameraParam);

    // operation prompt
    std::cout << "Press Space to create RGBD PointCloud and save to ply file! " << std::endl;
    std::cout << "Press ESC to exit! " << std::endl;

    // Dirname based on datetime
    std::string dirName = "RGBDPoints_" + getCurrentDateTime();
    // Create dir if not exist
    if (!std::filesystem::exists(dirName)) {
        std::filesystem::create_directory(dirName);
    }

    int count = 0;
    bool shouldQuit = false;
    bool shouldSave = false;
    visualizer.SetIMUPoseEstimator(&imuPoseEstimator);
    visualizer.SetKeyboardCallback([&](const pcl::visualization::KeyboardEvent & event) {
        if (event.getKeyCode() == 27) {
            shouldQuit = true;
        } else if (event.getKeyCode() == 32) {
            int count = 0;
            // Limit up to 10 repetitions
            while (count++ < 10) {
                // Wait for a frame of data, the timeout is 100ms
                auto frameset = pipeline.waitForFrames(100);
                if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
                    // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                    // millimeter)
                    auto depthValueScale = frameset->depthFrame()->getValueScale();
                    pointCloud.setPositionDataScaled(depthValueScale);
                    try {
                        // Generate a colored point cloud and save it
                        std::cout << "Save RGBD PointCloud ply file..." << std::endl;
                        pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                        std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                        static int countFile = 0;
                        std::string name = "RGBDPoints_test" + std::to_string(countFile++) + ".ply";
                        visualizer.UpdatePointCloudReconstruction(frame);
                        saveRGBPointsToPly(frame, dirName, name.c_str());
                        std::cout << dirName << "/" << name << " Saved" << std::endl;
                    }
                    catch (std::exception& e) {
                        std::cout << "Get point cloud failed" << std::endl;
                    }
                    break;
                }
                else {
                    std::cout << "Get color frame or depth frame failed!" << std::endl;
                }
            }
        }
    });
    while (true) {
        if (shouldQuit) break;
        auto frameset = pipeline.waitForFrames(1000 / 30);
        // Display Realtime point cloud
        if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
            // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
            // millimeter)
            try {
                auto depthValueScale = frameset->depthFrame()->getValueScale();
                pointCloud.setPositionDataScaled(depthValueScale);
                pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                visualizer.UpdatePointCloudRealtime(frame);
                //visualizer.UpdatePointCloudReconstruction(frame);
            } catch (...) {
            }
        }
    }
    // stop the pipeline
    pipeline.stop();

    // turn off the flow
    if (gyroSensor) {
        gyroSensor->stop();
    }
    if (accelSensor) {
        accelSensor->stop();
    }

    return 0;
}
catch (ob::Error& e)
{
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
