#include "PointCloudConstruction.h"

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
    int colorWidth = 1920, colorHeight = 0;
    try {
        // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            colorProfile = colorProfiles->getVideoStreamProfile(colorWidth, colorHeight, OB_FORMAT_RGB);
        }
        config->enableStream(colorProfile);
        colorWidth = colorProfile->width();
        colorHeight = colorProfile->height();
    }
    catch (ob::Error& e) {
        config->setAlignMode(ALIGN_DISABLE);
        std::cerr << "Current device is not support color sensor!" << std::endl;
    }

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode                            alignMode = ALIGN_DISABLE;
    int depthWidth = 1280, depthHeight = 0;
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
        std::shared_ptr<ob::VideoStreamProfile> depthProfile;
        try {
            // Select the profile with the same frame rate as color.
            if (colorProfile) {
                //depthProfile = depthProfileList->getProfile(0)->as<ob::VideoStreamProfile>();
                depthProfile = depthProfileList->getVideoStreamProfile(depthWidth, depthHeight, OB_FORMAT_Y16, 0);
            }
        }
        catch (...) {
            depthProfile = nullptr;
        }

        if (!depthProfile) {
            // If no matching profile is found, select the default profile.
            depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
        }
        config->enableStream(depthProfile);
        auto format = depthProfile->format();
        depthWidth = depthProfile->width();
        depthHeight = depthProfile->height();
    }
    config->setAlignMode(alignMode);

    auto device = pipeline.getDevice();

    device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
    device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, 5000);

    device->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 5000);
    device->setIntProperty(OB_PROP_MIN_DEPTH_INT, 10);
    device->setIntProperty(OB_PROP_MAX_DEPTH_INT, 1000);

    device->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PRECISION_1MM);

    // start pipeline with config
    pipeline.start(config);

    //PointCloudVisualizer visualizer;

    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
    auto cameraParam = pipeline.getCameraParam();

    PointCloudConstructor pCConstructor;
    pCConstructor.SetCameraParameters(cameraParam, colorWidth, colorHeight, depthWidth, depthHeight);

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


    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("RGBD PointCloud Reconstruction", 1280, 720);
    std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
    visualizer.AddGeometry(pc);
    visualizer.RegisterAnimationCallback([&](open3d::visualization::Visualizer* vis) {
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
                // Visualize open3d point cloud
                PointCloudFromFrame(pc, frame);
                visualizer.UpdateGeometry(pc);
                //visualizer.UpdatePointCloudRealtime(frame);
            }
            catch (...) {
            }
            if (kbhit()) {
                int key = getch();
                if (key == 27) {
                    return false;
                }
                else if (key == 32) {
                    int count = 0;
                    // Limit up to 10 repetitions
                    while (count++ < 10) {
                        // Wait for a frame of data, the timeout is 100ms
                        auto frameset = pipeline.waitForFrames(100);
                        auto rgbFrame = frameset->colorFrame();
                        auto depthFrame = frameset->depthFrame();
                        if (frameset != nullptr && depthFrame != nullptr && rgbFrame != nullptr)
                        {
                            pCConstructor.AddImage(rgbFrame, depthFrame);
                            open3d::geometry::PointCloud pointCloud = pCConstructor.getPointCloud();
                            // Converting point cloud for viz
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                            for (int i = 0; i < pointCloud.points_.size(); i++) {
                                pclPointCloud->emplace_back(pointCloud.points_[i].x(), pointCloud.points_[i].y(), pointCloud.points_[i].z(),
                                    pointCloud.colors_[i].x(), pointCloud.colors_[i].y(), pointCloud.colors_[i].z());
                            }

                            static int countFile = 0;
                            std::string name = "RGBDPoints_test" + std::to_string(countFile++) + ".ply";
                            std::cout << "Save RGBD PointCloud ply file..." << std::endl;
                            saveRGBPointsToPly(pclPointCloud, dirName, name.c_str());
                            auto depthValueScale = frameset->depthFrame()->getValueScale();
                            std::cout << dirName << "/" << name << " Saved" << std::endl;
                            break;
                        }
                        else {
                            std::cout << "Get color frame or depth frame failed!" << std::endl;
                        }
                    }
                }
            }
        }
        return true;
    });
    visualizer.Run();
    int count = 0;
    bool shouldQuit = false;
    bool shouldSave = false;
    //visualizer.SetKeyboardCallback([&](const pcl::visualization::KeyboardEvent & event) {
    //    if (event.keyDown()) {
    //        if (event.getKeyCode() == 27) {
    //            shouldQuit = true;
    //        }
    //        else if (event.getKeyCode() == 32) {
    //            int count = 0;
    //            // Limit up to 10 repetitions
    //            while (count++ < 10) {
    //                // Wait for a frame of data, the timeout is 100ms
    //                auto frameset = pipeline.waitForFrames(100);
    //                auto rgbFrame = frameset->colorFrame();
    //                auto depthFrame = frameset->depthFrame();
    //                if (frameset != nullptr && depthFrame != nullptr && rgbFrame != nullptr)
    //                {
    //                    pCConstructor.AddImage(rgbFrame, depthFrame);
    //                    open3d::geometry::PointCloud pointCloud = pCConstructor.getPointCloud();
    //                    // Converting point cloud for viz
    //                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //                    for (int i = 0; i < pointCloud.points_.size(); i++) {
    //                        pclPointCloud->emplace_back(pointCloud.points_[i].x(), pointCloud.points_[i].y(), pointCloud.points_[i].z(),
    //                            pointCloud.colors_[i].x(), pointCloud.colors_[i].y(), pointCloud.colors_[i].z());
    //                    }


    //                    visualizer.UpdatePointCloudReconstruction(pclPointCloud);

    //                    static int countFile = 0;
    //                    std::string name = "RGBDPoints_test" + std::to_string(countFile++) + ".ply";
    //                    std::cout << "Save RGBD PointCloud ply file..." << std::endl;
    //                    saveRGBPointsToPly(pclPointCloud, dirName, name.c_str());
    //                    auto depthValueScale = frameset->depthFrame()->getValueScale();
    //                    std::cout << dirName << "/" << name << " Saved" << std::endl;
    //                    break;
    //                }
    //                else {
    //                    std::cout << "Get color frame or depth frame failed!" << std::endl;
    //                }
    //            }
    //        }
    //    }
    //});

    //while (true) {
    //    if (shouldQuit) break;
    //    auto frameset = pipeline.waitForFrames(1000 / 30);
    //    // Display Realtime point cloud
    //    if (frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
    //        // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
    //        // millimeter)
    //        try {
    //            auto depthValueScale = frameset->depthFrame()->getValueScale();
    //            pointCloud.setPositionDataScaled(depthValueScale);
    //            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
    //            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
    //            // Visualize open3d point cloud
    //            PointCloudFromFrame(pc, frame);
    //            visualizer.UpdateGeometry(pc);
    //            //visualizer.UpdatePointCloudRealtime(frame);
    //        }
    //        catch (...) {
    //        }
    //        int key = getch();
    //        if (key == 27) {
    //            shouldQuit = true;
    //        }
    //        else if (key == 32) {
    //            int count = 0;
    //            // Limit up to 10 repetitions
    //            while (count++ < 10) {
    //                // Wait for a frame of data, the timeout is 100ms
    //                auto frameset = pipeline.waitForFrames(100);
    //                auto rgbFrame = frameset->colorFrame();
    //                auto depthFrame = frameset->depthFrame();
    //                if (frameset != nullptr && depthFrame != nullptr && rgbFrame != nullptr)
    //                {
    //                    pCConstructor.AddImage(rgbFrame, depthFrame);
    //                    open3d::geometry::PointCloud pointCloud = pCConstructor.getPointCloud();
    //                    // Converting point cloud for viz
    //                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //                    for (int i = 0; i < pointCloud.points_.size(); i++) {
    //                        pclPointCloud->emplace_back(pointCloud.points_[i].x(), pointCloud.points_[i].y(), pointCloud.points_[i].z(),
    //                            pointCloud.colors_[i].x(), pointCloud.colors_[i].y(), pointCloud.colors_[i].z());
    //                    }

    //                    static int countFile = 0;
    //                    std::string name = "RGBDPoints_test" + std::to_string(countFile++) + ".ply";
    //                    std::cout << "Save RGBD PointCloud ply file..." << std::endl;
    //                    saveRGBPointsToPly(pclPointCloud, dirName, name.c_str());
    //                    auto depthValueScale = frameset->depthFrame()->getValueScale();
    //                    std::cout << dirName << "/" << name << " Saved" << std::endl;
    //                    break;
    //                }
    //                else {
    //                    std::cout << "Get color frame or depth frame failed!" << std::endl;
    //                }
    //            }
    //        }
    //    }
    //}
    // stop the pipeline
    pipeline.stop();

    return 0;
}
catch (ob::Error& e)
{
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
