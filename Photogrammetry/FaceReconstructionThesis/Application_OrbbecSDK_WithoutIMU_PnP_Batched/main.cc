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
    } catch (ob::Error& e) {
        config->setAlignMode(ALIGN_DISABLE);
        std::cerr << "Current device is not support color sensor!" << std::endl;
    }

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode alignMode = ALIGN_DISABLE;
    int depthWidth = 1280, depthHeight = 0;
    if (colorProfile) {
        // Try find supported depth to color align hardware mode profile
        depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
        if (depthProfileList->count() > 0) {
            alignMode = ALIGN_D2C_HW_MODE;
        } else {
            // Try find supported depth to color align software mode profile
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
            if (depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_SW_MODE;
            }
        }
    } else {
        depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    }

    std::shared_ptr<ob::VideoStreamProfile> depthProfile;
    if (depthProfileList->count() > 0) {
        try {
            // Select the profile with the same frame rate as color.
            if (colorProfile) {
                //depthProfile = depthProfileList->getProfile(0)->as<ob::VideoStreamProfile>();
                depthProfile = depthProfileList->getVideoStreamProfile(depthWidth, depthHeight, OB_FORMAT_Y16, 30);
            }
        } catch (...) {
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

    auto depthFps = depthProfile->fps();

    std::cout << depthFps << "fps " << depthWidth << "x" << depthHeight << " depth stream is enabled." << std::endl;

    auto device = pipeline.getDevice();

    // Query the current camera depth mode
    auto curDepthMode = device->getCurrentDepthWorkMode();
    // Get the list of camera depth modes
    auto depthModeList = device->getDepthWorkModeList();
    std::cout << "depthModeList size: " << depthModeList->count() << std::endl;
    for(uint32_t i = 0; i < depthModeList->count(); i++) {
        std::cout << "depthModeList[" << i << "]: " << (*depthModeList)[i].name;
        if(strcmp(curDepthMode.name, (*depthModeList)[i].name) == 0) {
            std::cout << "  (Current WorkMode)";
        }

        std::cout << std::endl;
    }

    //device->switchDepthWorkMode((*depthModeList)[0].name);
    //std::cout << "Switched to " << device->getCurrentDepthWorkMode().name << std::endl;

    config->setAlignMode(alignMode);

    device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
    device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, 15000);
    device->setIntProperty(OB_PROP_COLOR_GAIN_INT, 1);
    device->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, 255);

    device->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, false);
    device->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 5000);
    device->setIntProperty(OB_PROP_MIN_DEPTH_INT, 10);
    device->setIntProperty(OB_PROP_MAX_DEPTH_INT, 10000);
    //device->setIntProperty(OB_PROP_MAX_DEPTH_INT, 500);

    device->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PRECISION_1MM);

    // start pipeline with config
    pipeline.start(config);

    PointCloudVisualizer visualizer;

    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
    auto cameraParam = pipeline.getCameraParam();

    std::unique_ptr<PointCloudConstructor> pCConstructor(new PointCloudConstructor);
    pCConstructor->SetCameraParameters(cameraParam, colorWidth, colorHeight, depthWidth, depthHeight);

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
    bool scanStarted = false;
    bool renderingReconstruction = false;
    Timer timerScan("scan");
    visualizer.SetKeyboardCallback([&](const pcl::visualization::KeyboardEvent & event) {
        if (event.keyDown()) {
            if (event.getKeyCode() == 27) {
                shouldQuit = true;
            }
            else if (event.getKeyCode() == 32) {
                scanStarted = !scanStarted;
                if (!scanStarted)
                {
                    pCConstructor->RenderFromSavedImages();
                    renderingReconstruction = true;
                }
            }
        }
    });
    visualizer.SetMouseCallback([&](const pcl::visualization::MouseEvent & event) {
        if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::MiddleButton)
        {
            scanStarted = !scanStarted;
            if (!scanStarted)
            {
                pCConstructor->RenderFromSavedImages();
                renderingReconstruction = true;
            }
        }
    });
    pCConstructor->SetDirectory(dirName);
    std::unique_ptr<std::thread> updateThread(nullptr);
    while (true) {
        if (shouldQuit) break;
        auto frameset = pipeline.waitForFrames(1);
        if (frameset == nullptr) {
            continue;
        }
        auto rgbFrame = frameset->colorFrame();
        auto depthFrame = frameset->depthFrame();
        // Display Realtime point cloud
        if (depthFrame != nullptr && rgbFrame != nullptr) {
            // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
            // millimeter)
            try {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame;
                std::shared_ptr<cv::Mat> undistortedRGB, undistortedDepth, filteredDepth;
                undistortedRGB = std::make_shared<cv::Mat>();
                undistortedDepth = std::make_shared<cv::Mat>();
                filteredDepth = std::make_shared<cv::Mat>();
                pCConstructor->ProcessFrames(rgbFrame, depthFrame, *undistortedRGB, *undistortedDepth, *filteredDepth);
                frame = pCConstructor->CreatePointCloud(*undistortedRGB, *filteredDepth, pCConstructor->GetDepthCameraMatrix(), 1.0f);

                //timer.PrintTimePassed();
                visualizer.UpdatePointCloudRealtime(frame);
                //timer.PrintTimePassed();

                if (scanStarted)
                {
                    //if (timerScan.elapsedMilliseconds() >= 1000.0f / 20.0f)
                    {
                        timerScan.reset();
                        int count = 0;
                        pCConstructor->SaveImageMultiThread(undistortedRGB, undistortedDepth, filteredDepth);
                    }
                }
                if (renderingReconstruction)
                {
                    static bool updated = true;
                    if (updated)
                    {
                        updated = false;
                        if (updateThread) updateThread->join();
                        updateThread.reset(new std::thread([&]() {
                            pCConstructor->lockPointCloud();
                            auto pointCloud = pCConstructor->getPointCloud();
                            visualizer.UpdatePointCloudReconstruction(pointCloud);
                            pCConstructor->unlockPointCloud();
                            updated = true;
                        }));
                    }
                }
            } catch (...) {
            }
        }
    }
    // stop the pipeline
    pipeline.stop();
    pCConstructor->KillThreads();
    return 0;
}
catch (ob::Error& e)
{
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
