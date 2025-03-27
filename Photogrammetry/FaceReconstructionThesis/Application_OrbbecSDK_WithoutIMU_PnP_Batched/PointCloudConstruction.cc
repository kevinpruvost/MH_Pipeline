#include "PointCloudConstruction.h"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

PointCloudConstructor::PointCloudConstructor()
    : __mergedPointCloud()
    , __transformMatrix(Eigen::Matrix4d::Identity())
//    , __cameraViewer(new pcl::visualization::PCLVisualizer("Camera Pose Viewer"))
{
    //Eigen::Affine3f affine_transform;
    //affine_transform.matrix() = __transformMatrix;
    //__cameraViewer->addCoordinateSystem(1.0f, affine_transform, "camera_pose");
#ifdef _DEBUG
    //cv::namedWindow("TestDepth", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("UnfilteredTestDepth", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("TestRGB", cv::WINDOW_AUTOSIZE);

    //// Set size to half the screen
    //cv::resizeWindow("TestDepth", 640, 480);
    //cv::resizeWindow("TestRGB", 640, 480);
#endif
}

PointCloudConstructor::~PointCloudConstructor()
{
    KillThreads();
}

inline void DebugLog(const std::string& message)
{
#ifdef _DEBUG
    std::cout << message << std::endl;
#endif
}

void PointCloudConstructor::AddImage(const std::shared_ptr<ob::ColorFrame>& rgbFrame, const std::shared_ptr<ob::DepthFrame>& depthFrame)
{
    // Check number of CPU cores to calculate max number of threads
    Timer timer("AddImage");
    // Get RGB Image
    cv::Mat rawRgbMat(rgbFrame->height(), rgbFrame->width(), CV_8UC3, rgbFrame->data());
    cv::Mat rawDepthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    ProcessFrames(rgbFrame, depthFrame, __add_image_undistortedRGB, __add_image_undistortedDepth, __add_image_filteredDepth);

    timer.PrintTimePassed();
    // Write point cloud to file
    pcl::PLYWriter writer;

#ifdef _DEBUG
    // cv::Mat depth = __add_image_filteredDepth.clone();
    // depth.convertTo(depth, CV_8UC1, 255.0 / (maxValue - minValue), -255.0 * minValue / (maxValue - minValue));
    // cv::Mat rgb;
    // cv::cvtColor(__add_image_undistortedRGB, rgb, cv::COLOR_RGB2BGR);
    // //cv::imshow("TestRGB", rgb);
    // //cv::imshow("TestDepth", depth);
    // //cv::imshow("UnfilteredTestDepth", undistorted_depth);
    //
    // static int counter = 0;
    // cv::String rgbPath = cv::format("rgb.png");
    // cv::imwrite(rgbPath, __add_image_undistortedRGB);
    // cv::Mat undistorted_depth2;
    // __add_image_filteredDepth.convertTo(undistorted_depth2, CV_8UC1, 255.0 / (maxValue - minValue), -255.0 * minValue / (maxValue - minValue));
    // cv::String depthPath = cv::format("depth.png");
    // cv::imwrite(depthPath, undistorted_depth2);
    // cv::String unfilteredDepthPath = cv::format("unfiltered_depth.png");
    // cv::Mat unfilteredDepth;
    // __add_image_undistortedDepth.convertTo(unfilteredDepth, CV_8UC1, 255.0 / (maxValueLimitless - minValue), -255.0 * minValue / (maxValueLimitless - minValue));
    // cv::imwrite(unfilteredDepthPath, unfilteredDepth);
#endif

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = CreatePointCloud(__add_image_undistortedRGB, __add_image_filteredDepth, __depthIntrinsics, __scale);
    timer.PrintTimePassed();
    if (!__rgbImages.empty()) {
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> matches = matchFeatures(*__rgbImages.back(), __add_image_undistortedRGB, keypoints1, keypoints2);
        timer.PrintTimePassed();

#ifdef _DEBUG
        // Visualization of keypoints and matches
        cv::Mat img_keypoints1, img_keypoints2;
        cv::drawKeypoints(__rgbImages.back(), keypoints1, img_keypoints1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(__add_image_undistortedRGB, keypoints2, img_keypoints2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        cv::Mat img_matches;
        cv::drawMatches(__rgbImages.back(), keypoints1, __add_image_undistortedRGB, keypoints2, matches, img_matches);

        // Display the images with keypoints and matches
        //cv::imshow("Previous Frame Keypoints", img_keypoints1);
        //cv::imshow("Current Frame Keypoints", img_keypoints2);
        //cv::imshow("Matches", img_matches);

        // Save the visualization images
        cv::imwrite("previous_frame_keypoints.png", img_keypoints1);
        cv::imwrite("current_frame_keypoints.png", img_keypoints2);
        cv::imwrite("matches.png", img_matches);
#endif

         Eigen::Matrix4d transform = estimatePose(keypoints1, keypoints2, matches, *__depthImages.back(), __add_image_undistortedDepth, __depthIntrinsics);
         
         // __transformMatrix represents the cumulative transformation of all images from the first picture (which is the origin)
         __transformMatrix = transform * __transformMatrix;
         timer.PrintTimePassed();

         // So to transform the point cloud, we need to multiply it by the inverse of the cumulative transformation
         Eigen::Matrix4f transformMatrixFloat = __transformMatrix.cast<float>();
         pcl::transformPointCloud(*cloud, *cloud, transformMatrixFloat.inverse());
         timer.PrintTimePassed();

         // Or we could transform the merged point cloud by the new transformation
         //pcl::transformPointCloud(*__mergedPointCloud, *__mergedPointCloud, transform);

         // Transform the point 

         //transforms.push_back(cumulative_transform);
         
         *__mergedPointCloud += *cloud;
         //   writer.write("merged_pointcloud.ply", *__mergedPointCloud);
    } else {
        __mergedPointCloud = cloud;
    }
    timer.PrintTimePassed();

    static int pcCounter = 0;
    auto name = "pointcloud" + std::to_string(pcCounter++) + ".ply";
    //writer.write(name, *cloud);

    __rgbImages.emplace_back(std::make_shared<cv::Mat>(std::move(__add_image_undistortedRGB)));
    __depthImages.emplace_back(std::make_shared<cv::Mat>(std::move(__add_image_undistortedDepth)));
    __pointClouds.emplace_back(std::move(cloud));
    timer.PrintTimePassed();
}

void PointCloudConstructor::AddImageMultiThread(const std::shared_ptr<ob::ColorFrame>& rgbFrame,
    const std::shared_ptr<ob::DepthFrame>& depthFrame)
{
    static int imageCount = 0;
    static std::mutex queueMutex;
    static std::mutex imageWriteMutex, imageReadMutex;
    printf("More images! %d\n", imageCount++);
    queueMutex.lock();
    __rgbFrames.push(rgbFrame);
    __depthFrames.push(depthFrame);
    queueMutex.unlock();
    if (__queuingThread != nullptr) return;
    __queuingThread.reset(new std::thread([this]() {
        while (!killThreads) {
            printf("Try to lock: %d/n", __rgbFrames.size());
            std::unique_lock<std::mutex> transformLock(__transformMutex);
            std::condition_variable transformCv;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/5));
            while (__rgbFrames.empty() || __depthFrames.empty())
            {
                //printf("Thread %d waiting for transform\n", threadIndex);
                transformCv.wait_for(transformLock, std::chrono::milliseconds(1000/30), [&]() { return __rgbFrames.empty() || __depthFrames.empty(); });
            }
            if (__rgbFrames.empty() || __depthFrames.empty()) continue;
            queueMutex.lock();
            auto rgbFrame = __rgbFrames.front();
            auto depthFrame = __depthFrames.front();
            __rgbFrames.pop();
            __depthFrames.pop();
            queueMutex.unlock();
            // Get RGB Image
            ProcessFrames(rgbFrame, depthFrame, __add_image_undistortedRGB, __add_image_undistortedDepth, __add_image_filteredDepth);

            // TODO: unique ptr
            imageWriteMutex.lock();
            __rgbImages.emplace_back(std::make_shared<cv::Mat>(std::move(__add_image_undistortedRGB)));
            __depthImages.emplace_back(std::make_shared<cv::Mat>(std::move(__add_image_undistortedDepth)));
            __filteredDepthImages.emplace_back(std::make_shared<cv::Mat>(std::move(__add_image_filteredDepth)));
            imageWriteMutex.unlock();

            size_t sizeRGB = __rgbImages.back()->step[0] * __rgbImages.back()->rows;
            size_t sizeDepth = __depthImages.back()->step[0] * __depthImages.back()->rows;
            printf("RGB Image size: %d\n", sizeRGB);
            printf("Depth Image size: %d\n", sizeDepth);

            static int cond1 = 0, cond2 = 0, cond3 = 0;

            __threads.emplace_back(std::make_unique<std::thread>([this](int threadIndex) {
                Timer threadTimer("Thread " + std::to_string(threadIndex));
                    printf("Thread %d hello1\n", threadIndex);
                imageWriteMutex.lock();
                auto add_image_undistortedRGB = __rgbImages[threadIndex];
                auto add_image_undistortedDepth = __depthImages[threadIndex];
                auto add_image_filteredDepth = __filteredDepthImages[threadIndex];
                auto before_add_image_undistortedRGB = threadIndex != 0 ? __rgbImages[threadIndex-1] : nullptr;
                auto before_add_image_undistortedDepth = threadIndex != 0 ? __depthImages[threadIndex-1] : nullptr;
                imageWriteMutex.unlock();
                    printf("Thread %d hello2\n", threadIndex);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = CreatePointCloud(*add_image_undistortedRGB, *add_image_filteredDepth, __depthIntrinsics, __scale);

                if (threadIndex != 0)
                {
                    std::vector<cv::KeyPoint> keypoints1, keypoints2;
                    printf("Thread %d starts matching features\n", threadIndex);
                    std::vector<cv::DMatch> matches = matchFeatures(*before_add_image_undistortedRGB, *add_image_undistortedRGB, keypoints1, keypoints2);
                    //timer.PrintTimePassed();

                    Eigen::Matrix4d transform = estimatePose(keypoints1, keypoints2, matches, *before_add_image_undistortedDepth, *add_image_undistortedDepth, __depthIntrinsics);

                    // __transformMatrix represents the cumulative transformation of all images from the first picture (which is the origin)
                    // Wait for thread(threadIndex-1) to finish using __transformMatrix
                    std::unique_lock transformLock(__transformMutex);
                    std::condition_variable transformCv;
                    while (cond1 != threadIndex)
                    {
                        printf("Thread %d waiting for transform\n", threadIndex);
                        transformCv.wait_for(transformLock, std::chrono::milliseconds(100), [&]() { return cond1 == threadIndex; });
                    }
                    __transformMatrix = transform * __transformMatrix;
                    ++cond1;
                    transformCv.notify_all();
                    //timer.PrintTimePassed();

                    // So to transform the point cloud, we need to multiply it by the inverse of the cumulative transformation
                    Eigen::Matrix4f transformMatrixFloat = __transformMatrix.cast<float>();
                    pcl::transformPointCloud(*cloud, *cloud, transformMatrixFloat.inverse());
                    //timer.PrintTimePassed();

                    std::unique_lock<std::mutex> mergedLock(__mergedPointCloudMutex);
                    std::condition_variable mergedCv;
                    if (cond2 != threadIndex)
                    {
                        //printf("Thread %d waiting for merge\n", threadIndex);
                        mergedCv.wait(mergedLock, [&]() { return cond2 == threadIndex; });
                    }
                    *__mergedPointCloud += *cloud;
                    ++cond2;
                    mergedCv.notify_all();
                } else {
                    __mergedPointCloud = cloud;
                    ++cond1; ++cond2;
                    std::unique_lock<std::mutex> mergedLock(__mergedPointCloudMutex);
                    std::condition_variable mergedCv;
                    mergedCv.notify_all();
                }

                //__pointClouds.emplace_back(std::move(cloud));
                printf("Thread %d finished in %.2f ms: Merged Point Cloud Size: %d\n", threadIndex, threadTimer.elapsedMilliseconds(), __mergedPointCloud->size());
                ++__lastThreadFinished;
                // Print merged point clouds size
            }, __rgbImages.size()-1));
            printf("Thread %d started\n", __rgbImages.size()-1);
        }
    }));
}

void PointCloudConstructor::SaveImageMultiThread(std::shared_ptr<cv::Mat> undistortedRGB,
    std::shared_ptr<cv::Mat> undistortedDepth, std::shared_ptr<cv::Mat> filteredDepth)
{
    static std::unique_ptr<std::thread> saveThread[3];
    static std::mutex saveMutex;
    static std::queue<std::tuple<std::shared_ptr<cv::Mat>, std::shared_ptr<cv::Mat>, std::shared_ptr<cv::Mat>, int>> saveQueue;

    static int coun = 0;
    saveQueue.push(std::make_tuple(undistortedRGB, undistortedDepth, filteredDepth, coun));
    printf("Pushed to save queue: %d\n", coun++);

    if (saveThread[0] != nullptr) return;
    auto createThread = [&]()-> std::thread *
    {
        return new std::thread([&](){
            while (!killThreads) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));
                std::tuple<std::shared_ptr<cv::Mat>, std::shared_ptr<cv::Mat>, std::shared_ptr<cv::Mat>, int> images;
                {
                    std::scoped_lock<std::mutex> lock(saveMutex);
                    if (saveQueue.empty()) {
                        continue;
                    }
                    images = saveQueue.front();
                    saveQueue.pop();
                }

                cv::Mat & undistortedRGB = *std::get<0>(images);
                cv::Mat & undistortedDepth = *std::get<1>(images);
                cv::Mat & filteredDepth = *std::get<2>(images);
                int currentCounter = std::get<3>(images);

                cv::Mat rgb;
                cv::cvtColor(undistortedRGB, rgb, cv::COLOR_RGB2BGR);

                cv::String rgbPath = cv::format("%s/rgb_%d.png", __dirName.c_str(), currentCounter);
                cv::imwrite(rgbPath, rgb);
                cv::String depthPath = cv::format("%s/depth_%d.tiff", __dirName.c_str(), currentCounter);
                cv::imwrite(depthPath, filteredDepth);
                cv::String unfilteredDepthPath = cv::format("%s/unfiltered_depth_%d.tiff", __dirName.c_str(), currentCounter);
                cv::imwrite(unfilteredDepthPath, undistortedDepth);
                printf("Saved captures %d!\n", currentCounter);
            }
        });
    };
    saveThread[0].reset(createThread());
    saveThread[1].reset(createThread());
    saveThread[2].reset(createThread());
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudConstructor::getPointCloud() const { return __mergedPointCloud; }

static cv::Mat createCameraMatrix(const OBCameraIntrinsic& intrinsic)
{
    return (cv::Mat_<float>(3, 3) <<
        intrinsic.fx, 0, intrinsic.cx,
        0, intrinsic.fy, intrinsic.cy,
        0, 0, 1);
}

static cv::Mat createDistortionCoeffs(const OBCameraDistortion& distortion)
{
    return (cv::Mat_<float>(1, 8) <<
        distortion.k1, distortion.k2, distortion.p1, distortion.p2,
        distortion.k3, distortion.k4, distortion.k5, distortion.k6);
}

void PointCloudConstructor::SetCameraParameters(const OBCameraParam& params, int colorWidth, int colorHeight, int depthWidth, int depthHeight)
{
    __isMirrored = params.isMirrored;

    __rgbIntrinsics = createCameraMatrix(params.rgbIntrinsic);
    __depthIntrinsics = createCameraMatrix(params.depthIntrinsic);
    __rgbDistortion = createDistortionCoeffs(params.rgbDistortion);
    __depthDistortion = createDistortionCoeffs(params.depthDistortion);
    
    __rgbToDepthExtrinsics = cv::Mat::eye(4, 4, CV_32F);

    // [R|t] matrix

    // Copy the rotation matrix (3x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            __rgbToDepthExtrinsics.at<float>(i, j) = params.transform.rot[i * 3 + j];
        }
    }

    // Set the translation vector
    __rgbToDepthExtrinsics.at<float>(0, 3) = params.transform.trans[0];
    __rgbToDepthExtrinsics.at<float>(1, 3) = params.transform.trans[1];
    __rgbToDepthExtrinsics.at<float>(2, 3) = params.transform.trans[2];

    // Undistort init
    cv::Size colorSize(colorWidth, colorHeight);
    cv::Size depthSize(depthWidth, depthHeight);
    //colorSize = depthSize;
    depthSize = colorSize;
    // Resize for previewing
    // float downscaler = 2.0f;
    // __rgbIntrinsics = cv::Mat_<float>(3, 3) <<
    //     params.rgbIntrinsic.fx / downscaler, 0, params.rgbIntrinsic.cx / downscaler,
    //     0, params.rgbIntrinsic.fy / downscaler, params.rgbIntrinsic.cy / downscaler,
    //     0, 0, 1;
    // __depthIntrinsics = cv::Mat_<float>(3, 3) <<
    //     params.depthIntrinsic.fx / downscaler, 0, params.depthIntrinsic.cx / downscaler,
    //     0, params.depthIntrinsic.fy / downscaler, params.depthIntrinsic.cy / downscaler,
    //     0, 0, 1;
    // depthSize = cv::Size(depthSize.width / downscaler, depthSize.height / downscaler);
    // colorSize = cv::Size(colorSize.width / downscaler, colorSize.height / downscaler);

    cv::initUndistortRectifyMap(__rgbIntrinsics, __rgbDistortion, cv::Mat(), __rgbIntrinsics, colorSize, CV_32FC1, __undistortRGBMap1, __undistortRGBMap2);
    cv::initUndistortRectifyMap(__depthIntrinsics, __depthDistortion, cv::Mat(), __depthIntrinsics, depthSize, CV_32FC1, __undistortDepthMap1, __undistortDepthMap2);

    __depthToRgbRatioX = (float)colorSize.width / depthSize.width;
    __depthToRgbRatioY = (float)colorSize.height / depthSize.height;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudConstructor::CreatePointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& camera_matrix, float scale)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    float fx = camera_matrix.at<float>(0, 0);
    float fy = camera_matrix.at<float>(1, 1);
    float cx = camera_matrix.at<float>(0, 2);
    float cy = camera_matrix.at<float>(1, 2);

    int reducer = 4;
    cloud->points.reserve(depth.rows * depth.cols / reducer);
    uint32_t nb = 0;
    for (int y = 0; y < depth.rows; y++)
    {
        for (int x = 0; x < depth.cols; x++)
        {
            if (rand() % reducer != 0) continue;
            // Multiply by scale and then set to meters, because it's in mm
            float z = (float)(depth.at<short>(y, x));
            float float_x = x;
            float float_y = y;
            if (z > 0)
            {
                z = z;// / scale;
                float_x;// *= __depthToRgbRatioX;
                float_y;// *= __depthToRgbRatioY;
                cv::Vec3b color = rgb.at<cv::Vec3b>(float_y, float_x);

                // Ignore points which are too bright
                constexpr int threshold = 200;
                if (color[0] > threshold && color[1] > threshold && color[2] > threshold)
                    continue;

                cloud->points.emplace_back(
                ((float)float_x - cx) * z / fx, ((float)float_y - cy) * z / fy, z,
                color[0], color[1], color[2]);
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

void PointCloudConstructor::ProcessFrames(const std::shared_ptr<ob::ColorFrame>& rgbImage,
    const std::shared_ptr<ob::DepthFrame>& depthImage, cv::Mat& undistortedRGB, cv::Mat& undistortedDepth,
    cv::Mat& filteredDepth)
{
    // Get RGB Image
    cv::Mat rawRgbMat(rgbImage->height(), rgbImage->width(), CV_8UC3, rgbImage->data());
    cv::Mat rawDepthMat(depthImage->as<ob::VideoFrame>()->height(), depthImage->as<ob::VideoFrame>()->width(), CV_16UC1, depthImage->as<ob::VideoFrame>()->data());
    __scale = depthImage->getValueScale();
    // Apply scale on each pixel of the depth frame
    rawDepthMat *= __scale;

    // Undistort images
    undistortedRGB = UndistortRGBImage(rawRgbMat, __rgbIntrinsics, __rgbDistortion);
    undistortedDepth = UndistortDepthImage(rawDepthMat, __depthIntrinsics, __depthDistortion);
    filteredDepth = undistortedDepth.clone();

    // Get Depth Image
    // Apply scale on each pixel
    // rawDepthMat.convertTo(rawDepthMat2, CV_16UC1, scale);

    // Find minimum value (over 0) and maximum value and then normalize the depth image
    double minValue, maxValue, maxValueLimitless;

    // To find the minimum value greater than 0
    minValue = std::numeric_limits<double>::max();
    maxValue = maxValueLimitless = 0;
    double limit = 300;
    {
        // Get total rows and cols outside the loop for performance.
        int rows = filteredDepth.rows;
        int cols = filteredDepth.cols;

        while (maxValue == 0 && limit < 2000) {
            const ushort* data = reinterpret_cast<const ushort*>(filteredDepth.data); // Use pointer for faster access
            const ushort* end = data + rows * cols; // Calculate the end pointer

            while (data != end) {
                ushort value = *data++;
                if (value > 0) {
                    if (value < minValue) {
                        minValue = value;
                    }
#ifdef _DEBUG
                    if (value > maxValueLimitless) {
                        maxValueLimitless = value;
                    }
#endif
                    if (value > maxValue && value <= limit) {
                        maxValue = value;
                    }
                }
            }
            limit += 500;
        }
        // std::cout << "Maximum value: " << maxValue << std::endl;
        if (maxValue == 0) {
            return;
        }
        // Filter depth to go under limit
        ushort* data = reinterpret_cast<ushort*>(filteredDepth.data);  // Pointer access to data
        ushort* end = data + rows * cols;  // End pointer
        while (data != end) {
            if (*data >= limit) {
                *data = 0;  // Set the value to 0 if it's >= limit
            }
            ++data;
        }
    }
}

pcl::PolygonMesh::Ptr PointCloudToMesh(const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(pointCloud.makeShared());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.3);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    pcl::concatenateFields(pointCloud, *cloud_normals, *cloud);

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud);
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
    poisson.reconstruct(*mesh);

    // Step 4: Transfer RGB color to the reconstructed mesh (optional, if colors are required in the output mesh)
    // PCL's Poisson implementation doesn't directly support colors, so the vertices of the mesh need to have their colors transferred.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(mesh->cloud, *colored_cloud);  // Extract the point cloud from the mesh

    for (size_t i = 0; i < colored_cloud->points.size(); ++i)
    {
        // Assign RGB values from the original point cloud
        colored_cloud->points[i].r = cloud->points[i].r;
        colored_cloud->points[i].g = cloud->points[i].g;
        colored_cloud->points[i].b = cloud->points[i].b;
    }

    pcl::toPCLPointCloud2(*colored_cloud, mesh->cloud);  // Update the mesh cloud with the new colored points
    return mesh;
}

void PointCloudConstructor::FinishAndSave(const std::string& string)
{
    __finishingThread = std::make_unique<std::thread>([this, string]() {
        printf("Finishing point cloud production...\n");
        // Wait for frame queue to empty
        while (!__rgbFrames.empty() || !__depthFrames.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        printf("Frame queue empty.\n");
        while (__lastThreadFinished < __threads.size() - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        auto filePath = string + "/final_point_cloud.ply";
        printf("Saving %s...\n", filePath.c_str());
        pcl::PLYWriter writer;
        writer.write(filePath, *__mergedPointCloud);
        printf("Point cloud saved!\n");

        // Converts to mesh
        pcl::PolygonMesh::Ptr mesh = PointCloudToMesh(*__mergedPointCloud);
        pcl::io::savePLYFile(string + "/final_mesh.ply", *mesh);
    });
}

void PointCloudConstructor::lockPointCloud()
{
    __mergedPointCloudMutex.lock();
}

void PointCloudConstructor::unlockPointCloud()
{
    __mergedPointCloudMutex.unlock();
}

void PointCloudConstructor::KillThreads()
{
    killThreads = true;
}

void PointCloudConstructor::SetDirectory(const std::string& str)
{
    __dirName = str;
}

void PointCloudConstructor::RenderFromSavedImages()
{
    static std::unique_ptr<std::thread> renderThread[4];

    static std::atomic<int> turn_transformMatrix(0), turn_mergedPointCloud(0);
    static std::atomic<int> threadsFinished(0);

    if (renderThread[0] != nullptr) {
        return;
    }
    auto createThread = [this](int i)-> std::thread *
    {
        return new std::thread([this](int threadIndex)
        {
            while (!killThreads)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));
                Timer threadTimer("Thread " + std::to_string(threadIndex));
                // imageWriteMutex.lock();
                // auto add_image_undistortedRGB = __rgbImages[threadIndex];
                // auto add_image_undistortedDepth = __depthImages[threadIndex];
                // auto add_image_filteredDepth = __filteredDepthImages[threadIndex];
                // auto before_add_image_undistortedRGB = threadIndex != 0 ? __rgbImages[threadIndex-1] : nullptr;
                // auto before_add_image_undistortedDepth = threadIndex != 0 ? __depthImages[threadIndex-1] : nullptr;
                // imageWriteMutex.unlock();
                // Read images

                // If file does not exist, then it is over
                if (std::filesystem::exists(__dirName + "/rgb_" + std::to_string(threadIndex) + ".png") == false) {
                    ++threadsFinished;
                    if (threadsFinished >= std::size(renderThread)) {
                        // Voxel downsampling
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);

                        pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
                        voxelFilter.setInputCloud(__mergedPointCloud);
                        float leafSize = 1.0f;
                        while (cloudFiltered->size() < 1000 && cloudFiltered->size() != __mergedPointCloud->size()) {
                            leafSize += 0.1f;
                            voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
                            printf("Point cloud filter starting...!\n");
                            voxelFilter.filter(*cloudFiltered);
                        }

                        printf("Old point cloud size: %d | Downsampled point cloud size: %d\n", __mergedPointCloud->size(), cloudFiltered->size());
                        pcl::PLYWriter writer;
                        printf("Point cloud save starting...!\n");
                        writer.write(__dirName + "/final_point_cloud_downsampled.ply", *cloudFiltered);
                        writer.write(__dirName + "/final_point_cloud.ply", *__mergedPointCloud);
                        printf("Point cloud saved!\n");
                    }
                    return;
                }

                cv::Mat add_image_undistortedRGB = cv::imread(__dirName + "/rgb_" + std::to_string(threadIndex) + ".png");
                cv::Mat add_image_undistortedDepth = cv::imread(__dirName + "/unfiltered_depth_" + std::to_string(threadIndex) + ".tiff", cv::IMREAD_ANYDEPTH);
                cv::Mat add_image_filteredDepth = cv::imread(__dirName + "/depth_" + std::to_string(threadIndex) + ".tiff", cv::IMREAD_ANYDEPTH);
                cv::Mat before_add_image_undistortedRGB = threadIndex != 0 ? cv::imread(__dirName + "/rgb_" + std::to_string(threadIndex-1) + ".png") : cv::Mat();
                cv::Mat before_add_image_undistortedDepth = threadIndex != 0 ? cv::imread(__dirName + "/unfiltered_depth_" + std::to_string(threadIndex-1) + ".tiff", cv::IMREAD_ANYDEPTH) : cv::Mat();
                printf("Thread %d hello2\n", threadIndex);
                // Convert add_image_undistortedRGB fron BGR to RGB
                cv::cvtColor(add_image_undistortedRGB, add_image_undistortedRGB, cv::COLOR_RGB2BGR);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = CreatePointCloud(add_image_undistortedRGB, add_image_filteredDepth, __depthIntrinsics, __scale);

                if (threadIndex != 0)
                {
                    std::vector<cv::KeyPoint> keypoints1, keypoints2;
                    printf("Thread %d starts matching features\n", threadIndex);
                    std::vector<cv::DMatch> matches = matchFeatures(before_add_image_undistortedRGB, add_image_undistortedRGB, keypoints1, keypoints2);
                    //timer.PrintTimePassed();

                    Eigen::Matrix4d transform = estimatePose(keypoints1, keypoints2, matches, before_add_image_undistortedDepth, add_image_undistortedDepth, __depthIntrinsics);

                    // __transformMatrix represents the cumulative transformation of all images from the first picture (which is the origin)
                    // Wait for thread(threadIndex-1) to finish using __transformMatrix
                    std::unique_lock<std::mutex> transformLock(__transformMutex);
                    std::condition_variable transformCv;
                    while (turn_transformMatrix != threadIndex)
                    {
                        printf("Thread %d waiting for transform\n", threadIndex);
                        transformCv.wait_for(transformLock, std::chrono::milliseconds(100), [&]() { return turn_transformMatrix == threadIndex; });
                    }
                    __transformMatrix = transform * __transformMatrix;
                    ++turn_transformMatrix;
                    transformCv.notify_all();
                    //timer.PrintTimePassed();

                    // So to transform the point cloud, we need to multiply it by the inverse of the cumulative transformation
                    Eigen::Matrix4f transformMatrixFloat = __transformMatrix.cast<float>();
                    pcl::transformPointCloud(*cloud, *cloud, transformMatrixFloat.inverse());
                    //timer.PrintTimePassed();

                    std::unique_lock<std::mutex> mergedLock(__mergedPointCloudMutex);
                    std::condition_variable mergedCv;
                    if (turn_mergedPointCloud != threadIndex)
                    {
                        //printf("Thread %d waiting for merge\n", threadIndex);
                        mergedCv.wait(mergedLock, [&]() { return turn_mergedPointCloud == threadIndex; });
                    }
                    *__mergedPointCloud += *cloud;
                    ++turn_mergedPointCloud;
                    mergedCv.notify_all();
                } else {
                    __mergedPointCloud = cloud;
                    ++turn_transformMatrix; ++turn_mergedPointCloud;
                    std::unique_lock<std::mutex> mergedLock(__mergedPointCloudMutex);
                    std::condition_variable mergedCv;
                    mergedCv.notify_all();
                }

                //__pointClouds.emplace_back(std::move(cloud));
                printf("Thread %d finished in %.2f ms: Merged Point Cloud Size: %d\n", threadIndex, threadTimer.elapsedMilliseconds(), __mergedPointCloud->size());
                ++__lastThreadFinished;
                threadIndex += std::size(renderThread);
            }
        }, i);
    };
    for (int i = 0; i < 4; ++i) {
        renderThread[i].reset(createThread(i));
    }
}

cv::Mat PointCloudConstructor::UndistortRGBImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion)
{
    cv::Mat undistorted;
    //cv::undistort(image, undistorted, intrinsics, distortion);
    cv::remap(image, undistorted, __undistortRGBMap1, __undistortRGBMap2, cv::INTER_LINEAR);
    // resize the image to half for previews
    // cv::resize(undistorted, undistorted, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    return undistorted;
}

cv::Mat PointCloudConstructor::UndistortDepthImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion)
{
    cv::Mat undistorted;
    //cv::undistort(image, undistorted, intrinsics, distortion);
    cv::remap(image, undistorted, __undistortDepthMap1, __undistortDepthMap2, cv::INTER_LINEAR);
    // resize the image to half for previews
    // cv::resize(undistorted, undistorted, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    return undistorted;
}

std::vector<cv::DMatch> PointCloudConstructor::matchFeatures(const cv::Mat& img1, const cv::Mat& img2,
    std::vector<cv::KeyPoint>& keypoints1,
    std::vector<cv::KeyPoint>& keypoints2)
{
    //Timer timer("matchFeatures");
    // Convert images to grayscale if they're not already
    cv::Mat gray1, gray2;
    if (img1.channels() == 3) {
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    } else {
        gray1 = img1;
        gray2 = img2;
    }
    //timer.PrintTimePassed();

    // Create SIFT detector
    cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();

    // Create ORB detector
    // cv::Ptr<cv::Feature2D> detector = cv::ORB::create();

    // Detect keypoints and compute descriptors
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute(gray1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(gray2, cv::noArray(), keypoints2, descriptors2);

    //timer.PrintTimePassed();

    std::vector<cv::DMatch> good_matches;

    // Use FLANN matcher
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
    // Perform knn matching
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Use Brute Force
    // cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    // matcher.match(descriptors1, descriptors2, good_matches);

    //timer.PrintTimePassed();

    DebugLog("Number of keypoints in image 1: " + std::to_string(keypoints1.size()));

    // Optional: Further filter matches based on their distances
    std::sort(good_matches.begin(), good_matches.end());
    // const int numGoodMatches = std::min(100, static_cast<int>(good_matches.size()));
    // good_matches.erase(good_matches.begin() + numGoodMatches, good_matches.end());

    return good_matches;
}

float bilinearInterpolate(const cv::Mat& depth, float x, float y) {
    // Get integer coordinates of the top-left pixel
    int x1 = static_cast<int>(x);
    int y1 = static_cast<int>(y);

    // Ensure coordinates are within image bounds
    if (x1 < 0 || x1 >= depth.cols - 1 || y1 < 0 || y1 >= depth.rows - 1) {
        return 0.0f;  // Return 0 if coordinates are out of bounds
    }

    // Calculate the relative distances for interpolation
    int x2 = x1 + 1;
    int y2 = y1 + 1;

    float dx = x - x1;
    float dy = y - y1;

    // Get the depth values at the 4 surrounding pixels
    float z11 = depth.at<uint16_t>(y1, x1);
    float z12 = depth.at<uint16_t>(y2, x1);
    float z21 = depth.at<uint16_t>(y1, x2);
    float z22 = depth.at<uint16_t>(y2, x2);

    // Perform bilinear interpolation
    float z = (1 - dx) * (1 - dy) * z11
            + dx * (1 - dy) * z21
            + (1 - dx) * dy * z12
            + dx * dy * z22;

    return z;
}

Eigen::Matrix4d PointCloudConstructor::estimatePose(const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& depth1, const cv::Mat& depth2,
    const cv::Mat& camera_matrix)
{
    std::vector<cv::Point2f> points1, points2;
    std::vector<cv::Point3f> points1_3d, points2_3d;
    points1.reserve(matches.size());
    points2.reserve(matches.size());
    points1_3d.reserve(matches.size());
    points2_3d.reserve(matches.size());

    float fx = camera_matrix.at<float>(0, 0);
    float fy = camera_matrix.at<float>(1, 1);
    float cx = camera_matrix.at<float>(0, 2);
    float cy = camera_matrix.at<float>(1, 2);

    float reducer = 1.0f;
    for (const auto& match : matches)
    {
        cv::Point2f pt1 = keypoints1[match.queryIdx].pt;
        cv::Point2f pt2 = keypoints2[match.trainIdx].pt;

        // pt1.x *= __depthToRgbRatioX;
        // pt1.y *= __depthToRgbRatioY;
        // pt2.x *= __depthToRgbRatioX;
        // pt2.y *= __depthToRgbRatioY;

        float z1 = 
            //(float)depth1.at<uint16_t>(pt1.y, pt1.x)
            bilinearInterpolate(depth1, pt1.x, pt1.y)
            / reducer;
        //float z2 = 
        //    //(float)depth2.at<uint16_t>(pt2.y, pt2.x)
        //    bilinearInterpolate(depth2, pt2.x, pt2.y)
        //    / reducer;

        if (z1 > FLT_EPSILON)// && z2 > FLT_EPSILON)
        {
            float x1 = (pt1.x - cx) * z1 / fx;
            float y1 = (pt1.y - cy) * z1 / fy;

            //float x2 = (pt2.x - cx) * z2 / fx;
            //float y2 = (pt2.y - cy) * z2 / fy;

            //points1.emplace_back(pt1);
            points2.emplace_back(pt2);
            points1_3d.emplace_back(x1, y1, z1);
            //points2_3d.emplace_back(x2, y2, z2);
        }
    }

    DebugLog("Working with " + std::to_string(points2.size()) + " points");
    if (points2.size() < 10) {
        std::cerr << "Not enough points to estimate pose!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Use cv::solvePnPRansac instead of cv::estimateAffine3D
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    // Images are undistorted, so no need to pass distortion coefficients
    cv::Mat dc = cv::Mat::zeros(4, 1, CV_64F);
    bool res1 = cv::solvePnPRansac(points1_3d, points2, camera_matrix, dc, rvec, tvec, false, 10000, 3.0, 0.995, inliers, cv::SOLVEPNP_SQPNP);

    DebugLog("Number of inliers: " + std::to_string(inliers.size()));

    if (!res1 || inliers.size() <= 4) {
        std::cerr << "Failed to estimate pose!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Refine the pose using Gaussian-Newton optimization
    // cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, FLT_EPSILON);
    // cv::solvePnPRefineVVS(points1_3d, points2, camera_matrix, dc, rvec, tvec, criteria, 1.0f);

    // Convert rotation vector to rotation matrix
    cv::Mat R;
    //cv::Mat R2;
    cv::Rodrigues(rvec, R);
    //cv::Rodrigues(rvec, R2);

    // Create transformation matrices for both cameras
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // Fill rotation part
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            T(i, j) = R.at<double>(i, j);
            //T2(i, j) = R2.at<double>(i, j);
        }
    }

    // Fill translation part
    for (int i = 0; i < 3; i++)
    {
        T(i, 3) = tvec.at<double>(i);
        //T2(i, 3) = tvec2.at<double>(i);
    }

    // Print both translation and rotation vectors
    DebugLog("Translation: " + std::to_string(tvec.at<double>(0)) + ", " + std::to_string(tvec.at<double>(1)) + ", " + std::to_string(tvec.at<double>(2)));
    //std::cout << "Translation 2: " << tvec2 << std::endl;
    // Rotation 1: [0.9999999973487844, -7.213077070161737e-05, 9.979133911997523e-06;
    // 7.213060113115954e-05, 0.9999999972542273, 1.699181896964441e-05;
    // -9.980359517594991e-06, -1.699109912366759e-05, 0.9999999998058475
    // ]
    DebugLog("Rotation: " + std::to_string(R.at<double>(0, 0)) + ", " + std::to_string(R.at<double>(0, 1)) + ", " + std::to_string(R.at<double>(0, 2)) + "; "
        + std::to_string(R.at<double>(1, 0)) + ", " + std::to_string(R.at<double>(1, 1)) + ", " + std::to_string(R.at<double>(1, 2)) + "; "
        + std::to_string(R.at<double>(2, 0)) + ", " + std::to_string(R.at<double>(2, 1)) + ", " + std::to_string(R.at<double>(2, 2)));
    //std::cout << "Rotation 2: " << R2 << std::endl;

    return T;

    // Get camera positions
    //Eigen::Vector3f camera1_position = -T1.block<3, 3>(0, 0).transpose() * T1.block<3, 1>(0, 3);
    //Eigen::Vector3f camera2_position = -T2.block<3, 3>(0, 0).transpose() * T2.block<3, 1>(0, 3);
    //
    //std::cout << "Camera 1 position: " << camera1_position.transpose() << std::endl;
    //std::cout << "Camera 2 position: " << camera2_position.transpose() << std::endl;
    //
    //// Compute relative transformation
    //Eigen::Matrix4f relativeTransform = T2 * T1.inverse();
    //
    //return relativeTransform;

    // Estimate the affine transformation (including rotation and translation) between the two sets of 3D points
    // cv::Mat affine_transform;
    // std::vector<uchar> inliers;
    // cv::estimateAffine3D(points1, points2, affine_transform, inliers);

    // Convert the result to Eigen::Matrix4f
    //Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //for (int i = 0; i < 3; i++)
    //{
    //    for (int j = 0; j < 4; j++)
    //    {
    //        transform(i, j) = affine_transform.at<double>(i, j);
    //    }
    //}
    //return transform;
}