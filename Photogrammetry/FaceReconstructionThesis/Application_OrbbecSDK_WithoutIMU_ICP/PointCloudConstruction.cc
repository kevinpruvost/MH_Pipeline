#include "PointCloudConstruction.h"

PointCloudConstructor::PointCloudConstructor()
    : __pointCloud()
    , __transformMatrix(Eigen::Matrix4d::Identity())
{
#ifdef _DEBUG
    //cv::namedWindow("TestDepth", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("UnfilteredTestDepth", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("TestRGB", cv::WINDOW_AUTOSIZE);

    //// Set size to half the screen
    //cv::resizeWindow("TestDepth", 640, 480);
    //cv::resizeWindow("TestRGB", 640, 480);
#endif
}

// Function to convert cv::Mat to open3d::geometry::Image
std::shared_ptr<open3d::geometry::Image> ConvertMatToOpen3DImage(const cv::Mat& mat) {
    auto o3d_image = std::make_shared<open3d::geometry::Image>();
    o3d_image->Prepare(mat.cols, mat.rows, mat.channels(), mat.elemSize1());

    if (mat.isContinuous()) {
        memcpy(o3d_image->data_.data(), mat.data, mat.total() * mat.elemSize());
    }
    else {
        // Handle non-continuous matrix (unlikely in this case)
        for (int i = 0; i < mat.rows; ++i) {
            memcpy(o3d_image->data_.data() + i * mat.step, mat.ptr(i), mat.cols * mat.elemSize());
        }
    }
    return o3d_image;
}

void PointCloudConstructor::AddImage(const std::shared_ptr<ob::ColorFrame>& rgbFrame, const std::shared_ptr<ob::DepthFrame>& depthFrame)
{
    // Get RGB Image
    cv::Mat rawRgbMat(rgbFrame->height(), rgbFrame->width(), CV_8UC3, rgbFrame->data());
    cv::Mat rawDepthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    __scale = depthFrame->getValueScale();

    // Undistort images
    __add_image_undistortedRGB = UndistortRGBImage(rawRgbMat, __rgbIntrinsics, __rgbDistortion);
    // Convert color
    cv::cvtColor(__add_image_undistortedRGB, __add_image_undistortedRGB, cv::COLOR_BGR2RGB);

    __add_image_undistortedDepth = UndistortDepthImage(rawDepthMat, __depthIntrinsics, __depthDistortion);
    __add_image_filteredDepth = rawDepthMat.clone();

    // Get Depth Image
    // Apply scale on each pixel
    // rawDepthMat.convertTo(rawDepthMat2, CV_16UC1, scale);

    // Find minimum value (over 0) and maximum value and then normalize the depth image
    double minValue, maxValue, maxValueLimitless;

    // To find the minimum value greater than 0
    minValue = std::numeric_limits<double>::max();
    maxValue = maxValueLimitless = 0;
    double limit = 3000;
    while (maxValue == 0 && limit < 20000) {
        for (int i = 0; i < __add_image_filteredDepth.rows; ++i) {
            for (int j = 0; j < __add_image_filteredDepth.cols; ++j) {
                ushort value = __add_image_filteredDepth.at<ushort>(i, j);
                if (value > 0 && value < minValue) {
                    minValue = value;
                }
                if (value > maxValue)
                {
                    if (value > maxValueLimitless) maxValueLimitless = value;
                    if (value <= limit) {
                        maxValue = value;
                    }
                }
            }
        }
        limit += 1000;
    }
    std::cout << "Maximum value: " << maxValue << std::endl;
    if (maxValue == 0) {
        return;
    }
    // Filter depth to go under limit
    for (int i = 0; i < __add_image_filteredDepth.rows; ++i) {
        for (int j = 0; j < __add_image_filteredDepth.cols; ++j) {
            ushort & value = __add_image_filteredDepth.at<ushort>(i, j);
            if (value >= limit) {
                value = 0;
            }
        }
    }


    cv::Mat depth = __add_image_filteredDepth.clone();
    depth.convertTo(depth, CV_8UC1, 255.0 / (maxValue - minValue), -255.0 * minValue / (maxValue - minValue));
    cv::Mat rgb;
    cv::cvtColor(__add_image_undistortedRGB, rgb, cv::COLOR_RGB2BGR);
    //cv::imshow("TestRGB", rgb);
    //cv::imshow("TestDepth", depth);
    //cv::imshow("UnfilteredTestDepth", undistorted_depth);

    static int counter = 0;
    cv::String rgbPath = cv::format("rgb.png");
    cv::imwrite(rgbPath, __add_image_undistortedRGB);
    cv::Mat undistorted_depth2;
    __add_image_filteredDepth.convertTo(undistorted_depth2, CV_8UC1, 255.0 / (maxValue - minValue), -255.0 * minValue / (maxValue - minValue));
    cv::String depthPath = cv::format("depth.png");
    cv::imwrite(depthPath, undistorted_depth2);
    cv::String unfilteredDepthPath = cv::format("unfiltered_depth.png");
    cv::Mat unfilteredDepth;
    __add_image_undistortedDepth.convertTo(unfilteredDepth, CV_8UC1, 255.0 / (maxValueLimitless - minValue), -255.0 * minValue / (maxValueLimitless - minValue));
    cv::imwrite(unfilteredDepthPath, unfilteredDepth);

    open3d::geometry::PointCloud newCloud = CreatePointCloud(__add_image_undistortedRGB, __add_image_undistortedDepth, __depthIntrinsics, __scale);
    Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();

    //if (!__rgbImages.empty()) {
    //    // Compute the transformation matrix between the current and previous RGB images
    //    // This is done by finding the transformation that aligns the two point clouds
    //    // The transformation is computed using the Point-to-Point ICP algorithm
    //    auto settings = open3d::pipelines::registration::ICPConvergenceCriteria();
    //    settings.max_iteration_ = 200;
    //    settings.relative_fitness_ = 1e-6;
    //    settings.relative_rmse_ = 1e-6;

    //    open3d::pipelines::registration::RegistrationResult result = open3d::pipelines::registration::RegistrationICP(
    //        __pointClouds.back(),
    //        newCloud, 0.1,
    //        transformMatrix,
    //        open3d::pipelines::registration::TransformationEstimationPointToPlane(), settings);

    //    // Apply the transformation to the new point cloud
    //    newCloud.Transform(transformMatrix.cast<double>());

    //    // Update the transformation matrix
    //    __transformMatrix = transformMatrix * __transformMatrix;
    //}

    // Generate normals for the new point cloud
    float voxel_size = 0.05f;
    //newCloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0f, 30));

    if (!__rgbImages.empty()) {
        // Prepare the point clouds for Fast Global Registration
        //auto source_down = __pointClouds.back().VoxelDownSample(voxel_size);
        //auto target_down = newCloud.VoxelDownSample(voxel_size);

        //// Estimate FPFH (Fast Point Feature Histograms) for both point clouds
        //auto source_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        //    *source_down,
        //    open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 10.0f, 100)
        //);
        //printf("Done1!\n");

        //auto target_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        //    *target_down,
        //    open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 10.0f, 100)
        //);
        //printf("Done2!\n");

        //// Configure FastGlobalRegistration option
        //open3d::pipelines::registration::FastGlobalRegistrationOption option;
        //option.maximum_correspondence_distance_ = voxel_size * 0.5f; // Adjust as needed for more accuracy

        //// Perform Ransac Registration
        //auto corr1 = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
        //auto corr2 = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5);
        //std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker = {
        //    std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>(corr1),
        //    std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>(corr2)
        //};
        //auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
        //    *source_down,
        //    *target_down,
        //    *source_fpfh,
        //    *target_fpfh,
        //    true, option.maximum_correspondence_distance_,
        //    open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
        //    4,
        //    correspondence_checker
        //);

        //// Perform Fast Global Registration
        //// auto result = open3d::pipelines::registration::FastGlobalRegistrationBasedOnFeatureMatching(
        ////     *source_down,
        ////     *target_down,
        ////     *source_fpfh,
        ////     *target_fpfh,
        ////     option
        //// );
        //printf("Done3!\n");

        //// Extract transformation matrix from result
        //transformMatrix = result.transformation_;

        //// Apply the transformation to the new point cloud
        //newCloud.Transform(transformMatrix);

        //// Update the global transformation matrix
        //__transformMatrix = transformMatrix * __transformMatrix;

        //auto source = __pointClouds.back();

        // Scale depth images as they are smaller
        cv::Mat scaledDepth1;
        cv::Mat scaledDepth2;
        cv::resize(__depthImages.back(), scaledDepth2, __add_image_undistortedRGB.size());
        cv::resize(__add_image_undistortedDepth, scaledDepth1, __add_image_undistortedRGB.size());


        auto rgb_source = ConvertMatToOpen3DImage(__rgbImages.back());
        auto rgb_target = ConvertMatToOpen3DImage(__add_image_undistortedRGB);
        auto depth_source = ConvertMatToOpen3DImage(scaledDepth1);
        auto depth_target = ConvertMatToOpen3DImage(scaledDepth2);

        auto source = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*rgb_source, *depth_source, 1.0, 10000.0, false);
        auto target = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*rgb_target, *depth_target, 1.0, 10000.0, false);

        open3d::camera::PinholeCameraIntrinsic intrinsic;
        intrinsic.width_ = __add_image_undistortedRGB.cols;
        intrinsic.height_ = __add_image_undistortedRGB.rows;
        intrinsic.intrinsic_matrix_(0, 0) = __rgbIntrinsics.at<float>(0, 0);
        intrinsic.intrinsic_matrix_(1, 1) = __rgbIntrinsics.at<float>(1, 1);
        intrinsic.intrinsic_matrix_(0, 2) = __rgbIntrinsics.at<float>(0, 2);
        intrinsic.intrinsic_matrix_(1, 2) = __rgbIntrinsics.at<float>(1, 2);

        Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();

        open3d::pipelines::odometry::OdometryOption odometry_option;
        odometry_option.depth_diff_max_ = 0.150;
        odometry_option.depth_max_ = 1000000;
        odometry_option.depth_min_ = 1;

        std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> res = open3d::pipelines::odometry::ComputeRGBDOdometry(*target, *source, intrinsic, odo_init, open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(), odometry_option);

        if (std::get<0>(res)) {
            transformMatrix = std::get<1>(res);
            std::cout << "Odometry matrix: \n" << transformMatrix << std::endl;
            newCloud.Transform(transformMatrix.cast<double>());
            __transformMatrix = transformMatrix * __transformMatrix;
        }
        else {
            std::cerr << "Failed to compute odometry!" << std::endl;
        }
    }

    __rgbImages.emplace_back(std::move(__add_image_undistortedRGB));
    __depthImages.emplace_back(std::move(__add_image_undistortedDepth));
    __depthImagesFiltered.emplace_back(std::move(__add_image_filteredDepth));
    __pointClouds.emplace_back(std::move(newCloud));
    __transforms.push_back(transformMatrix);

    // Write new point cloud to file
    static int c = 0;
    auto path = cv::format("pointcloud_%d.ply", c++);
    open3d::io::WritePointCloud(path, __pointClouds.back());
}

void PointCloudConstructor::AddImageAndBuildPointCloud(const std::shared_ptr<ob::ColorFrame>& rgbImage, const std::shared_ptr<ob::DepthFrame>& depthImage)
{
    AddImage(rgbImage, depthImage);

    open3d::geometry::PointCloud cloud = CreatePointCloud(__add_image_undistortedRGB, __add_image_filteredDepth, __depthIntrinsics, __scale);
    __pointCloud = cloud;
}

open3d::geometry::PointCloud PointCloudConstructor::getPointCloud() const { return __pointCloud; }

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
    // TODO: Camera distortion is not good
    cv::initUndistortRectifyMap(__rgbIntrinsics, __rgbDistortion, cv::Mat(), __rgbIntrinsics, cv::Size(colorWidth, colorHeight), CV_32FC1, __undistortRGBMap1, __undistortRGBMap2);
    cv::initUndistortRectifyMap(__depthIntrinsics, __depthDistortion, cv::Mat(), __depthIntrinsics, cv::Size(depthWidth, depthHeight), CV_32FC1, __undistortDepthMap1, __undistortDepthMap2);
}

open3d::geometry::PointCloud PointCloudConstructor::CreatePointCloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Mat& camera_matrix, float scale = 1000.0f)
{
    // Create an empty Open3D PointCloud
    open3d::geometry::PointCloud cloud;

    // Camera intrinsic parameters
    float fx = camera_matrix.at<float>(0, 0);
    float fy = camera_matrix.at<float>(1, 1);
    float cx = camera_matrix.at<float>(0, 2);
    float cy = camera_matrix.at<float>(1, 2);

    // Iterate through each pixel in the depth map
    for (int y = 0; y < depth.rows; y++)
    {
        for (int x = 0; x < depth.cols; x++)
        {
            // Get the depth value and convert to meters
            float z = static_cast<float>(depth.at<uint16_t>(y, x));

            if (z > 0)
            {
                // Compute the 3D position in the camera coordinate system
                float Z = z / scale;
                float X = (x - cx) * Z / fx;
                float Y = (y - cy) * Z / fy;

                // Get the corresponding RGB color
                cv::Vec3b color = rgb.at<cv::Vec3b>(y, x);
                Eigen::Vector3d point(X, Y, Z);
                Eigen::Vector3d rgb_color(color[2] / 255.0, color[1] / 255.0, color[0] / 255.0);

                // Add the point and its color to the point cloud
                cloud.points_.push_back(point);
                cloud.colors_.push_back(rgb_color);
            }
        }
    }

    return cloud;
}

cv::Mat PointCloudConstructor::UndistortRGBImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion)
{
    cv::Mat undistorted;
    cv::undistort(image, undistorted, intrinsics, distortion);
    cv::remap(image, undistorted, __undistortRGBMap1, __undistortRGBMap2, cv::INTER_LINEAR);
    return undistorted;
}

cv::Mat PointCloudConstructor::UndistortDepthImage(const cv::Mat& image, const cv::Mat& intrinsics, const cv::Mat& distortion)
{
    cv::Mat undistorted;
    cv::undistort(image, undistorted, intrinsics, distortion);
    cv::remap(image, undistorted, __undistortDepthMap1, __undistortDepthMap2, cv::INTER_LINEAR);
    return undistorted;
}

open3d::geometry::PointCloud mergePointClouds(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& clouds,
    const std::vector<Eigen::Matrix4d>& transforms)
{
    // Create an empty Open3D PointCloud to store the merged result
    open3d::geometry::PointCloud merged_cloud;

    for (size_t i = 0; i < clouds.size(); ++i)
    {
        // Create a new transformed point cloud for the current cloud
        open3d::geometry::PointCloud transformed_cloud;

        // Apply the transformation to each point in the cloud[i]
        for (size_t j = 0; j < clouds[i]->points_.size(); ++j)
        {
            // Extract the point from the point cloud
            Eigen::Vector3d point = clouds[i]->points_[j];

            // Convert to homogeneous coordinates (4x1 vector)
            Eigen::Vector4d point_homo(point(0), point(1), point(2), 1.0);

            // Apply the transformation matrix
            Eigen::Vector4d point_transformed_homo = transforms[i] * point_homo;

            // Convert back to 3D coordinates by dividing by w (homogeneous)
            Eigen::Vector3d point_transformed(
                point_transformed_homo(0) / point_transformed_homo(3),
                point_transformed_homo(1) / point_transformed_homo(3),
                point_transformed_homo(2) / point_transformed_homo(3));

            // Add the transformed point to the transformed_cloud
            transformed_cloud.points_.push_back(point_transformed);

            // Copy the color information (normalized to [0, 1])
            Eigen::Vector3d color = clouds[i]->colors_[j];
            transformed_cloud.colors_.emplace_back(color);
        }

        // Merge the transformed cloud with the overall merged cloud
        merged_cloud += transformed_cloud;
    }

    return merged_cloud;
}

void PointCloudFromFrame(std::shared_ptr<open3d::geometry::PointCloud> & pc, const std::shared_ptr<ob::Frame>& frame)
{
    pc->points_.clear();
    pc->colors_.clear();
    std::vector<OBColorPoint> rgbPoints = getRGBPointsFromFrame(frame);
    // Fill the open3d point cloud
    // Reserve space for points and colors in the point cloud
    pc->points_.reserve(rgbPoints.size());
    pc->colors_.reserve(rgbPoints.size());

    // Fill the open3d point cloud with points and colors
    for (const auto& rgbPoint : rgbPoints) {
        // Add the 3D point to the point cloud
        pc->points_.emplace_back(rgbPoint.x, rgbPoint.y, rgbPoint.z);

        // Add the color (normalized to [0, 1] range)
        pc->colors_.emplace_back(rgbPoint.r, rgbPoint.g, rgbPoint.b);
    }
}
