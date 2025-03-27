#define NOMINMAX
#include "../Application_OrbbecSDK/tools.h"

#include <open3d/Open3D.h>
#include <open3d/t/pipelines/registration/Registration.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

void PCL_icp(const std::vector<std::string>& paths, const std::string& path)
{
    std::cout << "Using PCL for ICP" << std::endl;
    // Load point clouds
    // Create pointers for source and target point clouds
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds(paths.size(), nullptr);

    for (int i = 0; i < paths.size(); ++i)
    {
        std::cout << "Loading " << paths[i] << std::endl;
        pointClouds[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::io::loadPLYFile(paths[i], *pointClouds[i]);
    }

    // Register point clouds
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(pointClouds[1]);
    icp.setInputTarget(pointClouds[0]);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);

    pcl::PointCloud<pcl::PointXYZRGB> Final;
    printf("Aligning...\n");
    icp.align(Final);

    if (icp.hasConverged())
    {
        std::cout << "ICP has converged, score is: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix: \n" << icp.getFinalTransformation() << std::endl;

        // Transform the source cloud using the final transformation matrix
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*pointClouds[1], *transformed_source, icp.getFinalTransformation());

        // Merge the transformed source cloud and the target cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *merged_cloud = *transformed_source + *pointClouds[0];

        // Save the merged cloud to a PLY file
        pcl::PLYWriter writer;
        writer.write(path + "/aligned.ply", *merged_cloud, true);

        std::cout << "Merged point cloud saved to 'aligned.ply'." << std::endl;
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
    }
}

void Open3D_icp(const std::vector<std::string>& paths, const std::string& path)
{
    std::cout << "Using Open3D for ICP" << std::endl;
    // Load point clouds
    // Create pointers for source and target point clouds
    std::vector<open3d::geometry::PointCloud> pointClouds(paths.size());

    open3d::io::ReadPointCloudOption option;
    option.print_progress = true;
    option.format = "ply";
    option.remove_infinite_points = true;
    option.remove_nan_points = true;
    option.update_progress = nullptr;
    for (int i = 0; i < paths.size(); ++i)
    {
        std::cout << "Loading " << paths[i] << std::endl;
        open3d::io::ReadPointCloudFromPLY(paths[i], pointClouds[i], option);
    }

    // estimate normals
    for (int i = 0; i < pointClouds.size(); ++i)
    {
        std::cout << "Estimating normals for point cloud " << i << std::endl;
        pointClouds[i].EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.5, 200));
    }

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria(1e-10, 1e-10, 1000);

    // Register point clouds
    std::cout << "Registering point clouds..." << std::endl;

    // Point-to-point ICP
    auto result = open3d::pipelines::registration::RegistrationICP(pointClouds[1], pointClouds[0], 2, transformation, open3d::pipelines::registration::TransformationEstimationPointToPoint(), criteria);

    // Colored ICP
    //auto result = open3d::pipelines::registration::RegistrationColoredICP(pointClouds[1], pointClouds[0], 2, transformation, open3d::pipelines::registration::TransformationEstimationForColoredICP(), criteria);

    // Multi-scale ICP
    std::vector<double> voxel_sizes = { 4.0, 3.6, 3.0, 2.5 }; // coarse to fine scales
    std::vector<double> max_correspondence_distances = { 2.0, 1.8, 1.6, 1.5 }; // corresponding max distances


    // Downsampling and running ICP for each scale
    for (size_t i = 0; i < voxel_sizes.size(); ++i)
    {
        std::cout << "Running ICP at scale: " << i + 1 << ", Voxel size: " << voxel_sizes[i] << std::endl;

        // Downsample point clouds at current scale
        auto source_down = pointClouds[1].VoxelDownSample(voxel_sizes[i]);
        auto target_down = pointClouds[0].VoxelDownSample(voxel_sizes[i]);

        // Estimate normals for the downsampled point clouds
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_sizes[i], 30));
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_sizes[i], 30));

        // Run ICP at the current scale
        //auto result = open3d::pipelines::registration::RegistrationICP(
        //    *source_down, *target_down, max_correspondence_distances[i], transformation,
        //    open3d::pipelines::registration::TransformationEstimationPointToPoint(), criteria);
        
        // Run Colored ICP
        auto result = open3d::pipelines::registration::RegistrationColoredICP(
            *source_down, *target_down, voxel_sizes[i], transformation,
            open3d::pipelines::registration::TransformationEstimationForColoredICP(), criteria);

        // Update transformation
        transformation = result.transformation_ * transformation;

        std::cout << "Scale " << i + 1 << " ICP score: " << result.fitness_ << std::endl;
    }

    // Apply the final transformation to the original source point cloud
    pointClouds[1].Transform(transformation);

    // Merge the transformed source cloud and the target cloud
    pointClouds[0] += pointClouds[1];

    // Save the merged cloud to a PLY file
    open3d::io::WritePointCloudOption writeOption;
    writeOption.compressed = open3d::io::WritePointCloudOption::Compressed::Uncompressed;
    writeOption.format = "ply";
    writeOption.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Ascii;
    open3d::io::WritePointCloudToPLY(path + "/aligned.ply", pointClouds[0], writeOption);

    std::cout << "Merged point cloud saved to 'aligned.ply'." << std::endl;
}

int main(int argc, char** argv)
{
    // Windows GUI for chosing directory of point clouds to register
    std::string path = askForDirectory("Select the point clouds directory");

    // rm aligned.py
    std::filesystem::remove((path + "/aligned.ply").c_str());

    // Get all .ply files
    std::vector<std::string> paths = getFilesInDirectory(path, ".ply");

    Open3D_icp(paths, path);

    return EXIT_SUCCESS;
}