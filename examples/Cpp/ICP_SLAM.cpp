#include <iostream>
#include <memory>
#include <thread>
#include <sys/types.h>
#include <dirent.h>

#include "Core/Core.h"
#include "IO/IO.h"
#include "Visualization/Visualization.h"

double vox_s = 0.50;
double max_d = 0.50;
int i = 1;

std::vector<std::string> get_file_names(const std::string& dir_name)
{
    std::vector<std::string> file_names;
    DIR* dirp = opendir(dir_name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        file_names.push_back(dp->d_name);
    }
    closedir(dirp);
    return file_names;
}


int main() {
    std::string dir_name = "point_cloud";
    std::vector<std::string> recordfilename = get_file_names(dir_name);
    printf("There are %d files.\n", recordfilename.size());

    auto target = std::make_shared<open3d::PointCloud>();
    auto target_down = std::make_shared<open3d::PointCloud>();

    auto mesh = open3d::CreateMeshCoordinateFrame(3.0);

    auto Search_para = open3d::KDTreeSearchParamHybrid(vox_s * 2, 30);
    open3d::SetVerbosityLevel(open3d::VerbosityLevel::VerboseAlways);
    open3d::ReadPointCloud(recordfilename[0], *target);
    target_down = open3d::VoxelDownSample(*target, vox_s);
    open3d::EstimateNormals(*target_down, Search_para);
    Eigen::Matrix4d current_transformation = Eigen::Matrix4d::Identity();
    open3d::DrawGeometriesWithAnimationCallback(
        {target_down, mesh},
        [&](open3d::Visualizer *vis) {
            int time = clock();
            auto source_down = std::make_shared<open3d::PointCloud>();

            if (open3d::ReadPointCloud(recordfilename[i], *source_down)) {
                int timetime = clock();
                printf("Time for reading file = %dms\n", clock() - time);
                time = clock();
                source_down->points_ =
                    open3d::VoxelDownSample(*source_down, vox_s)->points_;
                printf("Time for down sampling = %dms\n", clock() - time);
                time = clock();
                source_down->Transform(current_transformation);
                printf("Time for transformation = %dms\n", clock() - time);
                time = clock();
                open3d::EstimateNormals(*source_down, Search_para);
                printf("Time for Normal Estimation = %dms\n", clock() - time);
                time = clock();
                open3d::RegistrationResult result_icp = open3d::RegistrationICP(
                    *source_down, *target_down, max_d,
                    Eigen::Matrix4d::Identity(),
                    open3d::TransformationEstimationPointToPlane());
                printf("Time for ICP = %dms\n", clock() - time);
                time = clock();
                current_transformation =
                    result_icp.transformation_ * current_transformation;
                mesh->Transform(result_icp.transformation_);
                printf("Time for Matrix * Matrix = %dms\n", clock() - time);
                time = clock();
                source_down->Transform(result_icp.transformation_);
                printf("Time for transformation 2 = %dms\n", clock() - time);
                time = clock();
                for (int k = 0; k < source_down->points_.size(); k++)
                    target_down->points_.push_back(source_down->points_[k]);
                printf("Time for push_back = %dms\n", clock() - time);
                time = clock();
                target_down->points_ =
                    open3d::VoxelDownSample(*target_down, vox_s)->points_;
                printf("Time for down sampling = %dms\n", clock() - time);
                time = clock();
                open3d::EstimateNormals(*target_down, Search_para);
                printf("Time for Normal Estimation = %dms\n", clock() - time);
                i++;
                printf("Time for all is %dms\n", clock() - timetime);
                return true;
            } else {
                return false;
            }
        },
        "Result");

    return 1;
}
