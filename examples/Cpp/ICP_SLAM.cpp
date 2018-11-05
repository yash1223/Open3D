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

std::vector<std::string> get_file_paths(const std::string& dir_name)
{
    std::vector<std::string> file_paths;
    DIR* dirp = opendir(dir_name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        std::string file_name(dp->d_name);
        if (file_name.size() >= 4 &&
            file_name.substr(file_name.size() - 4, 4) == ".pcd") {
                file_paths.push_back(dir_name + file_name);
        }
    }
    closedir(dirp);
    return file_paths;
}


int main() {
    std::string dir_name = "../../../examples/Cpp/point_cloud/";
    std::vector<std::string> recordfilename = get_file_paths(dir_name);
    printf("There are %ld files.\n", recordfilename.size());

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
                printf("Time for reading file = %ldms\n", clock() - time);
                time = clock();
                source_down->points_ =
                    open3d::VoxelDownSample(*source_down, vox_s)->points_;
                printf("Time for down sampling = %ldms\n", clock() - time);
                time = clock();
                source_down->Transform(current_transformation);
                printf("Time for transformation = %ldms\n", clock() - time);
                time = clock();
                open3d::EstimateNormals(*source_down, Search_para);
                printf("Time for Normal Estimation = %ldms\n", clock() - time);
                time = clock();
                open3d::RegistrationResult result_icp = open3d::RegistrationICP(
                    *source_down, *target_down, max_d,
                    Eigen::Matrix4d::Identity(),
                    open3d::TransformationEstimationPointToPlane());
                printf("Time for ICP = %ldms\n", clock() - time);
                time = clock();
                current_transformation =
                    result_icp.transformation_ * current_transformation;
                mesh->Transform(result_icp.transformation_);
                printf("Time for Matrix * Matrix = %ldms\n", clock() - time);
                time = clock();
                source_down->Transform(result_icp.transformation_);
                printf("Time for transformation 2 = %ldms\n", clock() - time);
                time = clock();
                for (int k = 0; k < source_down->points_.size(); k++)
                    target_down->points_.push_back(source_down->points_[k]);
                printf("Time for push_back = %ldms\n", clock() - time);
                time = clock();
                target_down->points_ =
                    open3d::VoxelDownSample(*target_down, vox_s)->points_;
                printf("Time for down sampling = %ldms\n", clock() - time);
                time = clock();
                open3d::EstimateNormals(*target_down, Search_para);
                printf("Time for Normal Estimation = %ldms\n", clock() - time);
                i++;
                printf("Time for all is %ldms\n", clock() - timetime);
                return true;
            } else {
                return false;
            }
        },
        "Result");

    return 1;
}
