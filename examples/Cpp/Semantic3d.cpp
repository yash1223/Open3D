#include <iostream>
#include <memory>
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

using namespace std;
using namespace open3d;

int main() {
    PointCloud point_cloud;
    ReadPointCloudFromXYZRGB(
        "/home/ylao/data/semantic3d/bildstein_station1_xyz_intensity_rgb.txt",
        point_cloud);
    cout << point_cloud.points_.size() << endl;
    return 0;
}