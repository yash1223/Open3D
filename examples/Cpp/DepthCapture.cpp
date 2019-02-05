// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <iostream>
#include <memory>
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

using namespace open3d;

class VisualizerWithDepthCapture : public VisualizerWithCustomAnimation
{
protected:
    void KeyPressCallback(GLFWwindow *window,
            int key, int scancode, int action, int mods) override {
        if (action == GLFW_RELEASE) {
            return;
        }
        if (key == GLFW_KEY_S) {
            CaptureDepthImage("depth.png");
            CaptureDepthPointCloud("depth.ply");
            PinholeCameraTrajectory camera;
            camera.parameters_.resize(1);
            view_control_ptr_->ConvertToPinholeCameraParameters(
                    camera.parameters_[0]);
            WriteIJsonConvertible("camera.json", camera);
        } else if (key == GLFW_KEY_L) {
            if (filesystem::FileExists("depth.png") &&
                    filesystem::FileExists("camera.json")) {
                PinholeCameraTrajectory camera;
                ReadIJsonConvertible("camera.json", camera);
                auto image_ptr = CreateImageFromFile("depth.png");
                auto pointcloud_ptr = CreatePointCloudFromDepthImage(*image_ptr,
                        camera.parameters_[0].intrinsic_,
                        camera.parameters_[0].extrinsic_);
                AddGeometry(pointcloud_ptr);
            }
        } else if (key == GLFW_KEY_K) {
            if (filesystem::FileExists("depth.ply")) {
                auto pointcloud_ptr = CreatePointCloudFromFile("depth.ply");
                AddGeometry(pointcloud_ptr);
            }
        } else if (key == GLFW_KEY_P) {
            if (filesystem::FileExists("depth.png") &&
                    filesystem::FileExists("camera.json")) {
                PinholeCameraTrajectory camera;
                ReadIJsonConvertible("camera.json", camera);
                view_control_ptr_->ConvertFromPinholeCameraParameters(
                        camera.parameters_[0]);
            }
        } else {
            VisualizerWithCustomAnimation::KeyPressCallback(
                    window, key, scancode, action, mods);
        }
        UpdateRender();
    }
};

int main(int argc, char *argv[])
{
    SetVerbosityLevel(VerbosityLevel::VerboseAlways);
    std::string cameras_path = "/home/ylao/repo/intelstudios/code/colmap_results/colormap/result_20190205_0200/T1S16_0480_camera_trajectory.json";
    PinholeCameraTrajectory camera_params;
    ReadIJsonConvertible(cameras_path, camera_params);

    auto extrinsic = camera_params.parameters_[0].extrinsic_;
    std::cout << "PinholeCameraTrajectory loaded" << std::endl;
    std::cout << "extrinsic:" << std::endl;
    std::cout << extrinsic << std::endl;

    auto camera_center = camera_params.parameters_[0].GetCameraCenter();
    std::cout << "camera_center:" << std::endl;
    std::cout << camera_center << std::endl;
    return 0;
}
