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

#include "TriangleMeshAndImageUtilities.h"

#include <Core/Camera/PinholeCameraTrajectory.h>
#include <Core/ColorMap/ImageWarpingField.h>
#include <Core/Geometry/Image.h>
#include <Core/Geometry/RGBDImage.h>
#include <Core/Geometry/TriangleMesh.h>
#include <iostream>

namespace open3d {

inline std::tuple<float, float, float> Project3DPointAndGetUVDepth(
        const Eigen::Vector3d X,
        const PinholeCameraTrajectory& camera,
        int camid) {
    std::pair<double, double> f =
            camera.parameters_[camid].intrinsic_.GetFocalLength();
    std::pair<double, double> p =
            camera.parameters_[camid].intrinsic_.GetPrincipalPoint();
    Eigen::Vector4d Vt = camera.parameters_[camid].extrinsic_ *
                         Eigen::Vector4d(X(0), X(1), X(2), 1);
    float u = float((Vt(0) * f.first) / Vt(2) + p.first);
    float v = float((Vt(1) * f.second) / Vt(2) + p.second);
    float z = float(Vt(2));
    return std::make_tuple(u, v, z);
}

std::tuple<std::vector<std::vector<int>>, std::vector<std::vector<int>>>
CreateVertexAndImageVisibility(
        const TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_depth,
        const std::vector<std::shared_ptr<Image>>& images_mask,
        const PinholeCameraTrajectory& camera,
        double maximum_allowable_depth,
        double depth_threshold_for_visiblity_check) {
    auto n_camera = camera.parameters_.size();
    auto n_vertex = mesh.vertices_.size();
    std::vector<std::vector<int>> visiblity_vertex_to_image;
    std::vector<std::vector<int>> visiblity_image_to_vertex;
    visiblity_vertex_to_image.resize(n_vertex);
    visiblity_image_to_vertex.resize(n_camera);
    // #ifdef _OPENMP
    // #pragma omp parallel for schedule(static)
    // #endif
    for (int c = 0; c < n_camera; c++) {
        int viscnt = 0;
        size_t reject_image_boundary = 0;
        size_t reject_allowable_depth = 0;
        size_t reject_images_mask = 0;
        size_t reject_depth_threshold = 0;
        for (int vertex_id = 0; vertex_id < n_vertex; vertex_id++) {
            Eigen::Vector3d X = mesh.vertices_[vertex_id];
            float u, v, d;
            std::tie(u, v, d) = Project3DPointAndGetUVDepth(X, camera, c);
            int u_d = int(round(u)), v_d = int(round(v));
            if (d < 0.0 || !images_depth[c]->TestImageBoundary(u_d, v_d)) {
                reject_image_boundary++;
                continue;
            }
            float d_sensor = *PointerAt<float>(*images_depth[c], u_d, v_d);
            if (d_sensor > maximum_allowable_depth) {
                if (reject_allowable_depth < 10) {
                    std::cout << "d_sensor " << d_sensor
                              << " maximum_allowable_depth "
                              << maximum_allowable_depth << std::endl;
                }
                reject_allowable_depth++;
                continue;
            }
            if (*PointerAt<unsigned char>(*images_mask[c], u_d, v_d) == 255) {
                reject_images_mask++;
                continue;
            }
            if (std::fabs(d - d_sensor) >=
                depth_threshold_for_visiblity_check) {
                reject_depth_threshold++;
                continue;
            }
            // #ifdef _OPENMP
            // #pragma omp critical
            // #endif
            {
                visiblity_vertex_to_image[vertex_id].push_back(c);
                visiblity_image_to_vertex[c].push_back(vertex_id);
                viscnt++;
            }
        }
        PrintDebug(
                "[cam %d] Total %d, rj_image_boundary %d, rj_allowable_depth "
                "%d, rj_images_mask %d, rj_depth_threshold %d \n",
                c, n_vertex, reject_image_boundary, reject_allowable_depth,
                reject_images_mask, reject_depth_threshold);
        PrintDebug("[cam %d] %.5f percents are visible\n", c,
                   double(viscnt) / n_vertex * 100);
        fflush(stdout);
    }
    return std::move(std::make_tuple(visiblity_vertex_to_image,
                                     visiblity_image_to_vertex));
}

template <typename T>
std::tuple<bool, T> QueryImageIntensity(const Image& img,
                                        const Eigen::Vector3d& V,
                                        const PinholeCameraTrajectory& camera,
                                        int camid,
                                        int ch /*= -1*/,
                                        int image_boundary_margin /*= 10*/) {
    float u, v, depth;
    std::tie(u, v, depth) = Project3DPointAndGetUVDepth(V, camera, camid);
    if (img.TestImageBoundary(u, v, image_boundary_margin)) {
        int u_round = int(round(u));
        int v_round = int(round(v));
        if (ch == -1) {
            return std::make_tuple(true, *PointerAt<T>(img, u_round, v_round));
        } else {
            return std::make_tuple(true,
                                   *PointerAt<T>(img, u_round, v_round, ch));
        }
    } else {
        return std::make_tuple(false, 0);
    }
}

template <typename T>
std::tuple<bool, T> QueryImageIntensity(const Image& img,
                                        const ImageWarpingField& field,
                                        const Eigen::Vector3d& V,
                                        const PinholeCameraTrajectory& camera,
                                        int camid,
                                        int ch /*= -1*/,
                                        int image_boundary_margin /*= 10*/) {
    float u, v, depth;
    std::tie(u, v, depth) = Project3DPointAndGetUVDepth(V, camera, camid);
    if (img.TestImageBoundary(u, v, image_boundary_margin)) {
        Eigen::Vector2d uv_shift = field.GetImageWarpingField(u, v);
        if (img.TestImageBoundary(uv_shift(0), uv_shift(1),
                                  image_boundary_margin)) {
            int u_shift = int(round(uv_shift(0)));
            int v_shift = int(round(uv_shift(1)));
            if (ch == -1) {
                return std::make_tuple(true,
                                       *PointerAt<T>(img, u_shift, v_shift));
            } else {
                return std::make_tuple(
                        true, *PointerAt<T>(img, u_shift, v_shift, ch));
            }
        }
    }
    return std::make_tuple(false, 0);
}

void SetProxyIntensityForVertex(
        const TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_gray,
        const std::vector<ImageWarpingField>& warping_field,
        const PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        std::vector<double>& proxy_intensity,
        int image_boundary_margin) {
    auto n_vertex = mesh.vertices_.size();
    proxy_intensity.resize(n_vertex);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (auto i = 0; i < n_vertex; i++) {
        proxy_intensity[i] = 0.0;
        float sum = 0.0;
        for (auto iter = 0; iter < visiblity_vertex_to_image[i].size();
             iter++) {
            int j = visiblity_vertex_to_image[i][iter];
            float gray;
            bool valid = false;
            std::tie(valid, gray) = QueryImageIntensity<float>(
                    *images_gray[j], warping_field[j], mesh.vertices_[i],
                    camera, j, -1, image_boundary_margin);
            if (valid) {
                sum += 1.0;
                proxy_intensity[i] += gray;
            }
        }
        if (sum > 0) {
            proxy_intensity[i] /= sum;
        }
    }
}

void SetProxyIntensityForVertex(
        const TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_gray,
        const PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        std::vector<double>& proxy_intensity,
        int image_boundary_margin) {
    auto n_vertex = mesh.vertices_.size();
    proxy_intensity.resize(n_vertex);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (auto i = 0; i < n_vertex; i++) {
        proxy_intensity[i] = 0.0;
        float sum = 0.0;
        for (auto iter = 0; iter < visiblity_vertex_to_image[i].size();
             iter++) {
            int j = visiblity_vertex_to_image[i][iter];
            float gray;
            bool valid = false;
            std::tie(valid, gray) = QueryImageIntensity<float>(
                    *images_gray[j], mesh.vertices_[i], camera, j, -1,
                    image_boundary_margin);
            if (valid) {
                sum += 1.0;
                proxy_intensity[i] += gray;
            }
        }
        if (sum > 0) {
            proxy_intensity[i] /= sum;
        }
    }
}

void SetGeometryColorAverage(
        TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_color,
        const PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        int image_boundary_margin /*= 10*/) {
    auto n_vertex = mesh.vertices_.size();
    mesh.vertex_colors_.clear();
    mesh.vertex_colors_.resize(n_vertex);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < n_vertex; i++) {
        mesh.vertex_colors_[i] = Eigen::Vector3d::Zero();
        double sum = 0.0;
        for (auto iter = 0; iter < visiblity_vertex_to_image[i].size();
             iter++) {
            int j = visiblity_vertex_to_image[i][iter];
            unsigned char r_temp, g_temp, b_temp;
            bool valid = false;
            std::tie(valid, r_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], mesh.vertices_[i], camera, j, 0,
                    image_boundary_margin);
            std::tie(valid, g_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], mesh.vertices_[i], camera, j, 1,
                    image_boundary_margin);
            std::tie(valid, b_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], mesh.vertices_[i], camera, j, 2,
                    image_boundary_margin);
            float r = (float)r_temp / 255.0f;
            float g = (float)g_temp / 255.0f;
            float b = (float)b_temp / 255.0f;
            if (valid) {
                mesh.vertex_colors_[i] += Eigen::Vector3d(r, g, b);
                sum += 1.0;
            }
        }
        if (sum > 0.0) {
            mesh.vertex_colors_[i] /= sum;
        }
    }
}

double get_median(std::vector<double> scores) {
    size_t size = scores.size();
    if (size == 0) {
        return 0;  // Undefined, really.
    } else {
        sort(scores.begin(), scores.end());
        if (size % 2 == 0) {
            return (scores[size / 2 - 1] + scores[size / 2]) / 2;
        } else {
            return scores[size / 2];
        }
    }
}

void SetGeometryColorAverage(
        TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_color,
        const std::vector<ImageWarpingField>& warping_fields,
        const PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        int image_boundary_margin /*= 10*/) {
    auto n_vertex = mesh.vertices_.size();
    mesh.vertex_colors_.clear();
    mesh.vertex_colors_.resize(n_vertex);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < n_vertex; i++) {
        mesh.vertex_colors_[i] = Eigen::Vector3d::Zero();
        double sum = 0.0;
        std::vector<double> rs;
        std::vector<double> gs;
        std::vector<double> bs;
        for (auto iter = 0; iter < visiblity_vertex_to_image[i].size();
             iter++) {
            int j = visiblity_vertex_to_image[i][iter];
            unsigned char r_temp, g_temp, b_temp;
            bool valid = false;
            std::tie(valid, r_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], warping_fields[j], mesh.vertices_[i],
                    camera, j, 0, image_boundary_margin);
            std::tie(valid, g_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], warping_fields[j], mesh.vertices_[i],
                    camera, j, 1, image_boundary_margin);
            std::tie(valid, b_temp) = QueryImageIntensity<unsigned char>(
                    *images_color[j], warping_fields[j], mesh.vertices_[i],
                    camera, j, 2, image_boundary_margin);
            float r = (float)r_temp / 255.0f;
            float g = (float)g_temp / 255.0f;
            float b = (float)b_temp / 255.0f;
            if (valid) {
                mesh.vertex_colors_[i] += Eigen::Vector3d(r, g, b);
                rs.push_back(r);
                gs.push_back(g);
                bs.push_back(b);
                sum += 1.0;
            }
        }
        if (sum > 0.0) {
            mesh.vertex_colors_[i] = Eigen::Vector3d(
                    get_median(rs), get_median(gs), get_median(bs));
        }
    }
}

}  // namespace open3d
