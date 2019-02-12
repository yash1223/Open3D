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

#include "ColorMapOptimization.h"
#include <iostream>
#include <unordered_map>
#include <ctime>
#include <omp.h>

#include <Core/Camera/PinholeCameraTrajectory.h>
#include <Core/ColorMap/ColorMapOptimizationJacobian.h>
#include <Core/ColorMap/ImageWarpingField.h>
#include <Core/ColorMap/TriangleMeshAndImageUtilities.h>
#include <Core/Geometry/Image.h>
#include <Core/Geometry/RGBDImage.h>
#include <Core/Geometry/TriangleMesh.h>
#include <Core/Utility/Console.h>
#include <Core/Utility/Eigen.h>
#include <IO/ClassIO/ImageWarpingFieldIO.h>
#include <IO/ClassIO/PinholeCameraTrajectoryIO.h>

namespace open3d {

namespace {

void OptimizeImageCoorNonrigid(
        const TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_gray,
        const std::vector<std::shared_ptr<Image>>& images_dx,
        const std::vector<std::shared_ptr<Image>>& images_dy,
        std::vector<ImageWarpingField>& warping_fields,
        const std::vector<ImageWarpingField>& warping_fields_init,
        PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        const std::vector<std::vector<int>>& visiblity_image_to_vertex,
        std::vector<double>& proxy_intensity,
        const ColorMapOptimizationOption& option) {
    auto n_vertex = mesh.vertices_.size();
    auto n_camera = camera.parameters_.size();
    SetProxyIntensityForVertex(mesh, images_gray, warping_fields, camera,
                               visiblity_vertex_to_image, proxy_intensity,
                               option.image_boundary_margin_);
    for (int itr = 0; itr < option.maximum_iteration_; itr++) {
        PrintDebug("[Iteration %04d] ", itr + 1);
        double iter_start_time = omp_get_wtime();
        double residual = 0.0;
        double residual_reg = 0.0;
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
        for (int c = 0; c < n_camera; c++) {
            int nonrigidval = warping_fields[c].anchor_w_ *
                              warping_fields[c].anchor_h_ * 2;
            double rr_reg = 0.0;

            Eigen::Matrix4d pose;
            pose = camera.parameters_[c].extrinsic_;

            auto intrinsic = camera.parameters_[c].intrinsic_.intrinsic_matrix_;
            auto extrinsic = camera.parameters_[c].extrinsic_;
            ColorMapOptimizationJacobian jac;
            Eigen::Matrix4d intr = Eigen::Matrix4d::Zero();
            intr.block<3, 3>(0, 0) = intrinsic;
            intr(3, 3) = 1.0;

            auto f_lambda =
                    [&](int i,
                        Eigen::SparseMatrix<double, Eigen::RowMajor>& J_sparse,
                        double& r) {
                        jac.ComputeJacobianAndResidualNonRigidSparse(
                                i, J_sparse, r, mesh, proxy_intensity,
                                images_gray[c], images_dx[c], images_dy[c],
                                warping_fields[c], warping_fields_init[c], intr,
                                extrinsic, visiblity_image_to_vertex[c],
                                option.image_boundary_margin_);
                    };
            Eigen::MatrixXd JTJ;
            Eigen::VectorXd JTr_dense;
            Eigen::SparseMatrix<double, Eigen::RowMajor> J_sparse;
            double r2;
            // std::cout << "Camera: " << c << std::endl;
            std::tie(J_sparse, JTr_dense, r2) = ComputeJTJandJTr(
                    f_lambda, visiblity_image_to_vertex[c].size(), nonrigidval,
                    false);

            // Prune zero columns
            // Method 1: keep track of non-zero columns at J matrix creation
            // Method 2: some efficient eigen method
            Eigen::SparseMatrix<double, Eigen::ColMajor> J_sparse_col_major(
                    J_sparse);
            // std::cout << "num_rows: " << J_sparse_col_major.rows() <<
            // std::endl; std::cout << "num_cols: " << J_sparse_col_major.cols()
            // << std::endl;

            // J_sparse_col_major.outerSize() == J_sparse_col_major.cols()
            std::unordered_map<size_t, size_t> map_col_to_selected_col;
            std::unordered_map<size_t, size_t> map_selected_col_to_col;
            size_t num_cols = J_sparse_col_major.cols();
            size_t selected_col = 0;
            for (size_t col = 0; col < J_sparse_col_major.outerSize(); ++col) {
                bool col_has_value = false;
                for (Eigen::SparseMatrix<double>::InnerIterator it(
                             J_sparse_col_major, col);
                     it; ++it) {
                    col_has_value = true;
                    break;
                }
                if (col_has_value) {
                    map_col_to_selected_col[col] = selected_col;
                    map_selected_col_to_col[selected_col] = col;
                    selected_col++;
                }
            }
            size_t num_selected_cols = selected_col;
            // std::cout << "num_selected_cols " << num_selected_cols <<
            // std::endl;

            // col_selection_matrix is used to map columns to selected columns
            Eigen::SparseMatrix<double> col_selection_matrix(num_cols,
                                                             num_selected_cols);
            col_selection_matrix.reserve(
                    Eigen::VectorXi::Constant(num_selected_cols, 1));
            size_t col_idx = 0;
            for (size_t selected_col = 0; selected_col < num_selected_cols;
                 ++selected_col) {
                size_t col = map_selected_col_to_col[selected_col];
                col_selection_matrix.insert(col, selected_col) = 1;
            }

            // We remove J's empty column before computing JTJ
            Eigen::SparseMatrix<double> J_selected =
                    J_sparse_col_major * col_selection_matrix;
            JTJ = Eigen::MatrixXd(J_selected.transpose() * J_selected);
            Eigen::VectorXd JTr = JTr_dense.transpose() * col_selection_matrix;

            double weight = option.non_rigid_anchor_point_weight_ *
                            visiblity_image_to_vertex[c].size() / n_vertex;
            for (size_t j = 0; j < nonrigidval; j++) {
                size_t col = 6 + j;
                if (map_col_to_selected_col.find(col) !=
                    map_col_to_selected_col.end()) {
                    double r = weight * (warping_fields[c].flow_(j) -
                                         warping_fields_init[c].flow_(j));
                    size_t selected_col = map_col_to_selected_col[col];
                    JTJ(selected_col, selected_col) += weight * weight;
                    JTr(selected_col) += weight * r;
                    rr_reg += r * r;
                }
            }

            bool success;
            Eigen::VectorXd result;

            // This one might or might not fail
            //     std::tie(success, result) = SolveLinearSystemPSD(
            //             JTJ, -JTr, /*prefer_sparse=*/false,
            //             /*check_symmetric=*/true,
            //             /*check_det=*/true, /*check_psd=*/true);

            // Ignore warnings and just get a result
            std::tie(success, result) = SolveLinearSystemPSD(
                    JTJ, -JTr, /*prefer_sparse=*/true,
                    /*check_symmetric=*/false,
                    /*check_det=*/false, /*check_psd=*/false);

            Eigen::Vector6d result_pose;
            result_pose << result.block(0, 0, 6, 1);
            auto delta = TransformVector6dToMatrix4d(result_pose);
            pose = delta * pose;

            for (size_t j = 0; j < nonrigidval; j++) {
                size_t col = 6 + j;
                if (map_col_to_selected_col.find(col) !=
                    map_col_to_selected_col.end()) {
                    size_t selected_col = map_col_to_selected_col[col];
                    warping_fields[c].flow_(j) += result(selected_col);
                }
            }
            camera.parameters_[c].extrinsic_ = pose;

#ifdef _OPENMP
#pragma omp critical
#endif
            {
                residual += r2;
                residual_reg += rr_reg;
            }
        }
        double iter_time = omp_get_wtime() - iter_start_time;
        PrintDebug("Residual error : %.6f, reg : %.6f, time: %.2f\n", residual,
                   residual_reg, iter_time);
        SetProxyIntensityForVertex(mesh, images_gray, warping_fields, camera,
                                   visiblity_vertex_to_image, proxy_intensity,
                                   option.image_boundary_margin_);
    }  // namespace
}  // namespace

void OptimizeImageCoorRigid(
        const TriangleMesh& mesh,
        const std::vector<std::shared_ptr<Image>>& images_gray,
        const std::vector<std::shared_ptr<Image>>& images_dx,
        const std::vector<std::shared_ptr<Image>>& images_dy,
        PinholeCameraTrajectory& camera,
        const std::vector<std::vector<int>>& visiblity_vertex_to_image,
        const std::vector<std::vector<int>>& visiblity_image_to_vertex,
        std::vector<double>& proxy_intensity,
        const ColorMapOptimizationOption& option) {
    int total_num_ = 0;
    auto n_camera = camera.parameters_.size();
    SetProxyIntensityForVertex(mesh, images_gray, camera,
                               visiblity_vertex_to_image, proxy_intensity,
                               option.image_boundary_margin_);
    for (int itr = 0; itr < option.maximum_iteration_; itr++) {
        PrintDebug("[Iteration %04d] ", itr + 1);
        double residual = 0.0;
        total_num_ = 0;
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
        for (int c = 0; c < n_camera; c++) {
            Eigen::Matrix4d pose;
            pose = camera.parameters_[c].extrinsic_;

            auto intrinsic = camera.parameters_[c].intrinsic_.intrinsic_matrix_;
            auto extrinsic = camera.parameters_[c].extrinsic_;
            ColorMapOptimizationJacobian jac;
            Eigen::Matrix4d intr = Eigen::Matrix4d::Zero();
            intr.block<3, 3>(0, 0) = intrinsic;
            intr(3, 3) = 1.0;

            auto f_lambda = [&](int i, Eigen::Vector6d& J_r, double& r) {
                jac.ComputeJacobianAndResidualRigid(
                        i, J_r, r, mesh, proxy_intensity, images_gray[c],
                        images_dx[c], images_dy[c], intr, extrinsic,
                        visiblity_image_to_vertex[c],
                        option.image_boundary_margin_);
            };
            Eigen::Matrix6d JTJ;
            Eigen::Vector6d JTr;
            double r2;
            std::tie(JTJ, JTr, r2) =
                    ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                            f_lambda, visiblity_image_to_vertex[c].size(),
                            false);

            bool is_success;
            Eigen::Matrix4d delta;
            std::tie(is_success, delta) =
                    SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);
            pose = delta * pose;
            camera.parameters_[c].extrinsic_ = pose;
#ifdef _OPENMP
#pragma omp critical
#endif
            {
                residual += r2;
                total_num_ += visiblity_image_to_vertex[c].size();
            }
        }
        PrintDebug("Residual error : %.6f (avg : %.6f)\n", residual,
                   residual / total_num_);
        SetProxyIntensityForVertex(mesh, images_gray, camera,
                                   visiblity_vertex_to_image, proxy_intensity,
                                   option.image_boundary_margin_);
    }
}

std::tuple<std::vector<std::shared_ptr<Image>>,
           std::vector<std::shared_ptr<Image>>,
           std::vector<std::shared_ptr<Image>>,
           std::vector<std::shared_ptr<Image>>,
           std::vector<std::shared_ptr<Image>>>
CreateGradientImages(
        const std::vector<std::shared_ptr<RGBDImage>>& images_rgbd) {
    std::vector<std::shared_ptr<Image>> images_gray;
    std::vector<std::shared_ptr<Image>> images_dx;
    std::vector<std::shared_ptr<Image>> images_dy;
    std::vector<std::shared_ptr<Image>> images_color;
    std::vector<std::shared_ptr<Image>> images_depth;
    for (auto i = 0; i < images_rgbd.size(); i++) {
        auto gray_image = CreateFloatImageFromImage(images_rgbd[i]->color_);
        auto gray_image_filtered =
                FilterImage(*gray_image, Image::FilterType::Gaussian3);
        images_gray.push_back(gray_image_filtered);
        images_dx.push_back(
                FilterImage(*gray_image_filtered, Image::FilterType::Sobel3Dx));
        images_dy.push_back(
                FilterImage(*gray_image_filtered, Image::FilterType::Sobel3Dy));
        auto color = std::make_shared<Image>(images_rgbd[i]->color_);
        auto depth = std::make_shared<Image>(images_rgbd[i]->depth_);
        images_color.push_back(color);
        images_depth.push_back(depth);
    }
    return std::move(std::make_tuple(images_gray, images_dx, images_dy,
                                     images_color, images_depth));
}

std::vector<std::shared_ptr<Image>> CreateDepthBoundaryMasks(
        const std::vector<std::shared_ptr<Image>>& images_depth,
        const ColorMapOptimizationOption& option) {
    auto n_images = images_depth.size();
    std::vector<std::shared_ptr<Image>> masks;
    for (auto i = 0; i < n_images; i++) {
        PrintDebug("[MakeDepthMasks] Image %d/%d\n", i, n_images);
        masks.push_back(CreateDepthBoundaryMask(
                *images_depth[i],
                option.depth_threshold_for_discontinuity_check_,
                option.half_dilation_kernel_size_for_discontinuity_map_));
    }
    return masks;
}

std::vector<ImageWarpingField> CreateWarpingFields(
        const std::vector<std::shared_ptr<Image>>& images,
        const ColorMapOptimizationOption& option) {
    std::vector<ImageWarpingField> fields;
    for (auto i = 0; i < images.size(); i++) {
        int width = images[i]->width_;
        int height = images[i]->height_;
        fields.push_back(ImageWarpingField(width, height,
                                           option.number_of_vertical_anchors_));
    }
    return std::move(fields);
}

}  // namespace

std::vector<std::shared_ptr<Image>> ColorMapOptimization(
        TriangleMesh& mesh,
        const std::vector<std::shared_ptr<RGBDImage>>& images_rgbd,
        PinholeCameraTrajectory& camera,
        const ColorMapOptimizationOption& option
        /* = ColorMapOptimizationOption()*/) {
    PrintDebug("[ColorMapOptimization]\n");
    std::vector<std::shared_ptr<Image>> images_gray, images_dx, images_dy,
            images_color, images_depth;
    std::tie(images_gray, images_dx, images_dy, images_color, images_depth) =
            CreateGradientImages(images_rgbd);

    PrintDebug("[ColorMapOptimization] :: MakingMasks\n");
    auto images_mask = CreateDepthBoundaryMasks(images_depth, option);

    PrintDebug("[ColorMapOptimization] :: VisibilityCheck\n");
    std::vector<std::vector<int>> visiblity_vertex_to_image;
    std::vector<std::vector<int>> visiblity_image_to_vertex;
    std::tie(visiblity_vertex_to_image, visiblity_image_to_vertex) =
            CreateVertexAndImageVisibility(
                    mesh, images_depth, images_mask, camera,
                    option.maximum_allowable_depth_,
                    option.depth_threshold_for_visiblity_check_);

    std::vector<double> proxy_intensity;
    if (option.non_rigid_camera_coordinate_) {
        PrintDebug("[ColorMapOptimization] :: Non-Rigid Optimization\n");
        auto warping_uv_ = CreateWarpingFields(images_gray, option);
        auto warping_uv_init_ = CreateWarpingFields(images_gray, option);
        OptimizeImageCoorNonrigid(
                mesh, images_gray, images_dx, images_dy, warping_uv_,
                warping_uv_init_, camera, visiblity_vertex_to_image,
                visiblity_image_to_vertex, proxy_intensity, option);
        SetGeometryColorAverage(mesh, images_color, warping_uv_, camera,
                                visiblity_vertex_to_image,
                                option.image_boundary_margin_);
    } else {
        PrintDebug("[ColorMapOptimization] :: Rigid Optimization\n");
        OptimizeImageCoorRigid(mesh, images_gray, images_dx, images_dy, camera,
                               visiblity_vertex_to_image,
                               visiblity_image_to_vertex, proxy_intensity,
                               option);
        SetGeometryColorAverage(mesh, images_color, camera,
                                visiblity_vertex_to_image,
                                option.image_boundary_margin_);
    }
    return images_mask;
}

}  // namespace open3d
