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

#include "EigenHelperForNonRigidOptimization.h"

#include <Eigen/SparseCore>
#include <Core/Utility/Console.h>

namespace open3d {

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, double> ComputeJTJandJTr(
        std::function<void(int i,
                           Eigen::SparseMatrix<double>& J_sparse,
                           double& r)> f_jacobian_and_residual,
        int num_visable_vertex,
        int nonrigidval,
        bool verbose /*=true*/) {
    Eigen::MatrixXd JTJ(6 + nonrigidval, 6 + nonrigidval);
    Eigen::VectorXd JTr(6 + nonrigidval);
    double r2_sum = 0.0;
    JTJ.setZero();
    JTr.setZero();

    Eigen::SparseMatrix<double> J_sparse(num_visable_vertex, 6 + nonrigidval);
    double r;

    for (int i = 0; i < num_visable_vertex; i++) {
        f_jacobian_and_residual(i, J_sparse, r);
        JTr += r * J_sparse.row(i);
        r2_sum += r * r;
    }
    JTJ = Eigen::MatrixXd(J_sparse.transpose() * J_sparse);

    if (verbose) {
        PrintDebug("Residual : %.2e (# of elements : %d)\n",
                   r2_sum / (double)num_visable_vertex, num_visable_vertex);
    }
    return std::make_tuple(std::move(JTJ), std::move(JTr), r2_sum);
}

}  // namespace open3d
