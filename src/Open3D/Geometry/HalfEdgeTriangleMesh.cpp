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

#include "Open3D/Geometry/HalfEdgeTriangleMesh.h"
#include "Open3D/Utility/Helper.h"
#include "Open3D/Utility/Console.h"

namespace open3d {

// class HalfEdge {
// public:
//     // Index of the next HalfEdge
//     int next_ = -1;
//     // Index of the twin HalfEdge
//     int twin_ = -1;
//     // Index of the ordered vertices forming this half edge
//     Eigen::Vector2i vertex_indices = Eigen::Vector2i(-1, -1);
//     // Index of the triangle containing this half edge
//     int triangle_index = -1;
// };

HalfEdge::HalfEdge(int next,
                   int twin,
                   const Eigen::Vector2i& vertex_indices,
                   int triangle_index)
    : next_(next),
      twin_(twin),
      vertex_indices_(vertex_indices),
      triangle_index_(triangle_index) {}

HalfEdgeTriangleMesh::HalfEdgeTriangleMesh(const TriangleMesh& triangle_mesh) {
    // Copy
    vertices_ = triangle_mesh.vertices_;
    vertex_normals_ = triangle_mesh.vertex_normals_;
    vertex_colors_ = triangle_mesh.vertex_colors_;
    triangles_ = triangle_mesh.triangles_;
    triangle_normals_ = triangle_mesh.triangle_normals_;
    adjacency_list_ = triangle_mesh.adjacency_list_;
    Purge();
    ComputeHalfEdges();
}

bool HalfEdgeTriangleMesh::ComputeHalfEdges() {
    SetVerbosityLevel(VerbosityLevel::VerboseAlways);

    // Clean up
    // TODO: clean up all half-edge related structures
    half_edges_.clear();

    // Collect half edges
    // For valid manifolds, there mustn't be duplicated half-edges and each
    // half-edge can have at most one twin half-edge
    std::unordered_map<Eigen::Vector2i, HalfEdge,
                       hash_eigen::hash<Eigen::Vector2i>>
            map_end_points_to_half_edge;

    for (size_t triangle_index = 0; triangle_index < triangles_.size();
         triangle_index++) {
        const Eigen::Vector3i& triangle = triangles_[triangle_index];
        if (map_end_points_to_half_edge.count(
                    Eigen::Vector2i(triangle(0), triangle(1))) != 0 ||
            map_end_points_to_half_edge.count(
                    Eigen::Vector2i(triangle(1), triangle(2))) != 0 ||
            map_end_points_to_half_edge.count(
                    Eigen::Vector2i(triangle(2), triangle(0))) != 0) {
            PrintError(
                    "[ComputeHalfEdges] failed because duplicated half-edges"
                    "are found\n");
            return false;
        }

        size_t num_half_edges = half_edges_.size();
        // Edge 0->1
        half_edges_.push_back(HalfEdge(
                num_half_edges + 1, -1,
                Eigen::Vector2i(triangle(0), triangle(1)), triangle_index));
        // Edge 1->2
        half_edges_.push_back(HalfEdge(
                num_half_edges + 2, -1,
                Eigen::Vector2i(triangle(1), triangle(2)), triangle_index));
        // Edge 2->0
        half_edges_.push_back(HalfEdge(
                num_half_edges + 0, -1,
                Eigen::Vector2i(triangle(2), triangle(0)), triangle_index));
    }
    return true;
}

}  // namespace open3d
