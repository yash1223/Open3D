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
//     HalfEdge(const Eigen::Vector2i& vertex_indices,
//              int triangle_index,
//              int next,
//              int twin);

// public:
//     // Index of the next HalfEdge
//     int next_ = -1;
//     // Index of the twin HalfEdge
//     int twin_ = -1;
//     // Index of the ordered vertices forming this half edge
//     Eigen::Vector2i vertex_indices_ = Eigen::Vector2i(-1, -1);
//     // Index of the triangle containing this half edge
//     int triangle_index_ = -1;
// };

HalfEdge::HalfEdge(const Eigen::Vector2i& vertex_indices,
                   int triangle_index,
                   int next,
                   int twin)
    : vertex_indices_(vertex_indices),
      triangle_index_(triangle_index),
      next_(next),
      twin_(twin) {}

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
    // Check: for valid manifolds, there mustn't be duplicated half-edges
    std::unordered_map<Eigen::Vector2i, size_t,
                       hash_eigen::hash<Eigen::Vector2i>>
            map_end_points_to_half_edge_index;

    for (size_t triangle_index = 0; triangle_index < triangles_.size();
         triangle_index++) {
        const Eigen::Vector3i& triangle = triangles_[triangle_index];
        size_t num_half_edges = half_edges_.size();

        size_t he_0_index = num_half_edges;
        size_t he_1_index = num_half_edges + 1;
        size_t he_2_index = num_half_edges + 2;
        HalfEdge he_0(Eigen::Vector2i(triangle(0), triangle(1)), triangle_index,
                      he_1_index, -1);
        HalfEdge he_1(Eigen::Vector2i(triangle(1), triangle(2)), triangle_index,
                      he_2_index, -1);
        HalfEdge he_2(Eigen::Vector2i(triangle(2), triangle(0)), triangle_index,
                      he_0_index, -1);

        if (map_end_points_to_half_edge_index.find(he_0.vertex_indices_) !=
                    map_end_points_to_half_edge_index.end() ||
            map_end_points_to_half_edge_index.find(he_1.vertex_indices_) !=
                    map_end_points_to_half_edge_index.end() ||
            map_end_points_to_half_edge_index.find(he_2.vertex_indices_) !=
                    map_end_points_to_half_edge_index.end()) {
            PrintError(
                    "[ComputeHalfEdges] failed because duplicated half-edges"
                    "are found\n");
            return false;
        }

        half_edges_.push_back(he_0);
        half_edges_.push_back(he_1);
        half_edges_.push_back(he_2);
        map_end_points_to_half_edge_index[he_0.vertex_indices_] = he_0_index;
        map_end_points_to_half_edge_index[he_1.vertex_indices_] = he_1_index;
        map_end_points_to_half_edge_index[he_2.vertex_indices_] = he_2_index;
    }

    // Fill twin half-edge. In the previous step, it is already guaranteed that
    // each half-edge can have at most one twin half-edge.
    for (size_t half_edge_index = 0; half_edge_index < half_edges_.size();
         half_edge_index++) {
        HalfEdge& half_edge = half_edges_[half_edge_index];
        Eigen::Vector2i twin_end_points(half_edge.vertex_indices_(1),
                                        half_edge.vertex_indices_(0));
        if (map_end_points_to_half_edge_index.find(twin_end_points) !=
            map_end_points_to_half_edge_index.end()) {
            size_t twin_half_edge_index =
                    map_end_points_to_half_edge_index[twin_end_points];
            half_edge.twin_ = int(twin_half_edge_index);
            half_edges_[twin_half_edge_index].twin_ = int(half_edge_index);
        }
    }

    return true;
}

}  // namespace open3d
