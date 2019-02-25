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

void print_mesh(const TriangleMesh& mesh) {
    std::cout << "Num vertices: " << mesh.vertices_.size() << std::endl;
    for (size_t i = 0; i < mesh.vertices_.size(); ++i) {
        auto& vertex = mesh.vertices_[i];
        std::cout << "Vertex " << i << ": (" << vertex(0) << ", " << vertex(1)
                  << ", " << vertex(2) << ")" << std::endl;
    }
    std::cout << "Num triangles: " << mesh.triangles_.size() << std::endl;
    for (size_t i = 0; i < mesh.triangles_.size(); ++i) {
        auto& triangle = mesh.triangles_[i];
        std::cout << "Triangle " << i << ": (" << triangle(0) << ", "
                  << triangle(1) << ", " << triangle(2) << ")" << std::endl;
    }
}

HalfEdge::HalfEdge(const Eigen::Vector2i& vertex_indices,
                   int triangle_index,
                   int next,
                   int twin)
    : vertex_indices_(vertex_indices),
      triangle_index_(triangle_index),
      next_(next),
      twin_(twin) {}

void HalfEdgeTriangleMesh::Clear() {
    TriangleMesh::Clear();
    half_edges_.clear();
}

HalfEdgeTriangleMesh::HalfEdgeTriangleMesh(const TriangleMesh& triangle_mesh) {
    SetVerbosityLevel(VerbosityLevel::VerboseAlways);
    // Copy
    vertices_ = triangle_mesh.vertices_;
    vertex_normals_ = triangle_mesh.vertex_normals_;
    vertex_colors_ = triangle_mesh.vertex_colors_;
    triangles_ = triangle_mesh.triangles_;
    triangle_normals_ = triangle_mesh.triangle_normals_;
    adjacency_list_ = triangle_mesh.adjacency_list_;

    // Purge to remove duplications
    // std::cout << "[before]" << std::endl;
    // print_mesh(*this);
    Purge();
    // std::cout << "[after]" << std::endl;
    // print_mesh(*this);

    // If the original mesh is not a manifold, we set HalfEdgeTriangleMesh to
    // be empty. Caller to this constructor is responsible to checking
    // HalfEdgeTriangleMesh::IsEmpty().
    if (!ComputeHalfEdges()) {
        PrintError("Converting mesh to half-edge mesh filed, not manifold\n");
        Clear();
    }
}

int HalfEdgeTriangleMesh::NextNextTwinHalfEdgeIndex(int half_edge_index) const {
    const HalfEdge& curr_he = half_edges_[half_edge_index];
    int next_he_index = curr_he.next_;
    const HalfEdge& next_he = half_edges_[next_he_index];
    int next_next_he_index = next_he.next_;
    const HalfEdge& next_next_he = half_edges_[next_next_he_index];
    int next_next_twin_he_index = next_next_he.twin_;
    return next_next_twin_he_index;
}

bool HalfEdgeTriangleMesh::ComputeHalfEdges() {
    // Clean up
    // TODO: clean up all half-edge related structures
    half_edges_.clear();

    // Collect half edges
    // Check: for valid manifolds, there mustn't be duplicated half-edges
    std::unordered_map<Eigen::Vector2i, size_t,
                       hash_eigen::hash<Eigen::Vector2i>>
            vertex_indices_to_half_edge_index;

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

        if (vertex_indices_to_half_edge_index.find(he_0.vertex_indices_) !=
                    vertex_indices_to_half_edge_index.end() ||
            vertex_indices_to_half_edge_index.find(he_1.vertex_indices_) !=
                    vertex_indices_to_half_edge_index.end() ||
            vertex_indices_to_half_edge_index.find(he_2.vertex_indices_) !=
                    vertex_indices_to_half_edge_index.end()) {
            PrintError("ComputeHalfEdges failed. Duplicated half-edges.\n");
            // std::cout << he_0.vertex_indices_ << std::endl;
            // std::cout << he_1.vertex_indices_ << std::endl;
            // std::cout << he_2.vertex_indices_ << std::endl;
            return false;
        }

        half_edges_.push_back(he_0);
        half_edges_.push_back(he_1);
        half_edges_.push_back(he_2);
        vertex_indices_to_half_edge_index[he_0.vertex_indices_] = he_0_index;
        vertex_indices_to_half_edge_index[he_1.vertex_indices_] = he_1_index;
        vertex_indices_to_half_edge_index[he_2.vertex_indices_] = he_2_index;
    }

    // Fill twin half-edge. In the previous step, it is already guaranteed that
    // each half-edge can have at most one twin half-edge.
    for (size_t this_he_index = 0; this_he_index < half_edges_.size();
         this_he_index++) {
        HalfEdge& this_he = half_edges_[this_he_index];
        Eigen::Vector2i twin_end_points(this_he.vertex_indices_(1),
                                        this_he.vertex_indices_(0));
        if (vertex_indices_to_half_edge_index.find(twin_end_points) !=
            vertex_indices_to_half_edge_index.end()) {
            size_t twin_he_index =
                    vertex_indices_to_half_edge_index[twin_end_points];
            HalfEdge& twin_he = half_edges_[twin_he_index];
            this_he.twin_ = int(twin_he_index);
            twin_he.twin_ = int(this_he_index);
        }
    }

    // Get out-going half-edges from each vertex. This can be done during
    // half-edge construction. Done here for readability.
    std::vector<std::unordered_set<int>> half_edges_from_vertex(
            vertices_.size());
    for (size_t half_edge_index = 0; half_edge_index < half_edges_.size();
         half_edge_index++) {
        int src_vertex_index = half_edges_[half_edge_index].vertex_indices_(0);
        half_edges_from_vertex[src_vertex_index].insert(int(half_edge_index));
    }

    // Find ordered half-edges from each vertex by traversal. To be a valid
    // manifold, there can be at most 1 out-going half-edge for each vertex.
    std::vector<std::vector<int>> ordered_half_edges_from_vertex(
            vertices_.size());
    for (size_t vertex_index = 0; vertex_index < half_edges_from_vertex.size();
         vertex_index++) {
        size_t num_boundaries = 0;
        int init_half_edge_index = 0;
        for (const int& half_edge_index :
             half_edges_from_vertex[vertex_index]) {
            num_boundaries += int(half_edges_[half_edge_index].IsBoundary());
            init_half_edge_index = half_edge_index;
        }
        if (num_boundaries > 1) {
            PrintError("ComputeHalfEdges failed. Invalid vertex.\n");
            return false;
        }

        // Push edges to ordered_half_edges_from_vertex. If there is a boundary
        // edge, start from that; otherwise start with 0
        int curr_he_index = init_half_edge_index;
        ordered_half_edges_from_vertex[vertex_index].push_back(curr_he_index);
        int next_next_twin_he_index = NextNextTwinHalfEdgeIndex(curr_he_index);
        curr_he_index = next_next_twin_he_index;
        while (curr_he_index != -1 && curr_he_index != init_half_edge_index) {
            ordered_half_edges_from_vertex[vertex_index].push_back(
                    curr_he_index);
            next_next_twin_he_index = NextNextTwinHalfEdgeIndex(curr_he_index);
            curr_he_index = next_next_twin_he_index;
        }
    }

    return true;
}

}  // namespace open3d
