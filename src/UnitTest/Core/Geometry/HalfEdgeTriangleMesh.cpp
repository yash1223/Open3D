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

#include "Utility/UnitTest.h"
#include "Open3D/Geometry/HalfEdgeTriangleMesh.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"

#include <iostream>
#include <string>

using namespace open3d;
using namespace unit_test;

// [0: (-1, 2)]__________[1: (1, 2)]
//             \        /\
//              \  (0) /  \
//               \    / (1)\
//                \  /      \
//      [2: (0, 0)]\/________\[3: (2, 0)]
TriangleMesh get_mesh_two_triangles() {
    std::vector<Eigen::Vector3d> vertices{
            Eigen::Vector3d(-1, 2, 0), Eigen::Vector3d(1, 2, 0),
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 0, 0)};
    std::vector<Eigen::Vector3i> triangles{Eigen::Vector3i(0, 2, 1),
                                           Eigen::Vector3i(1, 2, 3)};
    TriangleMesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

// [0: (-1, 2)]__________[1: (1, 2)]
//             \        /\
//              \  (0) /  \
//               \    / (1)\
//                \  /      \
//      [2: (0, 0)]\/________\[3: (2, 0)]
//
// Non-manifold: triangle (1) is flipped
TriangleMesh get_mesh_two_triangles_flipped() {
    std::vector<Eigen::Vector3d> vertices{
            Eigen::Vector3d(-1, 2, 0), Eigen::Vector3d(1, 2, 0),
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 0, 0)};
    std::vector<Eigen::Vector3i> triangles{Eigen::Vector3i(0, 2, 1),
                                           Eigen::Vector3i(1, 3, 2)};
    TriangleMesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

//  [0: (-1, 2)]__________[1: (1, 2)]
//              \        /
//               \  (0) /
//                \    /
//                 \  /
//                  \/ [2: (0, 0)]
//                  /\
//                 /  \
//                /    \
//               /  (1) \
//              /________\
// [3: (-1, -2)]          [4: (1, -2)]
//
// Non-manifold
TriangleMesh get_mesh_two_triangles_invalid_vertex() {
    std::vector<Eigen::Vector3d> vertices{
            Eigen::Vector3d(-1, 2, 0), Eigen::Vector3d(1, 2, 0),
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-1, -2, 0),
            Eigen::Vector3d(1, -2, 0)};
    std::vector<Eigen::Vector3i> triangles{Eigen::Vector3i(0, 2, 1),
                                           Eigen::Vector3i(2, 3, 4)};
    TriangleMesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

//          [0: (-1, 2)]__________[1: (1, 2)]
//                     /\        /\
//                    /  \  (1) /  \
//                   / (0)\    / (2)\
//                  /      \  /      \
//     [2: (-2, 0)]/____[3: (O, 0)]___\[4: (2, 0)]
//                 \        /\        /
//                  \  (3) /  \  (5) /
//                   \    /    \    /
//                    \  /  (4) \  /
//                     \/________\/
//         [5: (-1, -2)]          [6: (1, -2)]
TriangleMesh get_mesh_hexagon() {
    std::vector<Eigen::Vector3d> vertices{
            Eigen::Vector3d(-1, 2, 0), Eigen::Vector3d(1, 2, 0),
            Eigen::Vector3d(-2, 0, 0), Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(2, 0, 0),  Eigen::Vector3d(-1, -2, 0),
            Eigen::Vector3d(1, -2, 0)};
    std::vector<Eigen::Vector3i> triangles{
            Eigen::Vector3i(0, 2, 3), Eigen::Vector3i(0, 3, 1),
            Eigen::Vector3i(1, 3, 4), Eigen::Vector3i(2, 5, 3),
            Eigen::Vector3i(3, 5, 6), Eigen::Vector3i(3, 6, 4)};
    TriangleMesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

//          [0: (-1, 2)]__________[1: (1, 2)]
//                     /\        /\
//                    /  \  (1) /  \
//                   / (0)\    / (2)\
//                  /      \  /      \
//     [2: (-2, 0)]/____[3: (O, 0)]___\[4: (2, 0)]
//                 \        /\
//                  \  (3) /  \
//                   \    /    \
//                    \  /  (4) \
//                     \/________\
//         [5: (-1, -2)]          [6: (1, -2)]
TriangleMesh get_mesh_partial_hexagon() {
    std::vector<Eigen::Vector3d> vertices{
            Eigen::Vector3d(-1, 2, 0), Eigen::Vector3d(1, 2, 0),
            Eigen::Vector3d(-2, 0, 0), Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(2, 0, 0),  Eigen::Vector3d(-1, -2, 0),
            Eigen::Vector3d(1, -2, 0)};
    std::vector<Eigen::Vector3i> triangles{
            Eigen::Vector3i(0, 2, 3), Eigen::Vector3i(0, 3, 1),
            Eigen::Vector3i(1, 3, 4), Eigen::Vector3i(2, 5, 3),
            Eigen::Vector3i(3, 5, 6)};
    TriangleMesh mesh;
    mesh.vertices_ = vertices;
    mesh.triangles_ = triangles;
    return mesh;
}

void assert_half_edge_vertex(const HalfEdgeTriangleMesh& mesh,
                             int half_edge_index,
                             const Eigen::Vector2i expected_vertex_indices) {
    EXPECT_TRUE(mesh.half_edges_[half_edge_index].vertex_indices_.isApprox(
            expected_vertex_indices));
}

TEST(HalfEdgeTriangleMesh, Constructor_TwoTriangles) {
    TriangleMesh mesh = get_mesh_two_triangles();
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_FALSE(he_mesh.IsEmpty());
}

TEST(HalfEdgeTriangleMesh, Constructor_TwoTrianglesFlipped) {
    TriangleMesh mesh = get_mesh_two_triangles_flipped();
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_TRUE(he_mesh.IsEmpty());  // Non-manifold
}

TEST(HalfEdgeTriangleMesh, Constructo_rTwoTrianglesInvalidVertex) {
    TriangleMesh mesh = get_mesh_two_triangles_invalid_vertex();
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_TRUE(he_mesh.IsEmpty());  // Non-manifold
}

TEST(HalfEdgeTriangleMesh, Constructor_Hexagon) {
    TriangleMesh mesh = get_mesh_hexagon();
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_FALSE(he_mesh.IsEmpty());
}

TEST(HalfEdgeTriangleMesh, Constructor_PartialHexagon) {
    TriangleMesh mesh = get_mesh_partial_hexagon();
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_FALSE(he_mesh.IsEmpty());
}

TEST(HalfEdgeTriangleMesh, Constructor_Sphere) {
    TriangleMesh mesh;
    ReadTriangleMesh(std::string(TEST_DATA_DIR) + "/sphere.ply", mesh);
    HalfEdgeTriangleMesh he_mesh(mesh);
    EXPECT_FALSE(he_mesh.IsEmpty());
}

TEST(HalfEdgeTriangleMesh, OrderedHalfEdgesFromVertex_TwoTriangles) {
    HalfEdgeTriangleMesh mesh(get_mesh_two_triangles());
    EXPECT_FALSE(mesh.IsEmpty());
    std::vector<int> ordered_half_edges;

    // Vertex 0
    ordered_half_edges = mesh.ordered_half_edge_from_vertex_[0];
    EXPECT_EQ(ordered_half_edges.size(), 1);
    assert_half_edge_vertex(mesh, ordered_half_edges[0], Eigen::Vector2i(0, 2));
    // Vertex 1
    ordered_half_edges = mesh.ordered_half_edge_from_vertex_[1];
    EXPECT_EQ(ordered_half_edges.size(), 2);
    assert_half_edge_vertex(mesh, ordered_half_edges[0], Eigen::Vector2i(1, 0));
    assert_half_edge_vertex(mesh, ordered_half_edges[1], Eigen::Vector2i(1, 2));
    // Vertex 2
    ordered_half_edges = mesh.ordered_half_edge_from_vertex_[2];
    EXPECT_EQ(ordered_half_edges.size(), 2);
    assert_half_edge_vertex(mesh, ordered_half_edges[0], Eigen::Vector2i(2, 3));
    assert_half_edge_vertex(mesh, ordered_half_edges[1], Eigen::Vector2i(2, 1));
    // Vertex 3
    ordered_half_edges = mesh.ordered_half_edge_from_vertex_[3];
    EXPECT_EQ(ordered_half_edges.size(), 1);
    assert_half_edge_vertex(mesh, ordered_half_edges[0], Eigen::Vector2i(3, 1));
}
