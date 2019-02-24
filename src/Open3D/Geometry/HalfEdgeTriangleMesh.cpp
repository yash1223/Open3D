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

namespace open3d {

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

void HalfEdgeTriangleMesh::ComputeHalfEdges() {}

}  // namespace open3d
