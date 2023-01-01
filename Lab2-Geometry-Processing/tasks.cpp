#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const& input, Engine::SurfaceMesh& output, std::uint32_t numIterations) {
        // your code here
        Engine::SurfaceMesh tmp = input;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            output = Engine::SurfaceMesh();
            DCEL links;
            links.AddFaces(tmp.Indices);
            if (! links.IsValid()) return;

            // Old vertex
            for (std::size_t i = 0; i < tmp.Positions.size(); ++i) {
                DCEL::Vertex v = links.GetVertex(i);
                std::vector<std::uint32_t> neighbours = v.GetNeighbors();
                std::size_t                n          = neighbours.size();
                float                      u          = (n == 3) ? (3.0 / 16) : (3.0 / (8 * n));
                glm::vec3                  new_v      = (1 - n * u) * tmp.Positions[i];
                for (std::size_t j = 0; j < neighbours.size(); ++j) {
                    new_v += u * tmp.Positions[neighbours[j]];
                }
                output.Positions.push_back(new_v);
            }

            // New vertex
            typedef std::pair<std::uint32_t, std::uint32_t> halfedge;
            std::map<halfedge, std::uint32_t>               edge_map {};
            for (DCEL::HalfEdge const* e : links.GetEdges()) {
                // Save index
                halfedge cur_halfedge = halfedge(e->From(), e->To());
                if (edge_map[cur_halfedge]) continue;
                edge_map[cur_halfedge] = output.Positions.size();
                edge_map[halfedge(e->To(), e->From())] = output.Positions.size();

                // Calculate new vertex
                glm::vec3 new_v;
                std::uint32_t from_idx = e->From();
                std::uint32_t to_idx   = e->To();
                glm::vec3 from_v   = tmp.Positions[from_idx];
                glm::vec3 to_v     = tmp.Positions[to_idx];
                if (links.GetVertex(from_idx).IsSide() && links.GetVertex(to_idx).IsSide()) {
                    new_v = (from_v + to_v) / 2.0f; // Side Edge
                } else {
                    std::uint32_t side1_idx = e->OppositeVertex();
                    std::uint32_t side2_idx = e->PairEdge()->OppositeVertex();
                    glm::vec3     side1_v   = tmp.Positions[side1_idx];
                    glm::vec3     side2_v   = tmp.Positions[side2_idx];
                    new_v                   = (from_v + to_v) * 3.0f / 8.0f + (side1_v + side2_v) / 8.0f;
                }
                output.Positions.push_back(new_v);
            }

            // Connect vertex
            for (int i = 0; i < tmp.Indices.size(); i += 3) {
                std::uint32_t v0_idx = tmp.Indices[i];
                std::uint32_t v1_idx = tmp.Indices[i + 1];
                std::uint32_t v2_idx = tmp.Indices[i + 2];
                // Each face is splited into 4
                std::uint32_t v01_idx = edge_map[halfedge(v0_idx, v1_idx)];
                std::uint32_t v12_idx = edge_map[halfedge(v1_idx, v2_idx)];
                std::uint32_t v20_idx = edge_map[halfedge(v2_idx, v0_idx)];
                
                output.Indices.push_back(v0_idx);
                output.Indices.push_back(v01_idx);
                output.Indices.push_back(v20_idx);

                output.Indices.push_back(v1_idx);
                output.Indices.push_back(v12_idx);
                output.Indices.push_back(v01_idx);

                output.Indices.push_back(v2_idx);
                output.Indices.push_back(v20_idx);
                output.Indices.push_back(v12_idx);

                output.Indices.push_back(v01_idx);
                output.Indices.push_back(v12_idx);
                output.Indices.push_back(v20_idx);
            }

            tmp = output;
        }
        output = tmp;
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // your code here
        DCEL links;
        links.AddFaces(input.Indices);

        std::vector<std::uint32_t> side_point_indices_flag(input.Positions.size());
        std::vector<std::uint32_t> side_point_indices_vec;
        std::int32_t               side_point_num = 0;
        for (std::size_t i = 0; i < input.Positions.size(); ++i) {
            if (links.GetVertex(i).IsSide()) {
                side_point_indices_flag[i] = 1;
                side_point_indices_vec.push_back(i);
                ++side_point_num;
                break;
            }
        }
        while (1) {
            std::int32_t cur_point_idx = side_point_indices_vec[side_point_num - 1];
            std::pair<std::int32_t, std::int32_t> neighbour     = links.GetVertex(cur_point_idx).GetSideNeighbors();
            if (!side_point_indices_flag[neighbour.first]) {
                side_point_indices_flag[neighbour.first] = 1;
                side_point_indices_vec.push_back(neighbour.first);
                ++side_point_num;
            } else if (!side_point_indices_flag[neighbour.second]) {
                side_point_indices_flag[neighbour.second] = 1;
                side_point_indices_vec.push_back(neighbour.second);
                ++side_point_num;
            } else break;
        }

        output.Indices = input.Indices;
        output.Positions = input.Positions;
        output.TexCoords = std::vector<glm::vec2>(output.Positions.size());

        // Init each vertex to [0.5, 0.5]
        for (std::int32_t i = 0; i < output.Positions.size(); ++i) {
            output.TexCoords[i] = glm::vec2(0.5f, 0.5f);
        }

        // Init side as square
        //std::int32_t p1 = side_point_num / 4;
        //output.TexCoords[side_point_indices_vec[p1]] = glm::vec2(1.0f, 0.0f);
        //
        //for (std::int32_t i = 1; i < p1; ++i) {
        //    output.TexCoords[side_point_indices_vec[i]] = glm::vec2(i * 1.0f / p1, 0.0f);
        //}

        //std::int32_t p2      = side_point_num / 2;
        //output.TexCoords[side_point_indices_vec[p2]] = glm::vec2(1.0f, 1.0f);
        //for (std::int32_t i = p1 + 1; i < p2; ++i) {
        //    output.TexCoords[side_point_indices_vec[i]] = glm::vec2(1.0f, (i - p1) * 1.0f / (p2 - p1));
        //}
        //
        //std::int32_t p3      = side_point_num * 3 / 4;
        //output.TexCoords[side_point_indices_vec[p3]] = glm::vec2(0.0f, 1.0f);
        //for (std::int32_t i = p2 + 1; i < p3; ++i) {
        //    output.TexCoords[side_point_indices_vec[i]] = glm::vec2((p3 - i) * 1.0f / (p3 - p2), 1.0f);
        //}

        //output.TexCoords[side_point_indices_vec[0]] = glm::vec2(0.0f, 0.0f);
        //for (std::int32_t i = p3 + 1; i < side_point_num; ++i) {
        //    output.TexCoords[side_point_indices_vec[i]] = glm::vec2(0.0f, (side_point_num - i) * 1.0f / (side_point_num - p3));
        //}

        // Init side as circle
        for (std::int32_t i = 0; i < side_point_num; ++i) {
            float theta = i * (2 * acos(-1.0)) / side_point_num;
            float x     = 0.5 + 0.5 * cos(theta);
            float y     = 0.5 + 0.5 * sin(theta);
            output.TexCoords[side_point_indices_vec[i]] = glm::vec2(x, y);
        }

        for (std::int32_t iter = 0; iter < numIterations; ++iter) {
            for (std::int32_t i = 0; i < output.Positions.size(); ++i) {
                if (side_point_indices_flag[i]) continue;
                auto neighbours = links.GetVertex(i).GetNeighbors();
                output.TexCoords[i] = glm::vec2(0.0f, 0.0f);
                for (int j = 0; j < neighbours.size(); ++j) {
                    output.TexCoords[i] += output.TexCoords[neighbours[j]];
                }
                output.TexCoords[i] /= (1.0f * neighbours.size());
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float valid_pair_threshold, float simplification_ratio) {
        // your code here
        DCEL links;
        links.AddFaces(input.Indices);

        // Step 1. Calculating Q
        std::vector<glm::mat4> Q(input.Positions.size());
        for (std::uint32_t i = 0; i < input.Positions.size(); ++i) {
            std::vector<DCEL::Triangle const *> faces = links.GetVertex(i).GetFaces();
            for (auto f : faces) {
                std::uint32_t const * p0_idx = f->Indices(0);
                std::uint32_t const * p1_idx = f->Indices(1);
                std::uint32_t const * p2_idx = f->Indices(2);
                glm::vec3             p0     = input.Positions[*p0_idx];
                glm::vec3             p1     = input.Positions[*p1_idx];
                glm::vec3             p2     = input.Positions[*p2_idx];
                // Compute p = [a,b,c,d], where ax+by+cz+d=0 is the plane of the triangle
                // and a*a+b*b+c*c=1
                // (p0-p1) x (p0-p2) gives the normal vector of the plane
                glm::vec3 p01 = p0 - p1;
                glm::vec3 p02 = p0 - p2;
                glm::vec3 norm {};
                norm[0] = p01[1] * p02[2] - p01[2] * p02[1];
                norm[1] = p01[2] * p02[0] - p01[0] * p02[2];
                norm[2] = p01[0] * p02[1] - p01[1] * p02[0];
                // normalizaed norm gives [a,b,c]
                norm /= sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]);
                // calculate d
                float     d = -(norm[0] * p0[0] + norm[1] * p0[1] + norm[2] * p0[2]);
                float     a = norm[0], b = norm[1], c = norm[2];
                glm::mat4 Kp = glm::mat4(
                    a * a, a * b, a * c, a * d, b * a, b * b, b * c, b * d, c * a, c * b, c * c, c * d, d * a, d * b, d * c, d * d);
                Q[i] += Kp;
            }
        }

        // Step 2. Pair selection
        typedef std::pair<std::int32_t, std::int32_t> Pair;
        std::vector<Pair>                             valid_pairs;
        std::set<Pair>                                valid_pairs_set;
        // For all edges
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            std::int32_t from = e->From(), to = e->To();
            if (valid_pairs_set.count(Pair(from, to))) continue;
            valid_pairs.push_back(Pair(from, to));
            valid_pairs_set.insert(Pair(from, to));
            valid_pairs_set.insert(Pair(to, from));
        }
        // For all ||vi-vj|| < t
        if (valid_pair_threshold) {
            for (std::int32_t i = 0; i < input.Positions.size(); ++i) {
                for (std::int32_t j = i + 1; j < input.Positions.size(); ++j) {
                    if (valid_pairs_set.count(Pair(i, j))) continue;
                    glm::vec3 delta_v = input.Positions[i] - input.Positions[j];
                    float     dist    = sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1] + delta_v[2] * delta_v[2]);
                    if (dist < valid_pair_threshold) {
                        valid_pairs.push_back(Pair(i, j));
                        valid_pairs_set.insert(Pair(i, j));
                        valid_pairs_set.insert(Pair(j, i));
                    }
                }
            }
        }

        // Step 3. Compute cost
        std::vector<float> costs(valid_pairs.size());
        auto is_inf = [](const float & val) {
            return ! (-FLT_MAX <= val && val <= FLT_MAX);
        };
        auto is_nan = [](const float & val) {
            return val != val;
        };
        auto is_valid = [&is_inf, &is_nan](const float & val) {
            return ! is_inf(val) && ! is_nan(val);
        };
        auto calculate_v_bar = [&Q, &is_valid, &input](Pair _p) -> glm::vec4 {
            glm::mat4 Q_bar  = Q[_p.first] + Q[_p.second];
            glm::mat4 Q_grad = glm::mat4(
                Q_bar[0][0], Q_bar[0][1], Q_bar[0][2], Q_bar[0][3], Q_bar[0][1], Q_bar[1][1], Q_bar[1][2], Q_bar[1][3], Q_bar[0][2], Q_bar[1][2], Q_bar[2][2], Q_bar[2][3], 0.0f, 0.0f, 0.0f, 1.0f);
            glm::mat4 Q_inv = glm::inverse(Q_grad);
            // Check whether Q_inv is legal
            bool Q_inv_legal = is_valid(Q_inv[0][0]);
            for (std::int32_t i = 0; i < 4; ++i) {
                for (std::int32_t j = 0; j < 4; ++j) {
                    if (! is_valid(Q_inv[i][j])) {
                        Q_inv_legal = false;
                        break;
                    }
                }
                if (! Q_inv_legal) break;
            }
            glm::vec4 v_bar {};
            if (Q_inv_legal) {
                // Q_grad is invertible
                v_bar[0] = Q_inv[0][3];
                v_bar[1] = Q_inv[1][3];
                v_bar[2] = Q_inv[2][3];
                v_bar[3] = Q_inv[3][3];
            } else {
                glm::vec4 vertex_1 {};
                vertex_1[0]        = input.Positions[_p.first][0];
                vertex_1[1]        = input.Positions[_p.first][1];
                vertex_1[2]        = input.Positions[_p.first][2];
                vertex_1[3]        = 1.0f;
                glm::vec4 vertex_2 {};
                vertex_2[0]      = input.Positions[_p.second][0];
                vertex_2[1]      = input.Positions[_p.second][1];
                vertex_2[2]      = input.Positions[_p.second][2];
                vertex_2[3]      = 1.0f;
                float     val_11 = 0, val_12 = 0, val_22 = 0;
                // Compute 
                // val_11 = v_1^T Q_bar v_1
                // val_12 = v_1^T Q_bar v_2 = v_2^T Q_bar v_1
                // val_22 = v_2 ^ T Q_bar v_2
                for (std::int32_t i = 0; i < 4; ++i) {
                    for (std::int32_t j = 0; j < 4; ++j) {
                        val_11 += Q_bar[i][j] * vertex_1[i] * vertex_1[j];
                        val_12 += Q_bar[i][j] * vertex_1[i] * vertex_2[j];
                        val_22 += Q_bar[i][j] * vertex_2[i] * vertex_2[j];
                    }
                }
                // To minimize lambda^2 * val_11 + 2 * lambda * (1-lambda) * val_12 + (1-lambda)^2 * val_22
                // (Equivalent) To minimize A * lambda^2 + 2 * B * lambda + C
                // where A = val_11+val_22-2*v_12, B = val_12-val_22, C = val_22
                float _A = val_11 + val_22 - 2 * val_12;
                float _B = val_12 - val_22, _C = val_22;
                float lambda = 0;
                if (_A == 0) {
                    if (_B > 0) lambda = 0;
                    else if (_B == 0) lambda = 0.5;
                    else if (_B < 0) lambda = 1;
                } else if (_A >0 ) {
                    float axis_of_symmetry = -_B / _A;
                    if (axis_of_symmetry >= 0 && axis_of_symmetry <= 1) lambda = axis_of_symmetry;
                    else if (axis_of_symmetry < 0) lambda = 0;
                    else if (axis_of_symmetry > 1) lambda = 1;
                } else if (_A < 0) {
                    float axis_of_symmetry = -_B / _A;
                    if (axis_of_symmetry >= 0.5) lambda = 0;
                    else if (axis_of_symmetry < 0.5) lambda = 1;
                }
                v_bar = lambda * vertex_1 + (1 - lambda) * vertex_2;
            }
            return v_bar;
        };
        auto calculate_cost = [&Q, &calculate_v_bar](Pair _p) -> float {
            glm::vec4 v_bar = calculate_v_bar(_p);
            glm::mat4 Q_bar = Q[_p.first] + Q[_p.second];
            float     cost  = 0;
            for (std::int32_t i = 0; i < 4; ++i) {
                for (std::int32_t j = 0; j < 4; ++j) {
                    cost += Q_bar[i][j] * v_bar[i] * v_bar[j];
                }
            }
            return cost;
        };
        for (std::int32_t i = 0; i < valid_pairs.size(); ++i) {
            costs[i] = calculate_cost(valid_pairs[i]);
        }

        // Step 4. Pair contract
        // Build union find set
        std::vector<std::int32_t> parent(input.Positions.size());
        for (std::int32_t i = 0; i < input.Positions.size(); ++i) {
            parent[i] = i;
        }
        std::function<std::int32_t(std::int32_t)> get_parent = [&parent, &get_parent](std::int32_t x) -> std::int32_t {
            return parent[x] == x ? x : parent[x] = get_parent(parent[x]);
        };
        // New vertex
        std::vector<glm::vec3> new_vertex = input.Positions;
        // Simplification
        std::int32_t remove_iteration = (1 - simplification_ratio) * new_vertex.size();
        for (std::int32_t iter = 0; iter < remove_iteration; ++iter) {
            // Find index of min cost pair
            float        min_cost     = FLT_MAX;
            std::int32_t min_cost_idx = -1;
            for (std::int32_t i = 0; i < costs.size(); ++i) {
                if (costs[i] < min_cost) {
                    min_cost     = costs[i];
                    min_cost_idx = i;
                }
            }
            if (min_cost_idx == -1) break;
            // Remove min cost pair
            costs[min_cost_idx] = FLT_MAX;
            // Take the min cost pair
            // Update v, Q for get_parent(pair.first)
            Pair         min_cost_pair = valid_pairs[min_cost_idx];
            std::int32_t root_first    = get_parent(min_cost_pair.first);
            std::int32_t root_second   = get_parent(min_cost_pair.second);
            new_vertex[root_first]     = calculate_v_bar(Pair(root_first, root_second));
            Q[root_first] += Q[root_second];
            parent[root_second] = root_first;
            // Update costs involving root_first
            for (std::int32_t i = 0; i < costs.size(); ++i) {
                if (costs[i] == FLT_MAX) continue; // Removed pairs
                Pair         pair_i = valid_pairs[i];
                std::int32_t root_1 = get_parent(pair_i.first);
                std::int32_t root_2 = get_parent(pair_i.second);
                if (root_1 == root_2) {
                    costs[i] = FLT_MAX;
                    continue;
                }
                if (root_1 != root_first && root_2 != root_first) continue;
                costs[i] = calculate_cost(Pair(root_1, root_2));
            }
        }

        // Step 5. Create new mesh
        // Vertex of output
        std::map<std::int32_t, std::int32_t> indices_in_new_mesh;
        for (std::int32_t i = 0; i < new_vertex.size(); ++i) {
            if (parent[i] == i) {
                output.Positions.push_back(new_vertex[i]);
                indices_in_new_mesh[i] = output.Positions.size()-1;
            }
        }
        // Faces of output
        typedef std::tuple<std::int32_t, std::int32_t, std::int32_t> Facetype;
        std::set<Facetype>                                           face_set {};
        for (std::int32_t i = 0; i < input.Indices.size(); i += 3) {
            std::int32_t idx0 = input.Indices[i];
            std::int32_t idx1 = input.Indices[i + 1];
            std::int32_t idx2 = input.Indices[i + 2];
            std::int32_t rt0  = get_parent(idx0);
            std::int32_t rt1  = get_parent(idx1);
            std::int32_t rt2  = get_parent(idx2);
            // After simplification, in the same set
            if (rt0 == rt1) continue;
            if (rt1 == rt2) continue;
            if (rt2 == rt0) continue;
            // Avoid repetition
            if (face_set.count(Facetype(rt0, rt1, rt2))) continue;
            // Still a face
            output.Indices.push_back(indices_in_new_mesh[rt0]);
            output.Indices.push_back(indices_in_new_mesh[rt1]);
            output.Indices.push_back(indices_in_new_mesh[rt2]);
            // Avoid repetition
            face_set.insert(Facetype(rt0, rt1, rt2));
            face_set.insert(Facetype(rt0, rt2, rt1));
            face_set.insert(Facetype(rt1, rt0, rt2));
            face_set.insert(Facetype(rt1, rt2, rt0));
            face_set.insert(Facetype(rt2, rt1, rt0));
            face_set.insert(Facetype(rt2, rt0, rt1));
        }
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // your code here
        DCEL links;
        links.AddFaces(input.Indices); 
        // Relations between vertex indices and faces never changed
        // So "AddFaces" outside the loop is OK
        if (! links.IsValid()) return;

        Engine::SurfaceMesh tmp = input;

        for (std::int32_t iter = 0; iter < numIterations; ++iter) {
            output = Engine::SurfaceMesh();

            for (std::size_t i = 0; i < tmp.Positions.size(); ++i) {
                DCEL::Vertex v = links.GetVertex(i);
                
                glm::vec3 v_star {};
                float     sum_weight = 0;
                if (useUniformWeight) {
                    std::vector<std::uint32_t> neighbours = v.GetNeighbors();
                    std::size_t                n          = neighbours.size();
                    sum_weight                            = n;
                    for (std::size_t j = 0; j < n; ++j) {
                        v_star += tmp.Positions[neighbours[j]];
                    }
                } else {
                    std::vector<DCEL::Triangle const *> faces = v.GetFaces();
                    for (auto f : faces) {
                        std::uint32_t const * p0_idx = f->Indices(0);
                        std::uint32_t const * p1_idx = f->Indices(1);
                        std::uint32_t const * p2_idx = f->Indices(2);
                        std::uint32_t         index_1 = 0, index_2 = 0;
                        if (*p0_idx == i) {
                            index_1 = *p1_idx;
                            index_2 = *p2_idx;
                        } else if (*p1_idx == i) {
                            index_1 = *p0_idx;
                            index_2 = *p2_idx;
                        } else if (*p2_idx == i) {
                            index_1 = *p0_idx;
                            index_2 = *p1_idx;
                        }
                        glm::vec3 p0 = tmp.Positions[i];
                        glm::vec3 p1 = tmp.Positions[index_1];
                        glm::vec3 p2 = tmp.Positions[index_2];
                        auto      compute_len = [](glm::vec3 _a, glm::vec3 _b) -> float {
                            glm::vec3 _c = _a - _b;
                            return sqrt(_c[0] * _c[0] + _c[1] * _c[1] + _c[2] * _c[2]);
                        };
                        float len01 = compute_len(p0, p1);
                        float len12 = compute_len(p1, p2);
                        float len20 = compute_len(p2, p0);

                        float cos_alpha_01 = (len20 * len20 + len12 * len12 - len01 * len01) / (2 * len20 * len12);
                        float cos_alpha_02 = (len01 * len01 + len12 * len12 - len20 * len20) / (2 * len01 * len12);
                        // Avoid 3 vertex on a line
                        if (cos_alpha_01 >= 0.99 || cos_alpha_01 <= -0.99) continue;
                        if (cos_alpha_02 >= 0.99 || cos_alpha_02 <= -0.99) continue;

                        float weight_01 = cos_alpha_01 / sqrt(1 - cos_alpha_01 * cos_alpha_01);
                        float weight_02 = cos_alpha_02 / sqrt(1 - cos_alpha_02 * cos_alpha_02);
                        sum_weight += weight_01 + weight_02;
                        v_star += weight_01 * p1;
                        v_star += weight_02 * p2;
                    }
                }
                v_star /= sum_weight;
                glm::vec3 new_v = tmp.Positions[i] * (1 - lambda) + v_star * lambda;
                output.Positions.push_back(new_v);
            }

            tmp = output;
        }
        output = tmp;
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here
        
        auto unit = [](std::int32_t i) -> glm::vec3 {
            if (i == 0) return glm::vec3(1.0f, 0.0f, 0.0f);
            if (i == 1) return glm::vec3(0.0f, 1.0f, 0.0f);
            if (i == 2) return glm::vec3(0.0f, 0.0f, 1.0f);
            return glm::vec3(0.0f, 0.0f, 0.0f);
        };
        auto edge_dir = [&unit](std::int32_t i) -> glm::vec3 {
            return unit(i >> 2);
        };
        auto edge_from = [&dx, &unit](glm::vec3 & v0, std::int32_t i) -> glm::vec3 {
            return v0 + dx * (i & 1) * unit(((i >> 2) + 1) % 3) + dx * ((i >> 1) & 1) * unit(((i >> 2) + 2) % 3);
        };

        for (std::int32_t _i = 0; _i < n; ++_i) {
            for (std::int32_t _j = 0; _j < n; ++_j) {
                for (std::int32_t _k = 0; _k < n; ++_k) {
                    glm::vec3 v0 = grid_min + glm::vec3(_i, _j, _k) * dx;

                    // Step 1. Calculating vertex state and intersected edges
                    std::int32_t state_vertex = 0;
                    for (std::int32_t i = 0; i < 8; ++i) {
                        glm::vec3 v_i = v0 + glm::vec3((i & 1), (i >> 1 & 1), (i >> 2 & 1)) * dx;
                        std::int32_t state_vertex_i = sdf(v_i) >= 0 ? 1 : 0;
                        state_vertex ^= (state_vertex_i << i);
                    }
                    glm::int32_t state_edge = c_EdgeStateTable.at(state_vertex);

                    // Step 2. Calculating intersection point
                    std::vector<glm::vec3> intersection_points(12);
                    for (std::int32_t i = 0; i < 12; ++i) {
                        if ((state_edge >> i) & 1) {
                            // Find 2 vertex of the edge
                            glm::vec3 v_from = edge_from(v0, i);
                            glm::vec3 v_to   = v_from + edge_dir(i) * dx;
                            // Compute intersection point
                            float lambda = 0.5;
                            float sdf_from = sdf(v_from);
                            float sdf_to   = sdf(v_to);
                            if (sdf_from != sdf_to) {
                                lambda = -sdf_to / (sdf_from - sdf_to);
                            }
                            intersection_points[i] = lambda * v_from + (1 - lambda) * v_to;
                        }
                    }
                
                    // Step 3. Insert new vertex and 
                    std::vector<std::int32_t> edge_to_vertex_idx(12);
                    for (std::int32_t i = 0; i < 12; ++i) {
                        if ((state_edge >> i) & 1) {
                            edge_to_vertex_idx[i] = output.Positions.size();
                            output.Positions.push_back(intersection_points[i]);
                        }
                    }

                    // Step 4. Connect new faces
                    std::array<int, 16> new_faces = c_EdgeOrdsTable.at(state_vertex);
                    for (std::int32_t i = 0; i < 16; i += 3) {
                        if (new_faces[i] < 0) break;
                        output.Indices.push_back(edge_to_vertex_idx[new_faces[i]]);
                        output.Indices.push_back(edge_to_vertex_idx[new_faces[i + 2]]);
                        output.Indices.push_back(edge_to_vertex_idx[new_faces[i + 1]]);
                    }
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
