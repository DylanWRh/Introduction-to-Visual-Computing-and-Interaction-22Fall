#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + glm::rotate(ik.JointGlobalRotation[i-1], ik.JointLocalOffset[i]);
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat

        // int Iter_cnt = 0;

        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int nJoints = ik.NumJoints();
            for (int i = nJoints - 2; i >= 0; i--) {
                glm::vec3 orig = glm::normalize(ik.JointGlobalPosition[nJoints - 1] - ik.JointGlobalPosition[i]);
                glm::vec3 dest = glm::normalize(EndPosition - ik.JointGlobalPosition[i]);
                glm::quat rot  = glm::rotation(orig, dest);
                ik.JointLocalRotation[i] *= rot;
                ForwardKinematics(ik, i);
            }

            // Iter_cnt = CCDIKIteration;
        }

        // printf("%d\n", Iter_cnt);
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {

        // int Iter_cnt = 0;

        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 dir = glm::normalize(ik.JointGlobalPosition[i] - backward_positions[i + 1]);
                backward_positions[i] = backward_positions[i + 1] + dir * glm::length(ik.JointLocalOffset[i + 1]);
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 dir = glm::normalize(backward_positions[i + 1] - forward_positions[i]);
                forward_positions[i + 1] = forward_positions[i] + dir * glm::length(ik.JointLocalOffset[i + 1]);
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions

            // Iter_cnt = IKIteration;
        }

        // printf("%d\n", Iter_cnt);

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        /*int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;

        // sub-task 4.1
        // float threshold0 = 0.2f;
        // float threshold1 = 0.5f, threshold2 = 2.0f;

        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
            
            // sub-task 4.1
            // if (index && glm::l2norm((*custom)[index] - (*custom)[index - 1]) <= threshold0) {
            //     index--;
            // }
            // if (index && glm::l2norm((*custom)[index] - (*custom)[index - 1]) >= threshold1 && glm::l2norm((*custom)[index] - (*custom)[index - 1]) <= threshold2) {
            //     int new_p = glm::l2norm((*custom)[index] - (*custom)[index - 1]) / threshold1 + 1;
            //     (*custom)[index + new_p] = (*custom)[index];
            //     for (int j = index; j < index + new_p; j++) {
            //         float new_r  = glm::pi<float>() * (i - 1) / nums + glm::pi<float>() * (j - index) / (new_p * nums);
            //         float x_val = 1.5e-3f * custom_x(92 * new_r);
            //         float y_val = 1.5e-3f * custom_y(92 * new_r);
            //         (*custom)[j] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
            //     }
            //     index = index + new_p;
            // }
        }
        custom->resize(index);
        return custom;*/

        // Sub-Task 4, draw VCX
        int nums = 600;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int                      index = 0;
        for (int i = 0; i < nums / 3; i++) {
            float x_val = -0.75f + 1.5f * i / nums;
            float y_val = 0.5f - abs(1.0f * (6 * i - nums) / nums);
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }
        float PI = acos(-1.0);
        float THETA = PI / 3;
        float DELTA = (2 * PI - 2 * THETA) * 3 / nums;
        for (int i = 0; i < nums / 3; i++) {
            float ALPHA = THETA + DELTA * i;
            float x_val = 0.25f * cos(ALPHA);
            float y_val = - 0.5f * sin(ALPHA);
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }
        for (int i = 0; i < nums / 6; i++) {
            float x_val = 0.75f - 3.0f * i / nums;
            float y_val = -0.5f + 6.0f * i / nums;
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }
        for (int i = 0; i < nums / 6; i++) {
            float x_val = 0.25f + 3.0f * i / nums;
            float y_val = -0.5f + 6.0f * i / nums;
            (*custom)[index++] = glm::vec3(x_val, 0.0f, y_val);
        }
        custom->resize(index);
        return custom;
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        int const steps = 3;
        float const ddt = dt / steps; 

        int nn = system.Positions.size();
        int cn = system.Springs.size();

        Eigen::VectorXf f_ext = Eigen::VectorXf::Zero(3 * nn);
        for (int i = 0; i < nn; ++i) {
            f_ext[i * 3 + 1] = - system.Mass * system.Gravity;
        }

        Eigen::SparseMatrix<float> M(3 * nn, 3 * nn);
        std::vector<Eigen::Triplet<float>> MTriplets;
        for (int i = 0; i < nn; ++i) {
            for (int j = 0; j < 3; ++j) {
                MTriplets.push_back(Eigen::Triplet<float>(3 * i + j, 3 * i + j, system.Mass));
            }
        }
        M.setFromTriplets(MTriplets.begin(), MTriplets.end());
       
        Eigen::SparseMatrix<float> L(3 * nn, 3 * nn);
        std::vector<Eigen::Triplet<float>> LTriplets;
        for (int i = 0; i < cn; ++i) {
            auto spring = system.Springs[i];
            auto const si     = spring.AdjIdx.first;
            auto const ei     = spring.AdjIdx.second;
            float      k      = system.Stiffness;

            for (int j = 0; j < 3; ++j) {
                LTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * si + j, k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * ei + j, -k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * ei + j, k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * si + j, -k));
            }
        }
        L.setFromTriplets(LTriplets.begin(), LTriplets.end());

        Eigen::SparseMatrix<float> J(3 * nn, 3 * cn);
        std::vector<Eigen::Triplet<float>> JTriplets;
        for (int i = 0; i < cn; ++i) {
            auto       spring = system.Springs[i];
            auto const si     = spring.AdjIdx.first;
            auto const ei     = spring.AdjIdx.second;
            float      k      = system.Stiffness;

            for (int j = 0; j < 3; ++j) {
                JTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * i + j, -k));
                JTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * i + j, k));
            }
        }
        J.setFromTriplets(JTriplets.begin(), JTriplets.end());

        Eigen::SparseMatrix<float> Q = M + ddt * ddt * L;

        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(Q);

        for (std::size_t s = 0; s < steps; s++) {
            /*std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }*/
            Eigen::VectorXf y = Eigen::VectorXf::Zero(3 * nn);
            for (int i = 0; i < nn; ++i) {
                for (int j = 0; j < 3; ++j) {
                    y[3 * i + j] = system.Positions[i][j] + ddt * system.Velocities[i][j];
                }
            }

            Eigen::VectorXf d = Eigen::VectorXf::Zero(3 * cn);
            for (int i = 0; i < cn; ++i) {
                auto       spring = system.Springs[i];
                auto const si     = spring.AdjIdx.first;
                auto const ei     = spring.AdjIdx.second;
                float      r      = spring.RestLength;

                glm::vec3 di = glm::normalize(system.Positions[ei] - system.Positions[si]) * r;
                for (int j = 0; j < 3; ++ j) {
                    d[3 * i + j] = di[j];
                }
            }

            Eigen::VectorXf b = ddt * ddt * (J * d + f_ext) + M * y;
            Eigen::VectorXf x = solver.solve(b);

            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                for (int j = 0; j < 3; ++j) {
                    system.Velocities[i][j] = (x[3 * i + j] - system.Positions[i][j]) / ddt;
                    system.Positions[i][j] = x[3 * i + j];
                }
            }
        }
    }
}
