#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "FastSimulation/FastSimulation.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    
    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        int const steps = system.Step;
        float const ddt = dt / steps; 

        int nn = system.Positions.size();
        int cn = system.Springs.size();

        // External force, gravity and wind here
        Eigen::VectorXf f_ext = Eigen::VectorXf::Zero(3 * nn);
        for (int i = 0; i < nn; ++i) {
            f_ext[i * 3 + 1] = - system.Mass * system.Gravity;

            if (system.HasWind) {
                f_ext[i * 3] += system.WindStrength * system.WindDirX;
                f_ext[i * 3 + 1] += system.WindStrength * system.WindDirY;
                f_ext[i * 3 + 2] += system.WindStrength * system.WindDirZ;
            }
        }

        Eigen::SparseMatrix<float> Q      = system._M + ddt * ddt * system._L;
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(Q);

        for (std::size_t s = 0; s < steps; s++) {
            Eigen::VectorXf y = Eigen::VectorXf::Zero(3 * nn);
            for (int i = 0; i < nn; ++i) {
                for (int j = 0; j < 3; ++j) {
                    y[3 * i + j] = system.Positions[i][j] + ddt * (1 - system.Damping / 1000.f) * system.Velocities[i][j];
                }
            }

            // Local Solver
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

            // Global Solver
            Eigen::VectorXf b = ddt * ddt * (system._J * d + f_ext) + system._M * y;
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
