#include "Engine/app.h"
#include "FastSimulation/CaseMassSpring.h"
#include "FastSimulation/FastSimulation.h"
#include "Common/ImGuiHelper.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace VCX::Labs::Animation {
    CaseMassSpring::CaseMassSpring() :
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points),
        _springsItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void CaseMassSpring::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("Part. Mass", &_massSpringSystem.Mass, .5f, 10.f);
            ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 10.f, 300.f);
            ImGui::SliderFloat("Damp.", &_massSpringSystem.Damping, 0.f, 1.f);
            ImGui::SliderFloat("Gravity", &_massSpringSystem.Gravity, .1f, 1.f);
            ImGui::SliderInt("Steps", &_massSpringSystem.Step, 1, 20);

            if (ImGui::CollapsingHeader("Wind")) {
                if (ImGui::Button(_massSpringSystem.HasWind ? "Stop Wind" : "Start Wind")) _massSpringSystem.HasWind = ! _massSpringSystem.HasWind;
                ImGui::SliderFloat("Strength", &_massSpringSystem.WindStrength, 0.f, 1.f);
                ImGui::SliderFloat("Dir. X", &_massSpringSystem.WindDirX, -1.f, 1.f);
                ImGui::SliderFloat("Dir. Y", &_massSpringSystem.WindDirY, -1.f, 1.f);
                ImGui::SliderFloat("Dir. Z", &_massSpringSystem.WindDirZ, -1.f, 1.f);
            }
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Part. Size", &_particleSize, 1, 6);
            ImGui::ColorEdit3("Part. Color", glm::value_ptr(_particleColor));
            ImGui::SliderFloat("Spr. Width", &_springWidth, .001f, 1.f);
            ImGui::ColorEdit3("Spr. Color", glm::value_ptr(_springColor));
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseMassSpring::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) AdvanceMassSpringSystem(_massSpringSystem, Engine::GetDeltaTime());
        
        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
        _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View"      , _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glPointSize(_particleSize);
        glLineWidth(_springWidth);

        _program.GetUniforms().SetByName("u_Color", _springColor);
        _springsItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", _particleColor);
        _particlesItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseMassSpring::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseMassSpring::ResetSystem() {
        _massSpringSystem = { };
        std::size_t const n = 10;
        float const delta = 2.f / n;
        auto constexpr GetID = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };
        for (std::size_t i = 0; i <= n; i++) {
            for (std::size_t j = 0; j <= n; j++) {
                _massSpringSystem.AddParticle(glm::vec3(i * delta , 1.5f, j * delta - 1.f));
                if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                if (i > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 2, j));
                if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                if (j > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 2));
                if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));
            }
        }
        _massSpringSystem.Fixed[GetID(0, 0)] = true;
        _massSpringSystem.Fixed[GetID(0, n)] = true;
        // _massSpringSystem.Fixed[GetID(n, 0)] = true;
        // _massSpringSystem.Fixed[GetID(n, n)] = true;
        std::vector<std::uint32_t> indices;
        for (auto const & spring : _massSpringSystem.Springs) {
            indices.push_back(std::uint32_t(spring.AdjIdx.first));
            indices.push_back(std::uint32_t(spring.AdjIdx.second));
        }
        _springsItem.UpdateElementBuffer(indices);

        // Pre-calculated matrices M, L, J
        int nn = _massSpringSystem.Positions.size();
        int cn = _massSpringSystem.Springs.size();

        _massSpringSystem._M = Eigen::SparseMatrix<float>(3 * nn, 3 * nn);
        std::vector<Eigen::Triplet<float>> MTriplets;
        for (int i = 0; i < nn; ++i) {
            for (int j = 0; j < 3; ++j) {
                MTriplets.push_back(Eigen::Triplet<float>(3 * i + j, 3 * i + j, _massSpringSystem.Mass));
            }
        }
        _massSpringSystem._M.setFromTriplets(MTriplets.begin(), MTriplets.end());

        _massSpringSystem._L = Eigen::SparseMatrix<float>(3 * nn, 3 * nn);
        std::vector<Eigen::Triplet<float>> LTriplets;
        for (int i = 0; i < cn; ++i) {
            auto       spring = _massSpringSystem.Springs[i];
            auto const si     = spring.AdjIdx.first;
            auto const ei     = spring.AdjIdx.second;
            float      k      = _massSpringSystem.Stiffness;

            for (int j = 0; j < 3; ++j) {
                LTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * si + j, k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * ei + j, -k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * ei + j, k));
                LTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * si + j, -k));
            }
        }
        _massSpringSystem._L.setFromTriplets(LTriplets.begin(), LTriplets.end());

        _massSpringSystem._J = Eigen::SparseMatrix<float>(3 * nn, 3 * cn);
        std::vector<Eigen::Triplet<float>> JTriplets;
        for (int i = 0; i < cn; ++i) {
            auto       spring = _massSpringSystem.Springs[i];
            auto const si     = spring.AdjIdx.first;
            auto const ei     = spring.AdjIdx.second;
            float      k      = _massSpringSystem.Stiffness;

            for (int j = 0; j < 3; ++j) {
                JTriplets.push_back(Eigen::Triplet<float>(3 * si + j, 3 * i + j, -k));
                JTriplets.push_back(Eigen::Triplet<float>(3 * ei + j, 3 * i + j, k));
            }
        }
        _massSpringSystem._J.setFromTriplets(JTriplets.begin(), JTriplets.end());
    }
}
