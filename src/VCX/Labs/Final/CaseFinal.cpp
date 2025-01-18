#include <fstream>

#include "Engine/app.h"

#include "Labs/Final/CaseFinal.h"
#include "Labs/Final/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Final {
    static constexpr auto c_Size = std::pair(960U, 960U);

    CaseFinal::CaseFinal():
        _texture(
            Engine::GL::SamplerOptions {
                .MinFilter = Engine::GL::FilterMode::Linear,
                .MagFilter = Engine::GL::FilterMode::Nearest }),
        _empty(c_Size.first, c_Size.second),
        _recompute(true) {

        _empty.Fill(glm::vec3(1));
    }

    void CaseFinal::OnSetupPropsUI() {
        Common::ImGuiHelper::SaveImage(_texture, {c_Size.first, c_Size.second}, false);
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        float oldRadius = _radius;
        ImGui::SliderFloat("Radius", &_radius, 0.75, 2);
        ImGui::Spacing();
        int oldSampleRate = _sampleRate;
        ImGui::SliderInt("Sample Rate", &_sampleRate, 1, 10);
        ImGui::Spacing();
        int oldIterTimes = _iterTimes;
        ImGui::SliderInt("Iter Times", &_iterTimes, 1, 40);
        ImGui::Spacing();
        if (_radius != oldRadius || _sampleRate != oldSampleRate || _iterTimes != oldIterTimes) _recompute = true;
    }

    Common::CaseRenderResult CaseFinal::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        Common::ImageRGB result(c_Size.first, c_Size.second);
        auto files = VCX::Engine::GetDropFiles();
        std::string file = "";
        if (files.size() != 0) {
            _sampleRate = 1;
            file = files[0];
            _recompute = true;
        }
        if (_recompute) {
            _recompute = false;
            FinalTask(result, (double)_radius, (size_t)_sampleRate, (size_t)_iterTimes, file);
            _texture.Update(result);
        }
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseFinal::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
            ImGui::SetScrollX(window, window->Scroll.x - delta.x);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
            ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_texture, c_Size, pos);
    }
}