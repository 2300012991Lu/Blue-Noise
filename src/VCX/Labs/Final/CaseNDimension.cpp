#include "Labs/Final/CaseNDimension.h"

#include "Engine/app.h"

#include "Labs/Final/CaseNDimension.h"
#include "Labs/Final/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Final {
    static constexpr auto c_Size = std::pair(0U, 0U);

    CaseNDimension::CaseNDimension():
        _texture(
            Engine::GL::SamplerOptions {
                .MinFilter = Engine::GL::FilterMode::Linear,
                .MagFilter = Engine::GL::FilterMode::Nearest }),
        _empty(c_Size.first, c_Size.second),
        _recompute(true) {

        _empty.Fill(glm::vec3(1));
    }

    void CaseNDimension::OnSetupPropsUI() {
        Common::ImGuiHelper::SaveImage(_texture, {c_Size.first, c_Size.second}, false);
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        float oldRadius = _radius;
        ImGui::SliderFloat("Radius", &_radius, 0.75, 3);
        ImGui::Spacing();
        int oldN = _n;
        ImGui::SliderInt("Dimension", &_n, 2, 5);
        ImGui::Spacing();
        int oldSize = _size;
        ImGui::SliderInt("Dim Length", &_size, 5, 25);
        ImGui::Spacing();
        if (_radius != oldRadius || _n != oldN || _size != oldSize) _recompute = true;
    }

    Common::CaseRenderResult CaseNDimension::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            NDimension((double)_radius, (size_t)_n, (size_t)_size);
        }
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseNDimension::OnProcessInput(ImVec2 const & pos) {
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