#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/ICase.h"
#include "Labs/Final/Data.h"

namespace VCX::Labs::Final {

	class CaseNDimension : public Common::ICase {
    public:
        CaseNDimension();

        virtual std::string_view const GetName() override { return "Blue Noise N Dimension"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D      _texture;
        Common::ImageRGB                 _empty;

        bool   _enableZoom = true;
        bool   _recompute  = true;
        float  _radius     = 2;
        int    _n          = 3;
        int    _size       = 10;
    };

}