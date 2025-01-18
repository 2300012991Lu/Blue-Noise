#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Final/CaseFinal.h"
#include "Labs/Final/CaseNDimension.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Final {
    class App : public Engine::IApp {
    private:
        Common::UI      _ui;
        std::size_t     _caseId = 0;
        CaseFinal       _caseFinal;
        CaseNDimension  _caseNDimension;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseFinal,
            _caseNDimension
        };

    public:
        App();

        void OnFrame() override;
    };
}
