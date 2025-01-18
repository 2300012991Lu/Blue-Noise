#include "Assets/bundled.h"
#include "Labs/Final/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::Final::App>(Engine::AppContextOptions {
        .Title      = "VCX Final Project",
        .WindowSize = { 1024, 768 },
        .FontSize   = 16,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
