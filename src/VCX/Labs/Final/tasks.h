#pragma once

#include "Labs/Common/ImageRGB.h"
#include "Labs/Final/Data.h"

namespace VCX::Labs::Final {
    
    void FinalTask(Common::ImageRGB& canvas, const double r, const size_t sampleRate, const size_t iterTimes, const std::string& file);
    void NDimension(const double r, const size_t n, const size_t l);
    
};