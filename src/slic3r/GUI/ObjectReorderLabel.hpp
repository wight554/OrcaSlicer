#pragma once

#include <cstddef>
#include <string>

namespace Slic3r::GUI {

struct ObjectReorderLabel {
    size_t      instance_id{ 0 };
    std::string text;
};

} // namespace Slic3r::GUI
