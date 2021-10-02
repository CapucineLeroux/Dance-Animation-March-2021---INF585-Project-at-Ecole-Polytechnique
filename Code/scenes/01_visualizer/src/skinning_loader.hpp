#pragma once

#include "vcl/vcl.hpp"
#include "skinning.hpp"
#include "skeleton.hpp"


void load_skinning_data(std::string const& directory, vcl::skeleton_animation_structure& skeleton_data, vcl::rig_structure& rig, vcl::mesh& shape);
void load_skinning_anim(std::string const& directory, vcl::skeleton_animation_structure& skeleton_data);
