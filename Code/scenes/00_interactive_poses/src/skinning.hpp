#pragma once

#include "vcl/vcl.hpp"


namespace vcl
{
	struct rig_structure
	{
                buffer<buffer<int>> joint; //for each vertex, gives the joints it is linked to
                buffer<buffer<float>> weight; //for each vertex, gives the corresponding joint weights
	};

	void normalize_weights(buffer<buffer<float>>& weights);

	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,
		buffer<vec3>& normal_skinned,
		buffer<affine_rt> const& skeleton_current,
		buffer<affine_rt> const& skeleton_rest_pose,
		buffer<vec3> const& position_rest_pose, 
		buffer<vec3> const& normal_rest_pose, 
		rig_structure const& rig);


}
