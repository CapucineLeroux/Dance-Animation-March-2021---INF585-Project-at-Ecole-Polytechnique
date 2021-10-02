#pragma once

#include "vcl/vcl.hpp"


namespace vcl
{
	// Helper structure storing an animated skeleton
	struct skeleton_animation_structure
	{
		buffer<int> parent_index;          // Connectivity of the skeleton. Stores at index i, the index of the parent joint
		buffer<affine_rt> rest_pose_local; // Rest pose storage of the rigid transforms (rotation and translation) of the joints in local coordinates

		// Number of joints in the skeleton
                size_t number_joint() const;

		// Return the rigid transforms of the joints of the rest pose in global coordinates
		buffer<affine_rt> rest_pose_global() const;

		// Apply scaling to the entire skeleton (scale the translation part of the rigid transforms)
		void scale(float s);

	};

	// Convert a skeleton defined in local coordinates to global coordinates
	buffer<affine_rt> skeleton_local_to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index);

        // Gives all the 3D positions of the joints in a 1D buffer
        buffer<vcl::vec3> get_joint_position(skeleton_animation_structure skeleton_data);
}
