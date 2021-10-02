#include "skeleton.hpp"

using namespace std;

namespace vcl
{

	size_t skeleton_animation_structure::number_joint() const
	{
		return parent_index.size();
	}


	void skeleton_animation_structure::scale(float s)
	{
		size_t const N_joint = number_joint();

		for(size_t k=0; k<N_joint; ++k)
			rest_pose_local[k].translate *= s;
	}
	

	buffer<affine_rt> skeleton_animation_structure::rest_pose_global() const
	{
		return skeleton_local_to_global(rest_pose_local, parent_index);
	}

	buffer<affine_rt> skeleton_local_to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index)
	{
		assert_vcl(parent_index.size()==local.size(), "Incoherent size of skeleton data");
		size_t const N = parent_index.size();
		buffer<affine_rt> global;
		global.resize(N);
		global[0] = local[0];

		for (size_t k = 1; k < N; ++k)
			global[k] = global[parent_index[k]] * local[k];
	
		return global;
	}

        buffer<vec3> get_joint_position(skeleton_animation_structure skeleton_data){

            size_t N = skeleton_data.rest_pose_local.size();
            buffer<vec3> joint_positions(N);


            for (size_t i=0 ; i<N ; i++){
                joint_positions[i] = skeleton_data.rest_pose_global()[i].translate;
            }

            return joint_positions;

        }

}
