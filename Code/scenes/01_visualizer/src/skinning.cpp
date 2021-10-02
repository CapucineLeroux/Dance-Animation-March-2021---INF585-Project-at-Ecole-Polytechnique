#include "skinning.hpp"

namespace vcl
{
	void normalize_weights(buffer<buffer<float>>& weights)
	{
		size_t const N = weights.size();
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			for(float w : weights[k]) s += w;
			assert_vcl_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}


	// Linear Blend Skinning
	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,  // position to deform
		buffer<vec3>& normal_skinned,    // normal to deform
		buffer<affine_rt> const& skeleton_current,    // rigid transforms for skeleton joints in current pose
		buffer<affine_rt> const& skeleton_rest_pose,  // rigid transforms of skeleton joints in rest pose
		buffer<vec3> const& position_rest_pose,       // vertex positions of the mesh in rest pose
		buffer<vec3> const& normal_rest_pose,         // normal coordinates of the mesh in rest pose
		rig_structure const& rig)                     // information of the skinning weights (joints and weights associated to a vertex)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		// Sanity check on sizes of buffers
		assert_vcl_no_msg(position_skinned.size()==N_vertex);
		assert_vcl_no_msg(normal_skinned.size()==N_vertex);
		assert_vcl_no_msg(normal_rest_pose.size()==N_vertex);
		assert_vcl_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_vcl_no_msg(rig.joint.size()==N_vertex);
                assert_vcl_no_msg(rig.weight.size()==N_vertex);
		// To do
		//   Compute the Linear Blend Skinning ...
                vec3 p;
                vec3 p0;
                vec3 n;
                vec3 n0;
                size_t nb_of_joints;

                for (size_t i=0 ; i<N_vertex ; i++){

                    p = {0,0,0};
                    p0 = position_rest_pose[i];

                    n = {0,0,0};
                    n0 = normal_rest_pose[i];

                    nb_of_joints = rig.joint[i].size();
                    affine_rt Tk;
                    affine_rt Tk0;
                    affine_rt Mk;
                    float alphak;
                    int joint_index;

                    for (size_t k=0 ; k<nb_of_joints ; k++){

                        joint_index = rig.joint[i][k];

                        Tk = skeleton_current[joint_index];
                        Tk0 = skeleton_rest_pose[joint_index];
                        Mk = Tk*inverse(Tk0);
                        alphak = rig.weight[i][k];

                        p = p + alphak*(Mk*p0);
                        n = n + alphak*(Mk*n0);
                    }

                    position_skinned[i] = p;
                    normal_skinned[i] = n;
                }


	}




}
