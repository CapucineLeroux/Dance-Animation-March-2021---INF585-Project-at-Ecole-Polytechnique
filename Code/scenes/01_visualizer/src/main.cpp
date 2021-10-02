/**
	Objectives:
	- Complete rigid transform interpolation in the function evaluate_local function in file skeleton.cpp
	- Complete the Linear Blend Skinning computation in the function skinning.cpp
*/

#include "vcl/vcl.hpp"
#include <iostream>

#include "skeleton.hpp"
#include "skeleton_drawable.hpp"
#include "skinning.hpp"
#include "skinning_loader.hpp"
#include "cloth_simulation.hpp"


using namespace vcl;
using namespace std;

struct gui_parameters {

	bool display_frame = true;
        bool surface_skinned = true;
	bool wireframe_skinned = false;
	bool surface_rest_pose = false;
	bool wireframe_rest_pose = false;

	bool skeleton_current_bone = true;
	bool skeleton_current_frame = false;
	bool skeleton_current_sphere = false;

	bool skeleton_rest_pose_bone = false;
	bool skeleton_rest_pose_frame = false;
	bool skeleton_rest_pose_sphere = false;

        //Controls if the body is moving
        bool action = true;
};

struct user_interaction_parameters {
	vec2 mouse_prev;
	timer_fps fps_record;
	gui_parameters gui;
	mesh_drawable global_frame;
	bool cursor_on_gui;
};
user_interaction_parameters user;

struct scene_environment
{
	camera_around_center camera;
	mat4 projection;
	vec3 light;
};
scene_environment scene;

struct visual_shapes_parameters
{
        mesh surface_skinned_mesh;
	mesh_drawable surface_skinned;
	mesh_drawable surface_rest_pose;
	skeleton_drawable skeleton_current;
	skeleton_drawable skeleton_rest_pose;
};
visual_shapes_parameters visual_data;

struct skinning_current_data
{
	buffer<vec3> position_rest_pose;
	buffer<vec3> position_skinned;
	buffer<vec3> normal_rest_pose;
	buffer<vec3> normal_skinned;

        buffer<affine_rt> skeleton_current;
	buffer<affine_rt> skeleton_rest_pose;
};

skeleton_animation_structure skeleton_data;
rig_structure rig;
skinning_current_data skinning_data;
obstacle_parameters body_obstacles;

struct cloth_structure
{
    int N_cloth;
    float side_length_x;
    float side_length_y;
    grid_2D<vec3> position;
    grid_2D<vec3> velocity;
    grid_2D<vec3> forces;

    grid_2D<vec3> normal;

    buffer<uint3> triangle_connectivity;
    mesh_drawable visual;

    simulation_parameters parameters;
};

cloth_structure cloth1;
GLuint texture_cloth1;
cloth_structure cloth2;
GLuint texture_cloth2;

mesh_drawable ground;

timer_interval timer;

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);

void initialize_data();
void display_scene();
void display_interface();
void compute_deformation();
void update_new_content(mesh& shape, GLuint texture_id);
void initialize_cloth();
void update_obstacles();
void change_cloth_orientation(); //just to handle the change of coordinates system (cloth is not originally oriented the same way as the body)
void change_back_cloth_orientation();





int main(int, char* argv[])
{

	std::cout << "Run " << argv[0] << std::endl;

	int const width = 1280, height = 1024;
	GLFWwindow* window = create_window(width, height);
	window_size_callback(window, width, height);
	std::cout << opengl_info_display() << std::endl;;

	imgui_init(window);
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);

	std::cout<<"Initialize data ..."<<std::endl;
	initialize_data();
        timer.scale = 1.f;

	std::cout<<"Start animation loop ..."<<std::endl;
	user.fps_record.start();
        timer.start();
	glEnable(GL_DEPTH_TEST);
	while (!glfwWindowShouldClose(window))
	{
		scene.light = scene.camera.position();
		user.fps_record.update();
                timer.update();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);
		imgui_create_frame();
		if(user.fps_record.event) {
			std::string const title = "VCL Display - "+str(user.fps_record.fps)+" fps";
			glfwSetWindowTitle(window, title.c_str());
		}

		ImGui::Begin("GUI",NULL,ImGuiWindowFlags_AlwaysAutoResize);
		user.cursor_on_gui = ImGui::IsAnyWindowFocused();

		if(user.gui.display_frame)
			draw(user.global_frame, scene);
		

		display_interface();
                if (user.gui.action) {compute_deformation();}

                //cloth
                float const dt = 0.005f * timer.scale;
                float const m1 = cloth1.parameters.mass_total/cloth1.position.size();
                float const m2 = cloth2.parameters.mass_total/cloth2.position.size();
                size_t const N_substeps = 5;
                for(size_t k_substep=0; k_substep<N_substeps; ++k_substep){

                        compute_forces(cloth1.forces, cloth1.position, cloth1.velocity, cloth1.normal, cloth1.side_length_x, cloth1.side_length_y, cloth1.parameters);
                        numerical_integration(cloth1.position, cloth1.velocity, cloth1.forces, m1, dt);

                        compute_forces(cloth2.forces, cloth2.position, cloth2.velocity, cloth2.normal, cloth2.side_length_x, cloth2.side_length_y, cloth2.parameters);
                        numerical_integration(cloth2.position, cloth2.velocity, cloth2.forces, m2, dt);

                        change_cloth_orientation();
                        update_obstacles();
                        apply_constraints(cloth1.position, cloth1.velocity, cloth1.side_length_x, cloth1.side_length_y, cloth2.position, cloth2.velocity, cloth2.side_length_x, cloth2.side_length_y, body_obstacles, skinning_data.skeleton_current);
                        change_back_cloth_orientation();

                        bool simulation_diverged1 = detect_simulation_divergence(cloth1.forces, cloth1.position);
                        bool simulation_diverged2 = detect_simulation_divergence(cloth2.forces, cloth2.position);
                        if(simulation_diverged1==true)
                        {
                                std::cerr<<" **** Simulation 1 has diverged **** "<<std::endl;
                                std::cerr<<" > Stop simulation 1 iterations"<<std::endl;
                                break;
                        }
                        if(simulation_diverged2==true)
                        {
                                std::cerr<<" **** Simulation 2 has diverged **** "<<std::endl;
                                std::cerr<<" > Stop simulation 2 iterations"<<std::endl;
                                break;
                        }
                }

                change_cloth_orientation();
		display_scene();
                change_back_cloth_orientation();

		ImGui::End();
		imgui_render_frame(window);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void initialize_data()
{
	GLuint const shader_mesh = opengl_create_shader_program(opengl_shader_preset("mesh_vertex"), opengl_shader_preset("mesh_fragment"));
	GLuint const shader_uniform_color = opengl_create_shader_program(opengl_shader_preset("single_color_vertex"), opengl_shader_preset("single_color_fragment"));
        GLuint const texture_white = opengl_texture_to_gpu(image_raw{1,1,image_color_type::rgba,{255,255,255,255}});
        GLuint const texture_turquoise = opengl_texture_to_gpu(image_raw{1,1,image_color_type::rgba,{93,177,121,210}});
        GLuint const texture_pink = opengl_texture_to_gpu(image_raw{1,1,image_color_type::rgba,{245,205,231,250}});
	mesh_drawable::default_shader = shader_mesh;
        mesh_drawable::default_texture = texture_white;
	curve_drawable::default_shader = shader_uniform_color;
	segments_drawable::default_shader = shader_uniform_color;

	user.global_frame = mesh_drawable(mesh_primitive_frame());
        user.gui.display_frame = false;
	scene.camera.distance_to_center = 2.5f;

        //ground

        ground = mesh_drawable(mesh_primitive_quadrangle({-1.5f,0,-1.5f},{1.5f,0,-1.5f},{1.5f,0,1.5f},{-1.5f,0,1.5f}));
        ground.texture = opengl_texture_to_gpu(image_load_png("../../../assets/ground/lino.png"));

        //body

        mesh shape;
        GLuint texture_id = mesh_drawable::default_texture;
        load_skinning_data("../../../assets/xbot/", skeleton_data, rig, shape);
        load_skinning_anim("../../../assets/xbot/anim_dance/", skeleton_data);

        normalize_weights(rig.weight);
        float const scaling = 0.005f;
        for(auto& p: shape.position) p *= scaling;
        skeleton_data.scale(scaling);

        update_new_content(shape, texture_id);

        //cloth

        // Enable blending
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        texture_cloth1 = opengl_texture_to_gpu(image_load_png("../../../assets/cloth/cloth.png"));
        texture_cloth2 = texture_turquoise;
        cloth1.N_cloth = 30;
        cloth2.N_cloth = 15;
        initialize_cloth();
        initialize_simulation_parameters(cloth1.parameters,cloth2.parameters);

}


void initialize_cloth()
{
    //cloth 1

        size_t const N_cloth1 = cloth1.N_cloth;
        float r = 0.06f;
        cloth1.side_length_x = 2*pi*r;
        cloth1.side_length_y = 0.6f;

        mesh const cloth_mesh1 = mesh_primitive_grid({1,0,cloth1.side_length_y},{1,0,0},{1,cloth1.side_length_x,0},{1,cloth1.side_length_x,cloth1.side_length_y},N_cloth1, N_cloth1);
        buffer<vec3> precomputed_initial_positions1; read_from_file("../../../assets/cloth/initial_positions_skirt.txt",precomputed_initial_positions1);
        cloth1.position = grid_2D<vec3>::from_buffer(precomputed_initial_positions1, N_cloth1, N_cloth1);

        cloth1.velocity.clear();
        cloth1.velocity.resize(N_cloth1*N_cloth1);

        cloth1.forces.clear();
        cloth1.forces.resize(N_cloth1,N_cloth1);

        cloth1.visual.clear();
        cloth1.visual = mesh_drawable(cloth_mesh1);
        cloth1.visual.texture = texture_cloth1;
        cloth1.visual.shading.phong = {0.3f, 0.7f, 0.05f, 32};

        cloth1.triangle_connectivity = cloth_mesh1.connectivity;
        cloth1.normal = grid_2D<vec3>::from_buffer(cloth_mesh1.normal, N_cloth1, N_cloth1);


    //cloth 2

        size_t const N_cloth2 = cloth2.N_cloth;
        r = 0.03f;
        cloth2.side_length_x = 2*pi*r;
        cloth2.side_length_y = 0.1f;

        mesh const cloth_mesh2 = mesh_primitive_grid({1,0,cloth2.side_length_y},{1,0,0},{1,cloth2.side_length_x,0},{1,cloth2.side_length_x,cloth2.side_length_y},N_cloth2, N_cloth2);

        buffer<vec3> precomputed_initial_positions2; read_from_file("../../../assets/cloth/initial_positions_15by15.txt",precomputed_initial_positions2);
        cloth2.position = grid_2D<vec3>::from_buffer(precomputed_initial_positions2, N_cloth2, N_cloth2);

        cloth2.velocity.clear();
        cloth2.velocity.resize(N_cloth2*N_cloth2);

        cloth2.forces.clear();
        cloth2.forces.resize(N_cloth2,N_cloth2);

        cloth2.visual.clear();
        cloth2.visual = mesh_drawable(cloth_mesh2);
        cloth2.visual.texture = texture_cloth2;
        cloth2.visual.shading.phong = {0.3f, 0.7f, 0.05f, 32};

        cloth2.triangle_connectivity = cloth_mesh2.connectivity;
        cloth2.normal = grid_2D<vec3>::from_buffer(cloth_mesh2.normal, N_cloth2, N_cloth2);

}


//obstacles are simple shapes (rounded cylinders and spheres) to simplify the body shape
//this is used for the cloth to handle collisions with the body
void update_obstacles(){

    vec3 p,p1,p2;
    float r;

    body_obstacles.sphere_centers.clear();
    body_obstacles.sphere_radiuses.clear();
    body_obstacles.cylinder_centers.clear();
    body_obstacles.cylinder_radiuses.clear();

    //head
    p = 0.5f*skinning_data.skeleton_current[15].translate + 0.5f*skinning_data.skeleton_current[20].translate;
    r = 0.06f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    //neck
    p = 0.5f*skinning_data.skeleton_current[10].translate + 0.5f*skinning_data.skeleton_current[15].translate;
    r = 0.03f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    //arms
    for(const auto& i : {16,17,23,24,25,26}) {
        p1 = skinning_data.skeleton_current[i].translate;
        p2 = skinning_data.skeleton_current[skeleton_data.parent_index[i]].translate;
        if (i==25 || i==26){r = 0.02f;}
        else {r = 0.03f;}
        body_obstacles.cylinder_centers.push_back(p1);
        body_obstacles.cylinder_centers.push_back(p2);
        body_obstacles.cylinder_radiuses.push_back(r);
    }

    //torax
    p = 0.5f*skinning_data.skeleton_current[7].translate + 0.25f*skinning_data.skeleton_current[11].translate + 0.25f*skinning_data.skeleton_current[12].translate;
    p.y -= 0.01f;
    p.z += 0.01f;
    r = 0.08f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    p = 0.5f*skinning_data.skeleton_current[7].translate + 0.5f*skinning_data.skeleton_current[4].translate;
    p.y += 0.01f;
    p.z += 0.02f;
    r = 0.065f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    p = skinning_data.skeleton_current[4].translate;
    r = 0.055f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    p = skinning_data.skeleton_current[0].translate;
    r = 0.065f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    p.z -= 0.02f;
    p.y -= 0.01f;
    r = 0.07f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    //legs

    p = skinning_data.skeleton_current[2].translate;
    r = 0.06f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    p = skinning_data.skeleton_current[3].translate;
    r = 0.06f;
    body_obstacles.sphere_centers.push_back(p);
    body_obstacles.sphere_radiuses.push_back(r);

    for(const auto& i : {5,6,8,9,13,14,18,19}) {

        p1 = skinning_data.skeleton_current[i].translate;
        p2 = skinning_data.skeleton_current[skeleton_data.parent_index[i]].translate;

        if (i==5){p2.x += 0.02f; p = 0.5f*p1+0.5f*p2; p.z += 0.01f;}
        if (i==6){p2.x -= 0.02f; p = 0.5f*p1+0.5f*p2; p.z += 0.01f;}

        if (i==5 || i==6){
            r = 0.05f;
            body_obstacles.cylinder_centers.push_back(p);
            body_obstacles.cylinder_centers.push_back(p2);
            body_obstacles.cylinder_radiuses.push_back(r);
            r = 0.04f;
            body_obstacles.cylinder_centers.push_back(p1);
            body_obstacles.cylinder_centers.push_back(p);
            body_obstacles.cylinder_radiuses.push_back(r);
        }
        else if (i==8 || i==9){
            p = 0.5f*p1+0.5f*p2;
            r = 0.04f;
            body_obstacles.cylinder_centers.push_back(p);
            body_obstacles.cylinder_centers.push_back(p2);
            body_obstacles.cylinder_radiuses.push_back(r);
            r = 0.03f;
            body_obstacles.cylinder_centers.push_back(p1);
            body_obstacles.cylinder_centers.push_back(p);
            body_obstacles.cylinder_radiuses.push_back(r);
        }
        else {
            r = 0.03f;
            body_obstacles.cylinder_centers.push_back(p1);
            body_obstacles.cylinder_centers.push_back(p2);
            body_obstacles.cylinder_radiuses.push_back(r);
        }
    }


}


void change_cloth_orientation(){

    for (size_t i=0 ; i<cloth1.position.size() ; i++){
        float x = cloth1.position[i][0];
        float y = cloth1.position[i][1];
        float z = cloth1.position[i][2];

        cloth1.position[i][0] = y;
        cloth1.position[i][1] = z;
        cloth1.position[i][2] = x;

        float vx = cloth1.velocity[i][0];
        float vy = cloth1.velocity[i][1];
        float vz = cloth1.velocity[i][2];

        cloth1.velocity[i][0] = vy;
        cloth1.velocity[i][1] = vz;
        cloth1.velocity[i][2] = vx;
    }

    for (size_t i=0 ; i<cloth2.position.size() ; i++){
        float x = cloth2.position[i][0];
        float y = cloth2.position[i][1];
        float z = cloth2.position[i][2];

        cloth2.position[i][0] = y;
        cloth2.position[i][1] = z;
        cloth2.position[i][2] = x;

        float vx = cloth2.velocity[i][0];
        float vy = cloth2.velocity[i][1];
        float vz = cloth2.velocity[i][2];

        cloth2.velocity[i][0] = vy;
        cloth2.velocity[i][1] = vz;
        cloth2.velocity[i][2] = vx;
    }

}


void change_back_cloth_orientation(){

    for (size_t i=0 ; i<cloth1.position.size() ; i++){
        float y = cloth1.position[i][0];
        float z = cloth1.position[i][1];
        float x = cloth1.position[i][2];

        cloth1.position[i][0] = x;
        cloth1.position[i][1] = y;
        cloth1.position[i][2] = z;

        float vy = cloth1.velocity[i][0];
        float vz = cloth1.velocity[i][1];
        float vx = cloth1.velocity[i][2];

        cloth1.velocity[i][0] = vx;
        cloth1.velocity[i][1] = vy;
        cloth1.velocity[i][2] = vz;
    }

    for (size_t i=0 ; i<cloth2.position.size() ; i++){
        float y = cloth2.position[i][0];
        float z = cloth2.position[i][1];
        float x = cloth2.position[i][2];

        cloth2.position[i][0] = x;
        cloth2.position[i][1] = y;
        cloth2.position[i][2] = z;

        float vy = cloth2.velocity[i][0];
        float vz = cloth2.velocity[i][1];
        float vx = cloth2.velocity[i][2];

        cloth2.velocity[i][0] = vx;
        cloth2.velocity[i][1] = vy;
        cloth2.velocity[i][2] = vz;
    }

}



void compute_deformation()
{
	float const t = timer.t;

        skinning_data.skeleton_current = skeleton_data.evaluate_global(t);
	visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index);

	// Compute skinning deformation
        skinning_LBS_compute(skinning_data.position_skinned, skinning_data.normal_skinned,
                skinning_data.skeleton_current, skinning_data.skeleton_rest_pose,
                skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
                rig);
	visual_data.surface_skinned.update_position(skinning_data.position_skinned);
	visual_data.surface_skinned.update_normal(skinning_data.normal_skinned);
	
}

void display_scene()
{
        //body

	if(user.gui.surface_skinned) 
		draw(visual_data.surface_skinned, scene);
	if (user.gui.wireframe_skinned)
		draw_wireframe(visual_data.surface_skinned, scene, {0.5f, 0.5f, 0.5f});

	draw(visual_data.skeleton_current, scene);

	if(user.gui.surface_rest_pose)
		draw(visual_data.surface_rest_pose, scene);
	if (user.gui.wireframe_rest_pose)
		draw_wireframe(visual_data.surface_rest_pose, scene, {0.5f, 0.5f, 0.5f});

	draw(visual_data.skeleton_rest_pose, scene);

        //ground

        draw(ground, scene);

        //cloth

        cloth1.visual.update_position(cloth1.position.data);
        normal_per_vertex(cloth1.position.data, cloth1.triangle_connectivity, cloth1.normal.data);
        cloth1.visual.update_normal(cloth1.normal.data);
        draw(cloth1.visual, scene);

        cloth2.visual.update_position(cloth2.position.data);
        normal_per_vertex(cloth2.position.data, cloth2.triangle_connectivity, cloth2.normal.data);
        cloth2.visual.update_normal(cloth2.normal.data);
        draw(cloth2.visual, scene);

}

void update_new_content(mesh& shape, GLuint texture_id)
{
        shape = shape.compute_normal();

	visual_data.surface_skinned.clear();
        visual_data.surface_skinned = mesh_drawable(shape);
	visual_data.surface_skinned.texture = texture_id;
        visual_data.surface_skinned_mesh = shape;

	visual_data.surface_rest_pose.clear();
	visual_data.surface_rest_pose = mesh_drawable(shape);
	visual_data.surface_rest_pose.texture = texture_id;

	skinning_data.position_rest_pose = shape.position;
	skinning_data.position_skinned = skinning_data.position_rest_pose;
	skinning_data.normal_rest_pose = shape.normal;
	skinning_data.normal_skinned = skinning_data.normal_rest_pose;

	skinning_data.skeleton_current = skeleton_data.rest_pose_global();
	skinning_data.skeleton_rest_pose = skinning_data.skeleton_current;

	visual_data.skeleton_current.clear();
	visual_data.skeleton_current = skeleton_drawable(skinning_data.skeleton_current, skeleton_data.parent_index);

	visual_data.skeleton_rest_pose.clear();
	visual_data.skeleton_rest_pose = skeleton_drawable(skinning_data.skeleton_rest_pose, skeleton_data.parent_index);
	
	timer.t_min = skeleton_data.animation_time[0];
	timer.t_max = skeleton_data.animation_time[skeleton_data.animation_time.size()-1];
	timer.t = skeleton_data.animation_time[0];
}

void display_interface()
{
	ImGui::Checkbox("Display frame", &user.gui.display_frame);
	ImGui::Spacing(); ImGui::Spacing();

	ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max, "%.2f s");
	ImGui::SliderFloat("Time Scale", &timer.scale, 0.05f, 2.0f, "%.2f s");

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Deformed "); 
	ImGui::Text("Surface: ");ImGui::SameLine();
	ImGui::Checkbox("Plain", &user.gui.surface_skinned); ImGui::SameLine();
	ImGui::Checkbox("Wireframe", &user.gui.wireframe_skinned);

	ImGui::Text("Skeleton: "); ImGui::SameLine();
	ImGui::Checkbox("Bones", &user.gui.skeleton_current_bone); ImGui::SameLine();
	ImGui::Checkbox("Frame", &user.gui.skeleton_current_frame); ImGui::SameLine();
	ImGui::Checkbox("Sphere", &user.gui.skeleton_current_sphere);

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Rest Pose");
	ImGui::Text("Surface: ");ImGui::SameLine();
	ImGui::Checkbox("Plain##Rest", &user.gui.surface_rest_pose); ImGui::SameLine();
	ImGui::Checkbox("Wireframe##Rest", &user.gui.wireframe_rest_pose);

	ImGui::Text("Skeleton: "); ImGui::SameLine();
	ImGui::Checkbox("Bones##Rest", &user.gui.skeleton_rest_pose_bone); ImGui::SameLine();
	ImGui::Checkbox("Frame##Rest", &user.gui.skeleton_rest_pose_frame); ImGui::SameLine();
	ImGui::Checkbox("Sphere##Rest", &user.gui.skeleton_rest_pose_sphere);

	ImGui::Spacing(); ImGui::Spacing();


	visual_data.skeleton_current.display_segments = user.gui.skeleton_current_bone;
	visual_data.skeleton_current.display_joint_frame = user.gui.skeleton_current_frame;
	visual_data.skeleton_current.display_joint_sphere = user.gui.skeleton_current_sphere;
	visual_data.skeleton_rest_pose.display_segments = user.gui.skeleton_rest_pose_bone;
	visual_data.skeleton_rest_pose.display_joint_frame = user.gui.skeleton_rest_pose_frame;
	visual_data.skeleton_rest_pose.display_joint_sphere = user.gui.skeleton_rest_pose_sphere;

        mesh new_shape;
        bool update = false;

        bool const xbot_dance = ImGui::Button("Dance"); ImGui::SameLine();
        bool const xbot_walk = ImGui::Button("Walk");

	GLuint texture_id = mesh_drawable::default_texture;
        if (xbot_walk || xbot_dance)  load_skinning_data("../../../assets/xbot/", skeleton_data, rig, new_shape);
        if (xbot_walk) load_skinning_anim("../../../assets/xbot/anim_walk/", skeleton_data);
        if (xbot_dance) load_skinning_anim("../../../assets/xbot/anim_dance/", skeleton_data);

        if (xbot_dance){
            timer.t = 0.f;
            mesh const cloth_mesh1 = mesh_primitive_grid({1,0,cloth1.side_length_y},{1,0,0},{1,cloth1.side_length_x,0},{1,cloth1.side_length_x,cloth1.side_length_y},cloth1.N_cloth,cloth1.N_cloth);
            buffer<vec3> precomputed_initial_positions; read_from_file("../../../assets/cloth/initial_positions_skirt.txt",precomputed_initial_positions);
            cloth1.position = grid_2D<vec3>::from_buffer(precomputed_initial_positions, cloth1.N_cloth, cloth1.N_cloth);

            cloth1.velocity.clear();
            cloth1.velocity.resize(cloth1.N_cloth*cloth1.N_cloth);

            cloth1.forces.clear();
            cloth1.forces.resize(cloth1.N_cloth,cloth1.N_cloth);

            cloth1.triangle_connectivity = cloth_mesh1.connectivity;
            cloth1.normal = grid_2D<vec3>::from_buffer(cloth_mesh1.normal, cloth1.N_cloth, cloth1.N_cloth);
        }
        if (xbot_walk){
            timer.t = 1.f;
            mesh const cloth_mesh1 = mesh_primitive_grid({1,0,cloth1.side_length_y},{1,0,0},{1,cloth1.side_length_x,0},{1,cloth1.side_length_x,cloth1.side_length_y},cloth1.N_cloth,cloth1.N_cloth);
            buffer<vec3> precomputed_initial_positions; read_from_file("../../../assets/cloth/initial_positions_skirt.txt",precomputed_initial_positions);
            cloth1.position = grid_2D<vec3>::from_buffer(precomputed_initial_positions, cloth1.N_cloth, cloth1.N_cloth);

            cloth1.velocity.clear();
            cloth1.velocity.resize(cloth1.N_cloth*cloth1.N_cloth);

            cloth1.forces.clear();
            cloth1.forces.resize(cloth1.N_cloth,cloth1.N_cloth);

            cloth1.triangle_connectivity = cloth_mesh1.connectivity;
            cloth1.normal = grid_2D<vec3>::from_buffer(cloth_mesh1.normal, cloth1.N_cloth, cloth1.N_cloth);
        }

        if (xbot_walk || xbot_dance) {
            update = true;
            normalize_weights(rig.weight);
            float const scaling = 0.005f;
            for(auto& p: new_shape.position) p *= scaling;
            skeleton_data.scale(scaling);
        }

        bool prec_action = user.gui.action;
        ImGui::Checkbox("Run animation", &user.gui.action);
        if (user.gui.action != prec_action){
            timer.t = 0.f;
            mesh const cloth_mesh1 = mesh_primitive_grid({1,0,cloth1.side_length_y},{1,0,0},{1,cloth1.side_length_x,0},{1,cloth1.side_length_x,cloth1.side_length_y},cloth1.N_cloth,cloth1.N_cloth);
            buffer<vec3> precomputed_initial_positions; read_from_file("../../../assets/cloth/initial_positions_skirt.txt",precomputed_initial_positions);
            cloth1.position = grid_2D<vec3>::from_buffer(precomputed_initial_positions, cloth1.N_cloth, cloth1.N_cloth);

            cloth1.velocity.clear();
            cloth1.velocity.resize(cloth1.N_cloth*cloth1.N_cloth);

            cloth1.forces.clear();
            cloth1.forces.resize(cloth1.N_cloth,cloth1.N_cloth);

            cloth1.triangle_connectivity = cloth_mesh1.connectivity;
            cloth1.normal = grid_2D<vec3>::from_buffer(cloth_mesh1.normal, cloth1.N_cloth, cloth1.N_cloth);
        }

        if (user.gui.action){
            if (update){
                update_new_content(new_shape, texture_id);
            }
        }

}



void window_size_callback(GLFWwindow* , int width, int height)
{
	glViewport(0, 0, width, height);
	float const aspect = width / static_cast<float>(height);
	scene.projection = projection_perspective(50.0f*pi/180.0f, aspect, 0.1f, 100.0f);
}


void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
	vec2 const& p0 = user.mouse_prev;

	glfw_state state = glfw_current_state(window);

	auto& camera = scene.camera;
	if(!user.cursor_on_gui){
		if(state.mouse_click_left && !state.key_ctrl)
			scene.camera.manipulator_rotate_trackball(p0, p1);
		if(state.mouse_click_left && state.key_ctrl)
			camera.manipulator_translate_in_plane(p1-p0);
		if(state.mouse_click_right)
			camera.manipulator_scale_distance_to_center( (p1-p0).y );
	}

	user.mouse_prev = p1;
}

void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
	opengl_uniform(shader, "projection", current_scene.projection);
	opengl_uniform(shader, "view", scene.camera.matrix_view());
	opengl_uniform(shader, "light", scene.light, false);
}



