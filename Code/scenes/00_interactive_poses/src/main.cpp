#include "vcl/vcl.hpp"
#include <iostream>

#include "skeleton.hpp"
#include "skeleton_drawable.hpp"
#include "skinning.hpp"
#include "skinning_loader.hpp"

using namespace vcl;
using namespace std;


//To save the different poses
buffer<buffer<quaternion>> skeleton_animation_quaternion_local;
buffer<buffer<vec3>> skeleton_animation_position_local;
buffer<float> skeleton_animation_time;
bool need_to_save = false;
bool S_is_pressed = false;
bool need_to_final_save = false;
bool F_is_pressed = false;

// current context of picking
struct picking_parameters {
        bool active;              // true if a vertex has been selected
        int index;                // the index of the selected vertex
        vcl::vec2 screen_clicked; // 2D position on screen where the mouse was clicked when the picking occured
        vcl::vec3 p_clicked;      // The 3D position corresponding to the picking
};

timer_event_periodic timer_update_shape(0.05f);
bool require_shape_update=false;

struct gui_parameters {

        bool display_frame = true;
        bool surface = false;
        bool wireframe = false;

        bool skeleton_bone = true;
        bool skeleton_frame = false;
        bool skeleton_sphere = true;
        bool obstacles = false;
};

struct user_interaction_parameters {

        vec2 mouse_prev;
        timer_fps fps_record;
        gui_parameters gui;
        mesh_drawable global_frame;
        bool cursor_on_gui;

        picking_parameters picking;
};
user_interaction_parameters user;

struct scene_environment
{
        camera_around_center camera;
        mat4 projection;
        mat4 projection_inverse;
        vec3 light;
};
scene_environment scene;

struct visual_shapes_parameters
{
        mesh_drawable surface;
        skeleton_drawable skeleton;
};
visual_shapes_parameters visual_data;

struct skinning_current_data
{
        buffer<vec3> position;
        buffer<vec3> normal;
        buffer<affine_rt> skeleton;

        buffer<vec3> position_rest_pose;
        buffer<vec3> normal_rest_pose;
        buffer<affine_rt> skeleton_rest_pose;
};

skeleton_animation_structure skeleton_data;
rig_structure rig;
skinning_current_data skinning_data;
GLuint texture_id = mesh_drawable::default_texture;
mesh shape;
vector<mesh_drawable> body_obstacles;

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);
template <typename T> void write_file (string file_name, buffer<buffer<T>> data);

void initialize_data();
void display_scene();
void display_interface();
void compute_deformation();
void update_new_content();


int main(int, char* argv[])
{

    buffer<buffer<int>> vertex_correspondance;
    mesh test_shape = dae_mesh_extractor("../../../assets/xbot/", vertex_correspondance);

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

        std::cout<<"Start animation loop ..."<<std::endl;
        user.fps_record.start();
        glEnable(GL_DEPTH_TEST);
        while (!glfwWindowShouldClose(window))
        {
                scene.light = scene.camera.position();
                user.fps_record.update();

                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT);
                glClear(GL_DEPTH_BUFFER_BIT);
                imgui_create_frame();
                if(user.fps_record.event) {
                        std::string const title = "VCL Display - "+str(user.fps_record.fps)+" fps";
                        glfwSetWindowTitle(window, title.c_str());
                }

                timer_update_shape.update();
                if(timer_update_shape.event && require_shape_update) // scheduling system to avoid too many times the FFD deformation
                {
                    normalize_weights(rig.weight);
                    update_new_content();
                    compute_deformation();
                    require_shape_update = false;
                }

                ImGui::Begin("GUI",NULL,ImGuiWindowFlags_AlwaysAutoResize);
                user.cursor_on_gui = ImGui::IsAnyWindowFocused();

                if(user.gui.display_frame)
                        draw(user.global_frame, scene);


                display_interface();
                display_scene();

                ImGui::End();
                imgui_render_frame(window);
                glfwSwapBuffers(window);
                glfwPollEvents();

                //Press S to save the current pose
                if (glfwGetKey(window,GLFW_KEY_S)) { S_is_pressed = true; }
                if (!glfwGetKey(window,GLFW_KEY_S) && S_is_pressed){ need_to_save = true; S_is_pressed = false;}
                if (need_to_save){

                    cout<<"Pose saved"<<endl;

                    float const scaling = 0.005f;
                    skeleton_data.scale(1.f/scaling);

                    buffer<quaternion> current_pose_rotations;
                    buffer<vec3> current_pose_translations;

                    for (size_t i=0 ; i<skeleton_data.rest_pose_local.size() ; i++){
                        current_pose_rotations.push_back(skeleton_data.rest_pose_local[i].rotate.data);
                        current_pose_translations.push_back(skeleton_data.rest_pose_local[i].translate);
                    }

                    skeleton_animation_quaternion_local.push_back(current_pose_rotations);
                    skeleton_animation_position_local.push_back(current_pose_translations);

                    need_to_save = false;
                    skeleton_data.scale(scaling);

                }

                //Press F to write down all the poses in a file
                if (glfwGetKey(window,GLFW_KEY_F)) { F_is_pressed = true; }
                if (!glfwGetKey(window,GLFW_KEY_F) && F_is_pressed){ need_to_final_save = true; F_is_pressed = false;}
                if (need_to_final_save){

                    cout<<"Final animations poses saved"<<endl;
                    //add the first pose as last pose to have a cycle movement
                    skeleton_animation_quaternion_local.push_back(skeleton_animation_quaternion_local[0]);
                    skeleton_animation_position_local.push_back(skeleton_animation_position_local[0]);

                    //space the poses equally through time (between time 0 and 1)
                    int N_poses = skeleton_animation_quaternion_local.size();
                    for (int i=0 ; i<N_poses ; i++){
                        skeleton_animation_time.push_back((float)i);
                    }

                    //save in three files the information
                    write_file("../../../assets/xbot/anim_dance/skeleton_animation_quaternion_local.txt",skeleton_animation_quaternion_local);
                    write_file("../../../assets/xbot/anim_dance/skeleton_animation_position_local.txt",skeleton_animation_position_local);
                    ofstream myfile ("../../../assets/xbot/anim_dance/skeleton_animation_time.txt");
                    if (myfile.is_open()){myfile << skeleton_animation_time; myfile.close();}
                    else cout << "Unable to open file";

                    need_to_final_save = false;

                }
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
        mesh_drawable::default_shader = shader_mesh;
        mesh_drawable::default_texture = texture_white;
        curve_drawable::default_shader = shader_uniform_color;
        segments_drawable::default_shader = shader_uniform_color;

        user.global_frame = mesh_drawable(mesh_primitive_frame());
        user.gui.display_frame = false;
        scene.camera.distance_to_center = 2.5f;

        load_skinning_data("../../../assets/xbot/", skeleton_data, rig, shape);

        normalize_weights(rig.weight);
        float const scaling = 0.005f;
        for(auto& p: shape.position) p *= scaling;
        skeleton_data.scale(scaling);

        skinning_data.position_rest_pose = shape.position;
        skinning_data.normal_rest_pose = shape.normal;
        skinning_data.skeleton_rest_pose = skeleton_data.rest_pose_global();

        update_new_content();

        // create body obstacles
        vec3 p,p1,p2;
        float r;

        //head
        p = 0.5f*skinning_data.skeleton[15].translate + 0.5f*skinning_data.skeleton[20].translate;
        r = 0.06f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
        //neck
        p = 0.5f*skinning_data.skeleton[10].translate + 0.5f*skinning_data.skeleton[15].translate;
        r = 0.03f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        //arms
        for(const auto& i : {16,17,23,24,25,26}) {
            p1 = skinning_data.skeleton[i].translate;
            p2 = skinning_data.skeleton[skeleton_data.parent_index[i]].translate;
            if (i==25 || i==26){r = 0.02f;}
            else {r = 0.03f;}
            body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p1,p2)));
            body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p1)));
            body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p2)));
        }

        //torax
        p = 0.5f*skinning_data.skeleton[7].translate + 0.25f*skinning_data.skeleton[11].translate + 0.25f*skinning_data.skeleton[12].translate;
        p.y -= 0.01f;
        p.z += 0.01f;
        r = 0.08f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        p = 0.5f*skinning_data.skeleton[7].translate + 0.5f*skinning_data.skeleton[4].translate;
        p.y += 0.01f;
        p.z += 0.02f;
        r = 0.065f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        p = skinning_data.skeleton[4].translate;
        r = 0.055f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        p = skinning_data.skeleton[0].translate;
        r = 0.065f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        p.z -= 0.02f;
        p.y -= 0.01f;
        r = 0.07f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        //legs
        p = skinning_data.skeleton[2].translate;
        r = 0.06f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
        p = skinning_data.skeleton[3].translate;
        r = 0.06f;
        body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));

        for(const auto& i : {5,6,8,9,13,14,18,19}) {
            p1 = skinning_data.skeleton[i].translate;
            p2 = skinning_data.skeleton[skeleton_data.parent_index[i]].translate;

            if (i==5){p2.x += 0.02f; p = 0.5f*p1+0.5f*p2; p.z += 0.01f;}
            if (i==6){p2.x -= 0.02f; p = 0.5f*p1+0.5f*p2; p.z += 0.01f;}

            if (i==5 || i==6){
                r = 0.05f;
                body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p,p2)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p2)));
                r = 0.04f;
                body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p,p1)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p1)));
            }
            else if (i==8 || i==9){
                p = 0.5f*p1+0.5f*p2;
                r = 0.04f;
                body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p,p2)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p2)));
                r = 0.03f;
                body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p,p1)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p1)));
            }
            else {
                r = 0.03f;
                body_obstacles.push_back(mesh_drawable(mesh_primitive_cylinder(r,p1,p2)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p1)));
                body_obstacles.push_back(mesh_drawable(mesh_primitive_sphere(r,p2)));
            }
        }


}

void compute_deformation()
{
        // Compute skinning deformation

        skinning_LBS_compute(skinning_data.position, skinning_data.normal,
                skinning_data.skeleton, skinning_data.skeleton_rest_pose,
                skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
                rig);
        visual_data.surface.update_position(skinning_data.position);
        visual_data.surface.update_normal(skinning_data.normal);

}

void display_scene()
{

        if(user.gui.surface)
                draw(visual_data.surface, scene);
        if (user.gui.wireframe)
                draw_wireframe(visual_data.surface, scene, {0.5f, 0.5f, 0.5f});

        draw(visual_data.skeleton, scene);

        if (user.gui.obstacles){
            for (int i=0 ; i<body_obstacles.size() ; i++){
                draw(body_obstacles[i],scene);
            }
        }

}

void update_new_content()
{
        texture_id = mesh_drawable::default_texture;
        visual_data.surface.clear();
        visual_data.surface = mesh_drawable(shape);
        visual_data.surface.texture = texture_id;

        skinning_data.position = shape.position;
        skinning_data.normal = shape.normal;

        skinning_data.skeleton = skeleton_data.rest_pose_global();

        visual_data.skeleton.clear();
        visual_data.skeleton = skeleton_drawable(skinning_data.skeleton, skeleton_data.parent_index);

}

void display_interface()
{
        ImGui::Checkbox("Display frame", &user.gui.display_frame);
        ImGui::Spacing(); ImGui::Spacing();

        ImGui::Text("Surface: ");ImGui::SameLine();
        ImGui::Checkbox("Plain##Rest", &user.gui.surface); ImGui::SameLine();
        ImGui::Checkbox("Wireframe##Rest", &user.gui.wireframe);
        ImGui::Checkbox("Proxies##Rest", &user.gui.obstacles);

        ImGui::Text("Skeleton: "); ImGui::SameLine();
        ImGui::Checkbox("Bones##Rest", &user.gui.skeleton_bone); ImGui::SameLine();
        ImGui::Checkbox("Frame##Rest", &user.gui.skeleton_frame); ImGui::SameLine();
        ImGui::Checkbox("Sphere##Rest", &user.gui.skeleton_sphere);

        ImGui::Spacing(); ImGui::Spacing();


        visual_data.skeleton.display_segments = user.gui.skeleton_bone;
        visual_data.skeleton.display_joint_frame = user.gui.skeleton_frame;
        visual_data.skeleton.display_joint_sphere = user.gui.skeleton_sphere;

}



void window_size_callback(GLFWwindow* , int width, int height)
{
        glViewport(0, 0, width, height);
        float const aspect = width / static_cast<float>(height);
        scene.projection = projection_perspective(50.0f*pi/180.0f, aspect, 0.1f, 100.0f);
        scene.projection_inverse = projection_perspective_inverse(50.0f*pi/180.0f, aspect, 0.1f, 100.0f);
}


void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
        vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
        vec2 const& p0 = user.mouse_prev;

        glfw_state state = glfw_current_state(window);

        auto& camera = scene.camera;
        if(!user.cursor_on_gui){
            if(!state.key_shift){
                if(state.mouse_click_left && !state.key_ctrl)
                        scene.camera.manipulator_rotate_trackball(p0, p1);
                if(state.mouse_click_left && state.key_ctrl)
                        camera.manipulator_translate_in_plane(p1-p0);
                if(state.mouse_click_right)
                        camera.manipulator_scale_distance_to_center( (p1-p0).y );
            }
        }


        if(state.key_shift) //Handles the interactive modification of the skeleton, need to press shift to modify a joint
        {
            buffer<vec3> joint_positions = get_joint_position(skeleton_data);

                if (!state.mouse_click_left) // Handle grid point picking here
                {
                        vec3 const ray_direction = camera_ray_direction(scene.camera.matrix_frame(),scene.projection_inverse, p1);
                        vec3 const ray_origin = scene.camera.position();

                        int index=0;
                        intersection_structure intersection = intersection_ray_spheres_closest(ray_origin, ray_direction, joint_positions, 0.01f, &index);

                        if (intersection.valid == true) {
                                user.picking = {true, index, p1, intersection.position};
                        }
                }
                if (state.mouse_click_left && user.picking.active) // Joint modification
                {
                        // Get vector orthogonal to camera orientation
                        vec3 const n = scene.camera.front();
                        // Compute intersection between current ray and the plane orthogonal to the view direction and passing by the selected object
                        vec3 const ray_direction = camera_ray_direction(scene.camera.matrix_frame(),scene.projection_inverse, p1);
                        vec3 const ray_origin = scene.camera.position();

                        intersection_structure intersection = intersection_ray_plane(ray_origin, ray_direction, user.picking.p_clicked, n);
                        vec3 cursor_3D_position = intersection.position;

                        int p_index = user.picking.index;
                        int parent_index = skeleton_data.parent_index[p_index];

                        //for the root joint, we only apply a translation to it
                        if (p_index == 0){
                            skeleton_data.rest_pose_local[0].translate = cursor_3D_position;
                            require_shape_update = true;
                        }

                        else {


                            int grand_parent_index = skeleton_data.parent_index[parent_index];

                            if (parent_index == 0){
                                grand_parent_index = 0;
                            }

                            vec3 p = skeleton_data.rest_pose_global()[p_index].translate;
                            vec3 parent = skeleton_data.rest_pose_global()[parent_index].translate;
                            const float bone_length = norm(p-parent);
                            vec3 new_p = parent + bone_length*(cursor_3D_position-parent)/norm(cursor_3D_position-parent);

                            //rotate the parent (unapplicable if the joint is a root)
                            float rotation_angle = acos(dot(p-parent,new_p-parent)/(norm(p-parent)*norm(new_p-parent)));
                            vec3 rotation_axis = cross(p-parent,new_p-parent)/norm(cross(p-parent,new_p-parent));
                            rotation R = rotation(rotation_axis,rotation_angle);
                            rotation R_grand_parent_global = skeleton_data.rest_pose_global()[grand_parent_index].rotate;
                            rotation R_parent_local = skeleton_data.rest_pose_local[parent_index].rotate;
                            skeleton_data.rest_pose_local[parent_index].rotate = inverse(R_grand_parent_global)*R*R_grand_parent_global*R_parent_local;

                            require_shape_update = true;
                        }
                }
        }
        else
                user.picking.active = false; // Unselect picking when shift is released


        user.mouse_prev = p1;
}

void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
        opengl_uniform(shader, "projection", current_scene.projection);
        opengl_uniform(shader, "view", scene.camera.matrix_view());
        opengl_uniform(shader, "light", scene.light, false);
}


template <typename T>
void write_file (string file_name, buffer<buffer<T>> data){

    ofstream myfile (file_name);
    if (myfile.is_open())
    {
        for (size_t i=0 ; i<data.size() ; i++){
            myfile << data[i];
            myfile << "\n";
        }

        myfile.close();
    }
    else cout << "Unable to open file";

};





