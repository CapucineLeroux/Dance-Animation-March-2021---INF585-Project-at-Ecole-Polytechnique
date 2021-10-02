#pragma once

#include "vcl/vcl.hpp"
#include "skeleton.hpp"

struct simulation_parameters
{
    float mass_total; // total mass of the cloth
    float K; // stiffness
    float mu; // damping    
};

struct obstacle_parameters
{
    std::vector<vcl::vec3> sphere_centers;
    std::vector<float> sphere_radiuses;
    std::vector<vcl::vec3> cylinder_centers; //twice as big as the radius list, with two consecutive centers for each cylinder
    std::vector<float> cylinder_radiuses;
};


void initialize_simulation_parameters(simulation_parameters& parameters,simulation_parameters& parameters2);
void compute_forces(vcl::grid_2D<vcl::vec3>& force, vcl::grid_2D<vcl::vec3> const& position, vcl::grid_2D<vcl::vec3> const& velocity, vcl::grid_2D<vcl::vec3>& normals, float side_length_x, float side_length_y, simulation_parameters const& parameters);

void numerical_integration(vcl::grid_2D<vcl::vec3>& position, vcl::grid_2D<vcl::vec3>& velocity, vcl::grid_2D<vcl::vec3> const& forces, float mass, float dt);

void apply_constraints(vcl::grid_2D<vcl::vec3>& position1, vcl::grid_2D<vcl::vec3>& velocity1, float side_length_x1, float side_length_y1, vcl::grid_2D<vcl::vec3>& position2, vcl::grid_2D<vcl::vec3>& velocity2, float side_length_x2, float side_length_y2, obstacle_parameters body_obstacles, vcl::buffer<vcl::affine_rt> skeleton_current);
void apply_external_constraints(vcl::grid_2D<vcl::vec3>& position, vcl::grid_2D<vcl::vec3>& velocity, obstacle_parameters body_obstacles);
bool detect_simulation_divergence(vcl::grid_2D<vcl::vec3> const& force, vcl::grid_2D<vcl::vec3> const& position);
