#include "cloth_simulation.hpp"

using namespace vcl;
using namespace std;


// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
void compute_forces(grid_2D<vec3>& force, grid_2D<vec3> const& position, grid_2D<vec3> const& velocity, grid_2D<vec3>& normals, float side_length_x, float side_length_y, simulation_parameters const& parameters)
{
    size_t const N = force.size();        // Total number of particles of the cloth Nu x Nv
    int const N_dim = int(force.dimension[0]); // Number of particles along one dimension (square dimension)

    float const K  = parameters.K;
    float const m  = parameters.mass_total/N;
    float const mu = parameters.mu;
    float const	L0x = side_length_x/(N_dim-1.0f);
    float const	L0y = side_length_y/(N_dim-1.0f);
    float const L0d = pow(L0x*L0x+L0y*L0y,0.5);

    // Gravity
    const vec3 g = {0,0,-9.81f};
    for(size_t k=0; k<N; ++k)
        force[k] = m*g;

    // Drag
    for(size_t k=0; k<N; ++k)
        force[k] += -mu*m*velocity[k];

    vec3 p, p_right, p_left, p_down, p_up, p1, p2, p3, p4, F_right, F_left, F_down, F_up, F1, F2, F3, F4;
    //1 = up right diagonal
    //2 = down right diagonal
    //3 = down left diagonal
    //4 = up left diagonal

    //bigger interior
    for (int j=1; j<N_dim-1; ++j){
        for (int i=1; i<N_dim-1; ++i){
            p = position(i,j);
            p_right = position(i,j+1);
            p_left = position(i,j-1);
            p_down = position(i+1,j);
            p_up = position(i-1,j);
            p1 = position(i-1,j+1);
            p2 = position(i+1,j+1);
            p3 = position(i+1,j-1);
            p4 = position(i-1,j-1);
            F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
            F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
            F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
            F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
            F1 = K*(norm(p1-p)-L0d)*(p1-p)/norm(p1-p);
            F2 = K*(norm(p2-p)-L0d)*(p2-p)/norm(p2-p);
            F3 = K*(norm(p3-p)-L0d)*(p3-p)/norm(p3-p);
            F4 = K*(norm(p4-p)-L0d)*(p4-p)/norm(p4-p);
            force(i,j) += F_right + F_left + F_down + F_up + F1 + F2 + F3 + F4;
        }
    }

    //corners
    p = position(0,0);
    p_right = position(0,1);
    p_down = position(1,0);
    p2 = position(1,1);
    F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
    F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
    F2 = K*(norm(p2-p)-L0d)*(p2-p)/norm(p2-p);
    force(0,0) += F_right + F_down + F2;

    p = position(0,N_dim-1);
    p_left = position(0,N_dim-2);
    p_down = position(1,N_dim-1);
    p3 = position(1,N_dim-2);
    F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
    F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
    F3 = K*(norm(p3-p)-L0d)*(p3-p)/norm(p3-p);
    force(0,N_dim-1) += F_left + F_down + F3;

    p = position(N_dim-1,0);
    p_right = position(N_dim-1,1);
    p_up = position(N_dim-2,0);
    p1 = position(N_dim-2,1);
    F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
    F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
    F1 = K*(norm(p1-p)-L0d)*(p1-p)/norm(p1-p);
    force(N_dim-1,0) += F_right + F_up + F1;

    p = position(N_dim-1,N_dim-1);
    p_left = position(N_dim-1,N_dim-2);
    p_up = position(N_dim-2,N_dim-1);
    p4 = position(N_dim-2,N_dim-2);
    F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
    F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
    F4 = K*(norm(p4-p)-L0d)*(p4-p)/norm(p4-p);
    force(N_dim-1,N_dim-1) += F_left + F_up + F4;

    //up borders
    for (int j=1; j<N_dim-1; ++j){
        p = position(0,j);
        p_right = position(0,j+1);
        p_left = position(0,j-1);
        p_down = position(1,j);
        p2 = position(1,j+1);
        p3 = position(1,j-1);
        F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
        F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
        F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
        F2 = K*(norm(p2-p)-L0d)*(p2-p)/norm(p2-p);
        F3 = K*(norm(p3-p)-L0d)*(p3-p)/norm(p3-p);
        force(0,j) += F_right + F_left + F_down + F2 + F3;
    }

    //down borders
    for (int j=1; j<N_dim-1; ++j){
        p = position(N_dim-1,j);
        p_right = position(N_dim-1,j+1);
        p_left = position(N_dim-1,j-1);
        p_up = position(N_dim-2,j);
        p1 = position(N_dim-2,j+1);
        p4 = position(N_dim-2,j-1);
        F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
        F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
        F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
        F1 = K*(norm(p1-p)-L0d)*(p1-p)/norm(p1-p);
        F4 = K*(norm(p4-p)-L0d)*(p4-p)/norm(p4-p);
        force(N_dim-1,j) += F_right + F_left + F_up + F1 + F4;
    }

    //left borders
    for (int i=1; i<N_dim-1; ++i){
        p = position(i,0);
        p_right = position(i,1);
        p_down = position(i+1,0);
        p_up = position(i-1,0);
        p1 = position(i-1,1);
        p2 = position(i+1,1);
        F_right = K*(norm(p_right-p)-L0x)*(p_right-p)/norm(p_right-p);
        F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
        F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
        F1 = K*(norm(p1-p)-L0d)*(p1-p)/norm(p1-p);
        F2 = K*(norm(p2-p)-L0d)*(p2-p)/norm(p2-p);
        force(i,0) += F_right + F_down + F_up + F1 + F2;
    }

    //right borders
    for (int i=1; i<N_dim-1; ++i){
        p = position(i,N_dim-1);
        p_left = position(i,N_dim-2);
        p_down = position(i+1,N_dim-1);
        p_up = position(i-1,N_dim-1);
        p3 = position(i+1,N_dim-2);
        p4 = position(i-1,N_dim-2);
        F_left = K*(norm(p_left-p)-L0x)*(p_left-p)/norm(p_left-p);
        F_down = K*(norm(p_down-p)-L0y)*(p_down-p)/norm(p_down-p);
        F_up = K*(norm(p_up-p)-L0y)*(p_up-p)/norm(p_up-p);
        F3 = K*(norm(p3-p)-L0d)*(p3-p)/norm(p3-p);
        F4 = K*(norm(p4-p)-L0d)*(p4-p)/norm(p4-p);
        force(i,N_dim-1) += F_left + F_down + F_up + F3 + F4;
    }

    //bending springs

    //smaller interior
    //2-neighbourhood
    for (int j=2; j<N_dim-2; ++j){
        for (int i=2; i<N_dim-2; ++i){
            p = position(i,j);
            p_right = position(i,j+2);
            p_left = position(i,j-2);
            p_down = position(i+2,j);
            p_up = position(i-2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_right + F_left + F_down + F_up;
        }
    }

    //corners
    for (int i=0 ; i<=1 ; i++){
        for (int j=0 ; j<=1 ; j++){
            p = position(i,j);
            p_right = position(i,j+2);
            p_down = position(i+2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            force(i,j) += F_right + F_down;
        }
    }

    for (int i=0 ; i<=1 ; i++){
        for (int j=N_dim-2 ; j<=N_dim-1 ; j++){
            p = position(i,j);
            p_left = position(i,j-2);
            p_down = position(i+2,j);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            force(i,j) += F_left + F_down;
        }
    }

    for (int i=N_dim-2 ; i<=N_dim-1 ; i++){
        for (int j=0 ; j<=1 ; j++){
            p = position(i,j);
            p_right = position(i,j+2);
            p_up = position(i-2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_right + F_up;
        }
    }

    for (int i=N_dim-2 ; i<=N_dim-1 ; i++){
        for (int j=N_dim-2 ; j<=N_dim-1 ; j++){
            p = position(i,j);
            p_left = position(i,j-2);
            p_up = position(i-2,j);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_left + F_up;
        }
    }

    //up double borders
    for (int j=2; j<N_dim-2; ++j){
        for (int i=0 ; i<=1 ; i++){
            p = position(i,j);
            p_right = position(i,j+2);
            p_left = position(i,j-2);
            p_down = position(i+2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            force(i,j) += F_right + F_left + F_down;
        }
    }

    //down double borders
    for (int j=2; j<N_dim-2; ++j){
        for (int i=N_dim-2 ; i<=N_dim-1 ; i++){
            p = position(i,j);
            p_right = position(i,j+2);
            p_left = position(i,j-2);
            p_up = position(i-2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_right + F_left + F_up;
        }
    }

    //left double borders
    for (int i=2; i<N_dim-2; ++i){
        for (int j=0 ; j<=1 ; j++){
            p = position(i,j);
            p_right = position(i,j+2);
            p_down = position(i+2,j);
            p_up = position(i-2,j);
            F_right = K*(norm(p_right-p)-2.f*L0x)*(p_right-p)/norm(p_right-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_right + F_down + F_up;
        }
    }

    //right double borders
    for (int i=2; i<N_dim-2; ++i){
        for (int j=N_dim-2 ; j<=N_dim-1 ; j++){
            p = position(i,j);
            p_left = position(i,j-2);
            p_down = position(i+2,j);
            p_up = position(i-2,j);
            F_left = K*(norm(p_left-p)-2.f*L0x)*(p_left-p)/norm(p_left-p);
            F_down = K*(norm(p_down-p)-2.f*L0y)*(p_down-p)/norm(p_down-p);
            F_up = K*(norm(p_up-p)-2.f*L0y)*(p_up-p)/norm(p_up-p);
            force(i,j) += F_left + F_down + F_up;
        }
    }


}

void numerical_integration(grid_2D<vec3>& position, grid_2D<vec3>& velocity, grid_2D<vec3> const& force, float mass, float dt)
{
    size_t const N = position.size();

    for(size_t k=0; k<N; ++k)
    {
        velocity[k] = velocity[k] + dt*force[k]/mass;
        position[k] = position[k] + dt*velocity[k];
    }

}


void initialize_simulation_parameters(simulation_parameters& parameters,simulation_parameters& parameters2)
{
	parameters.mass_total = 0.8f;
        parameters.K  = 5.0f;
	parameters.mu = 10.0f;

        parameters2.mass_total = 0.8f;
        parameters2.K  = 5.0f;
        parameters2.mu = 10.0f;
}



void apply_constraints(grid_2D<vec3>& position1, grid_2D<vec3>& velocity1, float side_length_x1, float side_length_y1, grid_2D<vec3>& position2, grid_2D<vec3>& velocity2, float side_length_x2, float side_length_y2, obstacle_parameters body_obstacles, buffer<affine_rt> skeleton_current)
{
    //internal constraints

        //cloth 1

    int N_cloth1 = int(position1.dimension[0]);
    float length_x = side_length_x1;
    float length_y = side_length_y1;

            //keeps the waist collar in place

    float r = length_x/(2.f*pi);
    vec3 center = skeleton_current[1].translate;
    center.y += 0.01;
    float z0 = center.y;

    for (int i=0 ; i<N_cloth1 ; i++){
        float theta = float(i)*2*pi/float(N_cloth1-1) + pi/2;
        position1(i,0) = {center.x+r*sin(theta),z0,center.z+r*cos(theta)};
    }

            //closes the cloth at the top 3rd

    for (int j=0 ; j<int(N_cloth1/3.5) ; j++){
        float theta = pi/2;
        float h = z0 - float(j)*length_y/float(N_cloth1-1);
        r += 0.005f;
        position1(0,j) = {center.x+r*sin(theta),h,center.z+r*cos(theta)};
        position1(N_cloth1-1,j) = position1(0,j);
    }


        //cloth 2

    int N_cloth2 = int(position2.dimension[0]);
    length_x = side_length_x2;
    length_y = side_length_y2;

        //keep the neck collar in place

    r = length_x/(2.f*pi);
    center = skeleton_current[10].translate;
    center.y += 0.01;
    z0 = center.y;

    for (int i=0 ; i<N_cloth2 ; i++){
        float theta = float(i)*2*pi/float(N_cloth2-1) + pi;
        position2(i,0) = {center.x+r*sin(theta),z0,center.z+r*cos(theta)};
    }

            //keep the waist collar in place

    center = skeleton_current[1].translate;
    z0 = center.y;

    for (int i=0 ; i<N_cloth2 ; i++){
        float theta = float(i)*2*pi/float(N_cloth2-1) + pi;
        position2(i,N_cloth2-1) = {center.x+r*sin(theta),z0,center.z+r*cos(theta)};
    }


    // Apply external constraints

    apply_external_constraints(position1,velocity1,body_obstacles);
    apply_external_constraints(position2,velocity2,body_obstacles);

}


void apply_external_constraints(grid_2D<vec3>& position, grid_2D<vec3>& velocity, obstacle_parameters body_obstacles){

    size_t N = position.size();

    //obstacles are planes, spheres and cylinders, we set an epsilon of collision detection for each type
    float epsilon_ground =  5e-3f;
    float epsilon_sphere = 2*epsilon_ground;
    float epsilon_cylinder = 2*epsilon_ground;
    float y_min = 0.f; //ground limit

    float t,r;
    vec3 p,v,p_sphere,p2,p1, proj_position, n, u;

    //See each cloth particle
    for (size_t i=0 ; i<N ; i++){

            p = position[i];
            v = velocity[i];

            //collision with spheres
            for (size_t k=0 ; k<body_obstacles.sphere_radiuses.size() ; k++){

                p_sphere = body_obstacles.sphere_centers[k];
                r = body_obstacles.sphere_radiuses[k];

                if (norm(p-p_sphere)<r+epsilon_sphere) {

                    n = (p-p_sphere)/norm(p-p_sphere);
                    position[i] = p_sphere + (r+epsilon_sphere)*n;
                    velocity[i] = v-dot(v,n)*n;

                }
            }

            //collision with rounded cylinders
            for (size_t k=0 ; k<body_obstacles.cylinder_radiuses.size() ; k++){

                p1 = body_obstacles.cylinder_centers[2*k];
                p2 = body_obstacles.cylinder_centers[2*k+1];
                r = body_obstacles.cylinder_radiuses[k];

                t = dot(p-p1,p2-p1)/dot(p2-p1,p2-p1);
                proj_position = (1.f-t)*p1 + t*p2;

                //check if vertex inside the bone cylinder
                if (-(r+epsilon_cylinder)/norm(p1-p2)<=t && t<=1+(r+epsilon_cylinder)/norm(p1-p2)) {

                    if (0.f<=t && t<=1.f && norm(p-proj_position)<r+epsilon_cylinder){
                        n = (p-proj_position)/norm(p-proj_position);
                        position[i] = proj_position + (r+epsilon_cylinder)*n;
                        velocity[i] = v - dot(v,n)*n;
                    }
                    if (t<0.f && norm(p-p1)<r+epsilon_cylinder){
                        u = (p-p1)/norm(p-p1);
                        position[i] = p1 + (r+epsilon_cylinder)*u;
                        velocity[i] = v - dot(v,u)*u;
                    }
                    if (t>1.f && norm(p-p2)<r+epsilon_cylinder){
                        u = (p-p2)/norm(p-p2);
                        position[i] = p2 + (r+epsilon_cylinder)*u;
                        velocity[i] = v - dot(v,u)*u;
                    }

                }

            }

            //collision with ground
            if (p.y<y_min+epsilon_ground){

                position[i].y = y_min+epsilon_ground;
                velocity[i].y = 0.f;
            }

    }
}


bool detect_simulation_divergence(grid_2D<vec3> const& force, grid_2D<vec3> const& position)
{
    bool simulation_diverged = false;
    const size_t N = position.size();
    for(size_t k=0; simulation_diverged==false && k<N; ++k)
    {
        const float f = norm(force[k]);
        const vec3& p = position[k];

        if( std::isnan(f) ) // detect NaN in force
        {
            std::cout<<"NaN detected in forces"<<std::endl;
            simulation_diverged = true;
        }

        if( f>600.0f ) // detect strong force magnitude
        {
            std::cout<<" **** Warning : Strong force magnitude detected "<<f<<" at vertex "<<k<<" ****"<<std::endl;
            simulation_diverged = true;
        }

        if( std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) ) // detect NaN in position
        {
            std::cout<<"NaN detected in positions"<<std::endl;
            simulation_diverged = true;
        }
    }

    return simulation_diverged;

}
