#include <iostream>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <numeric>


// Convert theta and phi to Cartesian coordinates
glm::vec3 theta_phi_to_xyz(float theta, float phi, float radius) {
    float x = radius * sin(theta) * cos(phi);
    float y = radius * sin(theta) * sin(phi);
    float z = radius * cos(theta);
    return glm::vec3(x, y, z);
}

std::vector<glm::mat4> sphere_gen(float radius, int num_points, int num_steps, glm::vec3 origin = glm::vec3(0.0f, 0.0f, 0.0f)) {
    std::vector<glm::mat4> transforms;
    std::vector<float> thetas(num_steps);
    std::vector<float> radiuses(num_steps);
    std::vector<int> n_view(num_steps);

    // set points num in each height as n_view
    for (int i = 0; i < num_steps; ++i) {
        thetas[i] = (M_PI / 2) * static_cast<float>(i) / num_steps;
        radiuses[i] = sin(thetas[i]);
    }

    float radius_sum = std::accumulate(radiuses.begin(), radiuses.end(), 0.0f);
    for (int i = 0; i < num_steps; ++i) {
        n_view[i] = std::round(num_points * radiuses[i] / radius_sum);
    }

    // generate points and rotation matrices
    for (int i = 0; i < num_steps; ++i) {
        for (int j = 0; j < n_view[i]; ++j) {
            float phi = 2 * M_PI * static_cast<float>(j) / n_view[i];
            glm::vec3 pts = theta_phi_to_xyz(thetas[i], phi, radius) + origin;

            // calculate orientation
            glm::vec3 u_z = -1.0f * glm::normalize(theta_phi_to_xyz(thetas[i], phi, 1));
            glm::vec3 u_y = glm::normalize(theta_phi_to_xyz(thetas[i] + M_PI / 2, phi, 1));
            glm::vec3 u_x = glm::normalize(glm::cross(u_y, u_z));

            // rotation matrix
            glm::mat3 rmat = glm::inverse(glm::mat3(u_x, u_y, u_z));

            // create transform matrix
            glm::mat4 transform = glm::mat4(1.0f);
            transform[3] = glm::vec4(pts, 1.0f);
            for (int m = 0; m < 3; ++m)
                for (int n = 0; n < 3; ++n)
                    transform[m][n] = rmat[m][n];

            transforms.push_back(transform);
        }
    }

    return transforms;
}

int main() {
    float radius = 1.5f;
    int num_points = 100;
    int num_steps = 5;
    glm::vec3 origin(0.0f, 0.0f, 0.0f);

    std::vector<glm::mat4> transforms = sphere_gen(radius, num_points, num_steps, origin);

    std::cout << "Generated points: \n";
    for (auto& transform : transforms) {
        std::cout << glm::to_string(transform) << "\n";
    }

    return 0;
}
