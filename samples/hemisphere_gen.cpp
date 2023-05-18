#include <iostream>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>

std::vector<glm::vec3> sphere_gen(float radius, int num_points, int num_steps, glm::vec3 origin = glm::vec3(0.0f, 0.0f, 0.0f)) {
    std::vector<float> radius_in_each_step(num_steps);
    for(int i = 0; i < num_steps; i++) {
        radius_in_each_step[i] = sin((M_PI / 2) * (static_cast<float>(i) / num_steps));
    }
    
    float total = 0;
    for(auto val : radius_in_each_step) {
        total += val;
    }

    std::vector<int> n_view_in_each_steps(num_steps);
    for(int i = 0; i < num_steps; i++) {
        n_view_in_each_steps[i] = round(num_points * (radius_in_each_step[i] / total));
    }
    
    std::vector<glm::vec3> points;
    for(int idx = 0; idx < num_steps; idx++) {
        for(int i = 0; i < n_view_in_each_steps[idx]; i++) {
            float theta = radius_in_each_step[idx];
            float phi = 2 * M_PI * i / n_view_in_each_steps[idx];
            float x = radius * sin(theta) * cos(phi) + origin.x;
            float y = radius * sin(theta) * sin(phi) + origin.y;
            float z = radius * cos(theta) + origin.z;
            points.push_back(glm::vec3(x, y, z));
        }
    }
    return points;
}



int main() {
    float radius = 1.5f;
    int num_points = 100;
    int num_steps = 5;
    glm::vec3 origin(0.0f, 0.0f, 0.0f);

    std::vector<glm::vec3> points = sphere_gen(radius, num_points, num_steps, origin);

    std::cout << "Generated points: \n";
    for (auto& point : points) {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    return 0;
}
