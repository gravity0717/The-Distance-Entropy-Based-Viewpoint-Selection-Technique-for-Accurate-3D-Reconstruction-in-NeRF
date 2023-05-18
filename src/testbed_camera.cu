#include <neural-graphics-primitives/common.h>
#include <neural-graphics-primitives/common_device.cuh>

#include <neural-graphics-primitives/testbed.h>

NGP_NAMESPACE_BEGIN

void Testbed::init_camera() {
    cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    intrinsics = pipe.get_active_profile()
                    .get_stream(RS2_STREAM_COLOR)
                    .as<rs2::video_stream_profile>()
                    .get_intrinsics(); 
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsics.fx;
    cameraMatrix.at<double>(1, 1) = intrinsics.fy;
    cameraMatrix.at<double>(0, 2) = intrinsics.ppx;
    cameraMatrix.at<double>(1, 2) = intrinsics.ppy;

}

void Testbed::get_color_image(){
    frames = pipe.wait_for_frames();
    color = frames.get_color_frame();
    colorImage = cv::Mat(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
}

void Testbed::set_aruco_board(){
    board = cv::aruco::GridBoard::create(5, 7, markerLength, gap, arucoDict);
}

void Testbed::get_aruco_pose(){
    outputImage = colorImage.clone();
    cv::aruco::detectMarkers(outputImage, arucoDict, markerCorners, markerIds);
    if(markerIds.size() > 0){
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Rodrigues(rvec, R);
        const double* dataPtr = R.ptr<double>(0);
        for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j){
                cameraPose[j][i] = dataPtr[i * 3 + j];
            }
        }
        dataPtr = tvec.ptr<double>(0);
        for(int i = 0; i < 3; ++i){
            cameraPose[3][i] = dataPtr[i];
        }
        isaruco = true;
    } else {
        isaruco = false;
    }
}

void Testbed::color_to_texture(){
    glGenTextures(1, &aruco_texture);
    glBindTexture(GL_TEXTURE_2D, aruco_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, outputImage.cols, outputImage.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, outputImage.ptr());
}

void Testbed::visualize_camera_pose(ImDrawList* list, const mat4& world2proj){
    ivec2 res{color_width, color_height};
    float aspect = float(res.x) / float(res.y);
    visualize_nerf_camera(list, world2proj, cameraPose, aspect, isaruco ? 0x48ff00ff : 0xfc050dff);
}


void Testbed::gen_candidate_views(){
    // define theta phi to xyz function using lambda
    auto theta_phi_to_xyz = [](float theta, float phi, float radius) -> glm::vec3 {
    float x = radius * sin(theta) * cos(phi);
    float y = radius * sin(theta) * sin(phi);
    float z = radius * cos(theta);
    return glm::vec3(x, y, z);
};

    vector<float> thetas(n_steps);
    vector<float> radiuses(n_steps);
    vector<int> n_view_in_steps(n_steps);

    // calculate thetas and radiuses
    for (int i = 0; i < num_steps; ++i) {
        thetas[i] = (M_PI / 2) * static_cast<float>(i) / num_steps;
        radiuses[i] = sin(thetas[i]);
    }

    float radius_sum = std::accumulate(radiuses.begin(), radiuses.end(), 0.0f);
    for (int i = 0; i < n_steps; ++i) {
        n_view_in_steps[i] = std::round(n_points * radiuses[i] / radius_sum);
    }

    // generate points and rotation matrices
    for (int i = 0; i < n_steps; ++i) {
        for (int j = 0; j < n_view[i]; ++j) {
            float phi = 2 * M_PI * static_cast<float>(j) / n_view[i];
            vec3 tvec = theta_phi_to_xyz(thetas[i], phi, radius) + origin;

            // calculate orientation
            vec3 u_z = -1.0f * normalize(theta_phi_to_xyz(thetas[i], phi, 1));
            vec3 u_y = normalize(theta_phi_to_xyz(thetas[i] + M_PI / 2, phi, 1));
            vec3 u_x = normalize(cross(u_y, u_z));

            // rotation matrix
            mat3 rmat = inverse(mat3(u_x, u_y, u_z));

            // Convert rotation matrix to quaternion
            quat tmp_quat = quat_cast(rmat);

            // Get angle and axis quaternion
            float angle = angle(tmp_quat);
            vec3 axis = axis(tmp_quat);

            // Rotation vector is angle * axis
            vec3 rvec = angle * axis;

            m_candidate_views.push_back({tvec, rvec});
        }
    }
}

NGP_NAMESPACE_END