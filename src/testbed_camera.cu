#include <neural-graphics-primitives/common.h>
#include <neural-graphics-primitives/common_device.cuh>

#include <neural-graphics-primitives/testbed.h>


NGP_NAMESPACE_BEGIN

bool Testbed::init_camera() {
    if (cam_type == CameraType::NOCAM) {
        return false;
    }
    else if (cam_type == CameraType::USB) {
        // something about usb camera config

        return true;
    }
    else if (cam_type == CameraType::REALSENSE) {
        // something about realsense config
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

        return false;
    }
    else {
        return false;
    }
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

    // we can define thetas, radiuses, n_view_in_steps which are depenedent on n_steps
    std::vector<float> thetas(n_steps);
    std::vector<float> radiuses(n_steps);
    std::vector<int> n_view_in_steps(n_steps); // number of views in each step

    // calculate thetas and radiuses
    for (int i = 0; i < n_steps; i++) {
        thetas[i] = (M_PI / 2) * static_cast<float>(i) / n_steps;
        radiuses[i] = sin(thetas[i]);
    }

    float radius_sum = std::accumulate(radiuses.begin(), radiuses.end(), 0.0f);
    for (int i = 0; i < n_steps; i++) {
        n_view_in_steps[i] = std::round(n_points * radiuses[i] / radius_sum);
    }

    // generate points and rotation matrices
    for (int i = 0; i < n_steps; i++) {
        for (int j = 0; j < n_view_in_steps[i]; j++) {
            float phi = 2 * M_PI * static_cast<float>(j) / n_view_in_steps[i];
            vec3 tvec = theta_phi_to_xyz(thetas[i], phi, radius) + origin;

            // calculate orientation
            vec3 u_z = -1.0f * normalize(theta_phi_to_xyz(thetas[i], phi, 1));
            vec3 u_y = normalize(theta_phi_to_xyz(thetas[i] + M_PI / 2, phi, 1));
            vec3 u_x = normalize(cross(u_y, u_z));

            // rotation matrix
            mat3 rmat = inverse(mat3(u_x, u_y, u_z));

            // Convert rotation matrix to quaternion
            quat tmp_quat = quat_cast(rmat);

            mat4 tmp_mat = glm::translate(glm::toMat4(tmp_quat), tvec);
            
            // CandidateView tmp_cv{tvec, tmp_quat, tmp_mat};
            m_candidate_views.push_back({tvec, tmp_quat, tmp_mat});
        }
    }
}

// load cam, board, candidate view cfgs
void Testbed::load_mint_config(const fs::path& path) {    
    std::ifstream f(native_string(path));
    nlohmann::json data = nlohmann::json::parse(f);

    nlohmann::json cam_cfg = data["camera"];
    nlohmann::json board_cfg = data["board"];
    nlohmann::json can_cfg = data["candidate_view"];

    tlog::info() << "Start candidate handler initialize";
    candidate_handler.init(can_cfg);
    tlog::info() << "End candidate handler initialize";

    tlog::info() << "type value is " << cam_cfg["type"];
    CameraType cam_type = (CameraType) cam_cfg["type"];
    switch (cam_type) {
        case CameraType::USB:
            cam_stream = std::make_unique<Testbed::USBCamera>();
            break;
        case CameraType::REALSENSE:
            cam_stream = std::make_unique<Testbed::RealSense>();
            break;
    }

    cam_stream->init(cam_cfg);
}


void Testbed::SphereCandidates::init(nlohmann::json& j){
    radius = j["radius"];
    n_total_candidates = j["n_total_candidates"];
    n_floor = j["n_floor"];
    n_areas = j["n_areas"];

    tlog::info() << "radius " << radius;

}

void Testbed::SphereCandidates::gen_candidate_views() {
    // define theta phi to xyz function using lambda
    auto theta_phi_to_xyz = [](float theta, float phi, float radius) -> glm::vec3 {
        float x = radius * sin(theta) * cos(phi);
        float y = radius * sin(theta) * sin(phi);
        float z = radius * cos(theta);
        return glm::vec3(x, y, z);
    };

    // we can define thetas, radiuses, n_view_in_steps which are depenedent on n_steps
    std::vector<float> thetas(n_floor);
    std::vector<float> radiuses(n_floor);
    std::vector<int> n_view_in_steps(n_floor); // number of views in each step

    // calculate thetas and radiuses
    for (int i = 0; i < n_floor; i++) {
        thetas[i] = (M_PI / 2) * static_cast<float>(i) / n_floor;
        radiuses[i] = sin(thetas[i]);
    }

    float radius_sum = std::accumulate(radiuses.begin(), radiuses.end(), 0.0f);
    for (int i = 0; i < n_floor; i++) {
        n_view_in_steps[i] = std::round(n_total_candidates * radiuses[i] / radius_sum);
    }

    // generate points and rotation matrices
    for (int i = 0; i < n_floor; i++) {
        for (int j = 0; j < n_view_in_steps[i]; j++) {
            float phi = 2 * M_PI * static_cast<float>(j) / n_view_in_steps[i];
            vec3 tvec = theta_phi_to_xyz(thetas[i], phi, radius) + origin;

            // calculate orientation
            vec3 u_z = -1.0f * normalize(theta_phi_to_xyz(thetas[i], phi, 1));
            vec3 u_y = normalize(theta_phi_to_xyz(thetas[i] + M_PI / 2, phi, 1));
            vec3 u_x = normalize(cross(u_y, u_z));

            // rotation matrix
            mat3 rmat = inverse(mat3(u_x, u_y, u_z));

            // Convert rotation matrix to quaternion
            quat tmp_quat = quat_cast(rmat);

            // Make Transform matrix
            mat4 tmp_mat = glm::translate(glm::toMat4(tmp_quat), tvec);
            
            // CandidateView tmp_cv{tvec, tmp_quat, tmp_mat};
            v_candidate_views.push_back({tvec, tmp_quat, tmp_mat});
        }
    }
}


void Testbed::CameraStream::init(nlohmann::json& j) {
    // set cam type
    cam_type = (CameraType)j["type"];

    // set cam model
    model = j["model"];

    // set fps
    fps = j["fps"];

    // set resolution
    resolution.width = j["resolution"][0];
    resolution.height = j["resolution"][1];

    // set intrinsic
    intrinsic.at<double>(0) = j["intrinsic"][0];
    intrinsic.at<double>(1) = j["intrinsic"][1];
    intrinsic.at<double>(2) = j["intrinsic"][2];
    intrinsic.at<double>(3) = j["intrinsic"][3];

    // set cameraMatrix
    camera_matrix.at<double>(0,0) = j["intrinsic"][0];
    camera_matrix.at<double>(1,1) = j["intrinsic"][1];
    camera_matrix.at<double>(0,2) = j["intrinsic"][2];
    camera_matrix.at<double>(1,2) = j["intrinsic"][3];

    // set distort coefficient
    dist_coeff.at<double>(0) = j["dist_coeff"][0];
    dist_coeff.at<double>(1) = j["dist_coeff"][1];
    dist_coeff.at<double>(2) = j["dist_coeff"][2];
    dist_coeff.at<double>(3) = j["dist_coeff"][3];
    dist_coeff.at<double>(4) = j["dist_coeff"][4];    
}

void Testbed::USBCamera::init(nlohmann::json& j) {
    // init using camera stream
    CameraStream::init(j);

    // open usb cap
    device = j["device"];
    cap.open(device.c_str());
}

void Testbed::RealSense::init(nlohmann::json& j) {
    // init using camera stream
    CameraStream::init(j);

    cfg.enable_stream(RS2_STREAM_COLOR, resolution.width, resolution.height, RS2_FORMAT_BGR8, fps);
    pipe.start(cfg);
}

bool Testbed::USBCamera::get_color_image(cv::Mat& color_image){
    if (!cap.isOpened())
    {
        printf("Can't open the video");
        return false;
    }

    cap >> color_image;
    return true;
}

bool Testbed::RealSense::get_color_image(cv::Mat& color_image) {
    frames = pipe.wait_for_frames();
    color = frames.get_color_frame();
    
    color_image = cv::Mat(resolution, CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    return true; // there must be something bad case. make it clear and add false return case
}


NGP_NAMESPACE_END