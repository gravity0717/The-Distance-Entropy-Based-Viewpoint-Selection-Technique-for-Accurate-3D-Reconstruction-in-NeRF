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

void Testbed::get_aruco_pose(){
    outputImage = colorImage.clone();
    cv::aruco::detectMarkers(outputImage, dictionary, markerCorners, markerIds);
    if(markerIds.size() > 0){
		cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength,  cameraMatrix, distCoeffs,
            rvecs, tvecs);
        
        cv::Rodrigues(rvecs[0], R);
        const double* dataPtr = R.ptr<double>(0);
        for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j){
                cameraPose[j][i] = dataPtr[i * 3 + j];
            }
        }
        for(int i = 0; i < 3; ++i){
            cameraPose[3][i] = tvecs[0][i];
        }
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
    visualize_nerf_camera(list, world2proj, cameraPose, aspect, 0x80ffffff);
}


NGP_NAMESPACE_END