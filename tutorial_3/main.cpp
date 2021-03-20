#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "../loadimage.hpp"
#include "../CameraCalibration.hpp"
#include "../CalcPose.hpp"
#include "../window.hpp"
#include "../shader.hpp"
#include "../shape.hpp"
#include "../object.hpp"

static const int img_num = 10;

int main()
{
	//�B�e�f�[�^�̓ǂݍ���
	std::vector<cv::Mat> cap_imgs;
	std::string folder_name = "../Capdata";
	cap_imgs = loadImage(folder_name, img_num);
	
	// GLFW ������������
	if (glfwInit() == GL_FALSE)
	{
		// �������Ɏ��s����
		std::cerr << "Can't initialize GLFW" << std::endl;
		return 1;
	}

	// �v���O�����I�����̏�����o�^����
	atexit(glfwTerminate);

	// OpenGL Version 3.2 Core Profile ��I������
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// �E�B���h�E���쐬����
	Window window(1280, 720);

	// �w�i�F���w�肷��
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);


	return EXIT_SUCCESS;
}