#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class Window
{
    // �E�B���h�E�̎��ʎq
    GLFWwindow* const window;

    // �E�B���h�E�̃T�C�Y
    GLfloat size[2];

    // ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦
    GLfloat scale;

    // �}�`�̐��K���f�o�C�X���W�n��ł̈ʒu
    GLfloat location[2];

    // �L�[�{�[�h�̏��
    int keyStatus;

public:

    // �R���X�g���N�^
    Window(int width = 640, int height = 480, const char* title = "Hello!")
        : window(glfwCreateWindow(width, height, title, NULL, NULL))
        , scale(100.0f), location{ 0, 0 }, keyStatus(GLFW_RELEASE)
    {
        if (window == NULL)
        {
            // �E�B���h�E���쐬�ł��Ȃ�����
            std::cerr << "Can't create GLFW window." << std::endl;
            exit(1);
        }

        // ���݂̃E�B���h�E�������Ώۂɂ���
        glfwMakeContextCurrent(window);

        // GLEW ������������
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK)
        {
            // GLEW �̏������Ɏ��s����
            std::cerr << "Can't initialize GLEW" << std::endl;
            exit(1);
        }

        // �쐬�����E�B���h�E�ɑ΂���ݒ�
        glfwSwapInterval(1);

        // �E�B���h�E�̃T�C�Y�ύX���ɌĂяo�������̓o�^
        glfwSetWindowSizeCallback(window, resize);

        // �}�E�X�z�C�[�����쎞�ɌĂяo�������̓o�^
        glfwSetScrollCallback(window, wheel);

        // �L�[�{�[�h���쎞�ɌĂяo�������̓o�^
        glfwSetKeyCallback(window, keyboard);

        // ���̃C���X�^���X�� this �|�C���^���L�^���Ă���
        glfwSetWindowUserPointer(window, this);

        // �J�����E�B���h�E�̏����ݒ�
        resize(window, width, height);
    }


    // �f�X�g���N�^
    virtual ~Window();

    // �E�B���h�E�����ׂ����𔻒肷��
    int shouldClose() const;

    // �J���[�o�b�t�@�����ւ��ăC�x���g�����o��
    void swapBuffers();

    // �E�B���h�E�̃T�C�Y�����o��
    const GLfloat* getSize() const;

    // ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦�����o��
    GLfloat getScale() const;

    // �ʒu�����o��
    const GLfloat* getLocation() const;

    // �E�B���h�E�̃T�C�Y�ύX���̏���
    static void resize(GLFWwindow* window, int width, int height);

    // �}�E�X�z�C�[�����쎞�̏���
    static void wheel(GLFWwindow* window, double x, double y);

    // �L�[�{�[�h���쎞�̏���
    static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

};
