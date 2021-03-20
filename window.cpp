#include "window.hpp"


Window::~Window()
{
    // �E�B���h�E��j������
    glfwDestroyWindow(window);
}

// �E�B���h�E�����ׂ����𔻒肷��
int Window::shouldClose() const
{
    return glfwWindowShouldClose(window) || glfwGetKey(window, GLFW_KEY_ESCAPE);
}

// �J���[�o�b�t�@�����ւ��ăC�x���g�����o��
void Window::swapBuffers()
{
    // �J���[�o�b�t�@�����ւ���
    glfwSwapBuffers(window);

    // �C�x���g�����o��
    if (keyStatus == GLFW_RELEASE)
        glfwWaitEvents();
    else
        glfwPollEvents();

    // �L�[�{�[�h�̏�Ԃ𒲂ׂ�
    if (glfwGetKey(window, GLFW_KEY_LEFT) != GLFW_RELEASE)
        location[0] -= 2.0f / size[0];
    else if (glfwGetKey(window, GLFW_KEY_RIGHT) != GLFW_RELEASE)
        location[0] += 2.0f / size[0];
    if (glfwGetKey(window, GLFW_KEY_DOWN) != GLFW_RELEASE)
        location[1] -= 2.0f / size[1];
    else if (glfwGetKey(window, GLFW_KEY_UP) != GLFW_RELEASE)
        location[1] += 2.0f / size[1];

    // �}�E�X�̍��{�^���̏�Ԃ𒲂ׂ�
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_RELEASE)
    {
        // �}�E�X�̍��{�^����������Ă�����}�E�X�J�[�\���̈ʒu���擾����
        double x, y;
        glfwGetCursorPos(window, &x, &y);

        // �}�E�X�J�[�\���̐��K���f�o�C�X���W�n��ł̈ʒu�����߂�
        location[0] = static_cast<GLfloat>(x) * 2.0f / size[0] - 1.0f;
        location[1] = 1.0f - static_cast<GLfloat>(y) * 2.0f / size[1];
    }
}

// �E�B���h�E�̃T�C�Y�����o��
const GLfloat* Window::getSize() const { return size; }

// ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦�����o��
GLfloat Window::getScale() const { return scale; }

// �ʒu�����o��
const GLfloat* Window::getLocation() const { return location; }

// �E�B���h�E�̃T�C�Y�ύX���̏���
void Window::resize(GLFWwindow* window, int width, int height)
{
    // �E�B���h�E�S�̂��r���[�|�[�g�ɂ���
    glViewport(0, 0, width, height);

    // ���̃C���X�^���X�� this �|�C���^�𓾂�
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // �J�����E�B���h�E�̃T�C�Y��ۑ�����
        instance->size[0] = static_cast<GLfloat>(width);
        instance->size[1] = static_cast<GLfloat>(height);
    }
}

// �}�E�X�z�C�[�����쎞�̏���
void Window::wheel(GLFWwindow* window, double x, double y)
{
    // ���̃C���X�^���X�� this �|�C���^�𓾂�
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // ���[���h���W�n�ɑ΂���f�o�C�X���W�n�̊g�嗦���X�V����
        instance->scale += static_cast<GLfloat>(y);
    }
}

// �L�[�{�[�h���쎞�̏���
void Window::keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // ���̃C���X�^���X�� this �|�C���^�𓾂�
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // �L�[�̏�Ԃ�ۑ�����
        instance->keyStatus = action;
    }
}