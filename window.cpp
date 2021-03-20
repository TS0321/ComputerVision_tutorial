#include "window.hpp"


Window::~Window()
{
    // ウィンドウを破棄する
    glfwDestroyWindow(window);
}

// ウィンドウを閉じるべきかを判定する
int Window::shouldClose() const
{
    return glfwWindowShouldClose(window) || glfwGetKey(window, GLFW_KEY_ESCAPE);
}

// カラーバッファを入れ替えてイベントを取り出す
void Window::swapBuffers()
{
    // カラーバッファを入れ替える
    glfwSwapBuffers(window);

    // イベントを取り出す
    if (keyStatus == GLFW_RELEASE)
        glfwWaitEvents();
    else
        glfwPollEvents();

    // キーボードの状態を調べる
    if (glfwGetKey(window, GLFW_KEY_LEFT) != GLFW_RELEASE)
        location[0] -= 2.0f / size[0];
    else if (glfwGetKey(window, GLFW_KEY_RIGHT) != GLFW_RELEASE)
        location[0] += 2.0f / size[0];
    if (glfwGetKey(window, GLFW_KEY_DOWN) != GLFW_RELEASE)
        location[1] -= 2.0f / size[1];
    else if (glfwGetKey(window, GLFW_KEY_UP) != GLFW_RELEASE)
        location[1] += 2.0f / size[1];

    // マウスの左ボタンの状態を調べる
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_RELEASE)
    {
        // マウスの左ボタンが押されていたらマウスカーソルの位置を取得する
        double x, y;
        glfwGetCursorPos(window, &x, &y);

        // マウスカーソルの正規化デバイス座標系上での位置を求める
        location[0] = static_cast<GLfloat>(x) * 2.0f / size[0] - 1.0f;
        location[1] = 1.0f - static_cast<GLfloat>(y) * 2.0f / size[1];
    }
}

// ウィンドウのサイズを取り出す
const GLfloat* Window::getSize() const { return size; }

// ワールド座標系に対するデバイス座標系の拡大率を取り出す
GLfloat Window::getScale() const { return scale; }

// 位置を取り出す
const GLfloat* Window::getLocation() const { return location; }

// ウィンドウのサイズ変更時の処理
void Window::resize(GLFWwindow* window, int width, int height)
{
    // ウィンドウ全体をビューポートにする
    glViewport(0, 0, width, height);

    // このインスタンスの this ポインタを得る
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // 開いたウィンドウのサイズを保存する
        instance->size[0] = static_cast<GLfloat>(width);
        instance->size[1] = static_cast<GLfloat>(height);
    }
}

// マウスホイール操作時の処理
void Window::wheel(GLFWwindow* window, double x, double y)
{
    // このインスタンスの this ポインタを得る
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // ワールド座標系に対するデバイス座標系の拡大率を更新する
        instance->scale += static_cast<GLfloat>(y);
    }
}

// キーボード操作時の処理
void Window::keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // このインスタンスの this ポインタを得る
    Window* const
        instance(static_cast<Window*>(glfwGetWindowUserPointer(window)));

    if (instance != NULL)
    {
        // キーの状態を保存する
        instance->keyStatus = action;
    }
}