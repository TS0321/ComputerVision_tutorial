#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class Window
{
    // ウィンドウの識別子
    GLFWwindow* const window;

    // ウィンドウのサイズ
    GLfloat size[2];

    // ワールド座標系に対するデバイス座標系の拡大率
    GLfloat scale;

    // 図形の正規化デバイス座標系上での位置
    GLfloat location[2];

    // キーボードの状態
    int keyStatus;

public:

    // コンストラクタ
    Window(int width = 640, int height = 480, const char* title = "Hello!")
        : window(glfwCreateWindow(width, height, title, NULL, NULL))
        , scale(100.0f), location{ 0, 0 }, keyStatus(GLFW_RELEASE)
    {
        if (window == NULL)
        {
            // ウィンドウが作成できなかった
            std::cerr << "Can't create GLFW window." << std::endl;
            exit(1);
        }

        // 現在のウィンドウを処理対象にする
        glfwMakeContextCurrent(window);

        // GLEW を初期化する
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK)
        {
            // GLEW の初期化に失敗した
            std::cerr << "Can't initialize GLEW" << std::endl;
            exit(1);
        }

        // 作成したウィンドウに対する設定
        glfwSwapInterval(1);

        // ウィンドウのサイズ変更時に呼び出す処理の登録
        glfwSetWindowSizeCallback(window, resize);

        // マウスホイール操作時に呼び出す処理の登録
        glfwSetScrollCallback(window, wheel);

        // キーボード操作時に呼び出す処理の登録
        glfwSetKeyCallback(window, keyboard);

        // このインスタンスの this ポインタを記録しておく
        glfwSetWindowUserPointer(window, this);

        // 開いたウィンドウの初期設定
        resize(window, width, height);
    }


    // デストラクタ
    virtual ~Window();

    // ウィンドウを閉じるべきかを判定する
    int shouldClose() const;

    // カラーバッファを入れ替えてイベントを取り出す
    void swapBuffers();

    // ウィンドウのサイズを取り出す
    const GLfloat* getSize() const;

    // ワールド座標系に対するデバイス座標系の拡大率を取り出す
    GLfloat getScale() const;

    // 位置を取り出す
    const GLfloat* getLocation() const;

    // ウィンドウのサイズ変更時の処理
    static void resize(GLFWwindow* window, int width, int height);

    // マウスホイール操作時の処理
    static void wheel(GLFWwindow* window, double x, double y);

    // キーボード操作時の処理
    static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

};
