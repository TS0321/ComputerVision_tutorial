#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <chrono>
class Window {
public:
	virtual void mouseButton(int button, int action, int mods) {
	
	}
	virtual void mousePos(double x, double y) {

	}
	virtual void mouseScroll(double x, double y) {

	}
	virtual void keyFun(int key, int scancode, int action, int mods) {

	}
	virtual void display(GLFWwindow *window) {

	}

public:
	void execute(const char* name, const int width, const int height) {
		using namespace std;
		chrono::system_clock::time_point start, end;
		if (!glfwInit()) return;

		GLFWwindow* window = glfwCreateWindow(width, height, name, NULL, NULL);
		if (!window) {
			glfwTerminate();
			return;
		}
		glfwMakeContextCurrent(window);
		glfwWindowHint(GLFW_SAMPLES, 4);

		setCallback(window);
		glfwSwapInterval(1);
		glClearColor(1.0, 0.0, 0.0, 1.0);
		while (!glfwWindowShouldClose(window)) {
			if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
				glfwSetWindowShouldClose(window, GL_TRUE);
			}

			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			start = chrono::system_clock::now();

			display(window);
			end = chrono::system_clock::now();
			double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
			printf("time %lf[ms]\n", time);
			glfwSwapBuffers(window);
			glfwPollEvents();
		}

		glfwTerminate();
	}

protected:
	void _mouseButton(int button, int action, int mods) {
		mouseButton(button, action, mods);
	}
	void _mousePos(double x, double y) {
		mousePos(x, y);
	}
	void _mouseScroll(double x, double y) {
		mouseScroll(x, y);
	}
	void _keyFun(int key, int scancode, int action, int mods) {
		keyFun(key, scancode, action, mods);
	}

protected:
	void setCallback(GLFWwindow* window) {
		glfwSetWindowUserPointer(window, this);

		glfwSetMouseButtonCallback(window, mouseButtonCB);
		glfwSetCursorPosCallback(window, mousePosCB);
		glfwSetScrollCallback(window, mouseScrollCB);
		glfwSetKeyCallback(window, keyFunCB);
	}

	static Window* getThisPtr(GLFWwindow* window) {
		return static_cast<Window*>(glfwGetWindowUserPointer(window));
	}

	static void mouseButtonCB(GLFWwindow* window, int button, int action, int mods) {
		getThisPtr(window)->_mouseButton(button, action, mods);
	}

	static void mousePosCB(GLFWwindow* window, double x, double y) {
		getThisPtr(window)->_mousePos(x, y);
	}

	static void mouseScrollCB(GLFWwindow* window, double x, double y) {
		getThisPtr(window)->_mouseScroll(x, y);
	}

	static void keyFunCB(GLFWwindow* window, int key, int scancode, int action, int mods) {
		getThisPtr(window)->_keyFun(key, scancode, action, mods);
	}
};

void load2D(const int w, const int h);
void dispImg(const unsigned char* ptr, const int w, const int h, const int ch);
void dispImg(cv::Mat& img, const int w, const int h, const int ch);
void load3D(const int w, const int h, const double fx, const double fy, const double cx, const double cy, const double nearPlane = 1.0, const double farPlane = 1000.0);
