#pragma once
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

GLboolean printShaderInfoLog(GLuint shader, const char* str);

GLboolean printProgramInfoLog(GLuint program);

GLuint createProgram(const char* vsrc, const char* fsrc);

bool readShaderSource(const char* name, std::vector<GLchar>& buffer);

GLuint loadProgram(const char* vert, const char* frag);