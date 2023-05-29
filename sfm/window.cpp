#include "window.hpp"

#include <iostream>

void load2D(const int w, const int h) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);		

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.5, viewport[2] - 0.5, viewport[3] - 0.5, -0.5, -1.0, 1.0);
	
	double mat[4 * 4] = { 0 };

	{
		const double shiftx = (viewport[2] - 1) * 0.5 - (w - 1) * 0.5;
		const double shifty = (viewport[3] - 1) * 0.5 - (h - 1) * 0.5;

		mat[0 * 4 + 0] = 1.0;
		mat[1 * 4 + 1] = 1.0;
		mat[2 * 4 + 2] = 1.0;
		mat[3 * 4 + 3] = 1.0;

		mat[3 * 4 + 0] = shiftx;
		mat[3 * 4 + 1] = shifty;

	}

	glMultMatrixd(mat);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void load3D(const int w, const int h, const double fx, const double fy, const double cx, const double cy, const double nearPlane, const double farPlane) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	double mat[4 * 4] = { 0 };
	
	{
		const double dx = (viewport[2] - 1) * 0.5 - ((w - 1) * 0.5 - cx);
		const double dy = (viewport[3] - 1) * 0.5 - ((h - 1) * 0.5 - cy);

		const double nx = nearPlane / fx;
		const double ny = nearPlane / fy;

		const double sw = (viewport[2] - 1);
		const double sh = (viewport[3] - 1);

		const double l = (-dx) * nx;
		const double r = (-dx + sw) * nx;
		const double t = (-dy) * ny;
		const double b = (-dy + sh) * ny;
		const double n = nearPlane;
		const double f = farPlane;

		mat[0 * 4 + 0] = 2 * n / (r - l);
		mat[1 * 4 + 1] = 2 * n / (t - b);

		mat[2 * 4 + 0] = -(r + l) / (r - l);
		mat[2 * 4 + 1] = -(t + b) / (t - b);
		mat[2 * 4 + 2] = +(f + n) / (f - n);
		mat[2 * 4 + 3] = 1.0;

		mat[3 * 4 + 2] = -2 * f * n / (f - n);
		mat[3 * 4 + 3] = 1.0;

		/*const double l = -cx;
		const double r = w - cx;
		const double t = cy;
		const double b = -(h - cy);
		const double n = nearPlane;
		const double f = farPlane;

		mat[0 * 4 + 0] = 2 * fx / (r - l);
		mat[1 * 4 + 1] = 2 * fy / (t - b);

		mat[2 * 4 + 0] = -(r + l) / (r - l);
		mat[2 * 4 + 1] = -(t + b) / (t - b);
		mat[2 * 4 + 2] = (f + n) / (f - n);
		mat[2 * 4 + 3] = 1.0;

		mat[3 * 4 + 2] = -(2 * f * n) / (f - n);*/

	}

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(mat);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void dispImg(const unsigned char* ptr, const int w, const int h, const int ch) {

	int format;
	switch (ch) {
	case 1: format = GL_LUMINANCE; break;
	case 3: format = GL_RGB; break;
	default: return;
	}

	unsigned int texid;
	{
		glGenTextures(1, &texid);

		glBindTexture(GL_TEXTURE_2D, texid);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, format, GL_UNSIGNED_BYTE, ptr);
	}

	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_TEXTURE_2D);

	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glBindTexture(GL_TEXTURE_2D, texid);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		glColor3d(1.0, 1.0, 1.0);
		glColorMask(1, 1, 1, 1);

		glBegin(GL_QUADS);
		glTexCoord2i(0, 0); glVertex2d(0 - 0.5, 0 - 0.5);
		glTexCoord2i(0, 1); glVertex2d(0 - 0.5, h - 0.5);
		glTexCoord2i(1, 1); glVertex2d(w - 0.5, h - 0.5);
		glTexCoord2i(1, 0); glVertex2d(w - 0.5, 0 - 0.5);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, 0);
	}
	glPopAttrib();

	glDeleteTextures(1, &texid);
}

void dispImg(cv::Mat& img, const int w, const int h, const int ch)
{
	int format;
	switch (ch) {
	case 1: format = GL_LUMINANCE; break;
	case 3: format = GL_BGR_EXT; break;
	default: return;
	}

	unsigned int texid;
	{
		glGenTextures(1, &texid);

		glBindTexture(GL_TEXTURE_2D, texid);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, format, GL_UNSIGNED_BYTE, img.data);
	}

	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_TEXTURE_2D);

	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glBindTexture(GL_TEXTURE_2D, texid);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		glColor3d(1.0, 1.0, 1.0);
		glColorMask(1, 1, 1, 1);

		glBegin(GL_QUADS);
		glTexCoord2i(0, 0); glVertex2d(0 - 0.5, 0 - 0.5);
		glTexCoord2i(0, 1); glVertex2d(0 - 0.5, h - 0.5);
		glTexCoord2i(1, 1); glVertex2d(w - 0.5, h - 0.5);
		glTexCoord2i(1, 0); glVertex2d(w - 0.5, 0 - 0.5);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, 0);
	}
	glPopAttrib();

	glDeleteTextures(1, &texid);
}