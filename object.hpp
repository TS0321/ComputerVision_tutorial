#pragma once
#include <array>
#include <GL/glew.h>

//
// �}�`�f�[�^
//
class Object
{
    // ���_�z��I�u�W�F�N�g��
    GLuint vao;

    // ���_�o�b�t�@�I�u�W�F�N�g��
    GLuint vbo;

public:

    // ���_����
    struct Vertex
    {
        // �ʒu
        GLfloat position[2];
    };

    // �R���X�g���N�^
    //   size: ���_�̈ʒu�̎���
    //   vertexcount: ���_�̐�
    //   vertex: ���_�������i�[�����z��
    Object(GLint size, GLsizei vertexcount, const Vertex* vertex);

    // �f�X�g���N�^
    virtual ~Object();

private:

    // �R�s�[�R���X�g���N�^�ɂ��R�s�[�֎~
    Object(const Object& o);

    // ����ɂ��R�s�[�֎~
    Object& operator=(const Object& o);

public:

    // ���_�z��I�u�W�F�N�g�̌���
    void bind() const;
};
