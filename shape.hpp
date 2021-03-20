#pragma once
#include <memory>

// �}�`�f�[�^
#include "object.hpp"

//
// �}�`�̕`��
//
class Shape
{
    // �}�`�f�[�^
    std::shared_ptr<const Object> object;

protected:

    // �`��Ɏg�����_�̐�
    const GLsizei vertexcount;

public:

    // �R���X�g���N�^
    //   size: ���_�̈ʒu�̎���
    //   vertexcount: ���_�̐�
    //   vertex: ���_�������i�[�����z��
    Shape(GLint size, GLsizei vertexcount, const Object::Vertex* vertex);

    // �`��
    void draw() const;

    // �`��̎��s
    virtual void execute() const;
};
