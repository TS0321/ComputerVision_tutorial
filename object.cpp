#include "object.hpp"

Object::Object(GLint size, GLsizei vertexcount, const Vertex* vertex)
{
    // 頂点配列オブジェクト
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // 頂点バッファオブジェクト
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
        vertexcount * sizeof(Vertex), vertex, GL_STATIC_DRAW);

    // 結合されている頂点バッファオブジェクトを in 変数から参照できるようにする
    glVertexAttribPointer(0, size, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
}

// デストラクタ
Object::~Object()
{
    // 頂点配列オブジェクトを削除する
    glDeleteBuffers(1, &vao);

    // 頂点バッファオブジェクトを削除する
    glDeleteBuffers(1, &vbo);
}


// 頂点配列オブジェクトの結合
void Object::bind() const
{
    // 描画する頂点配列オブジェクトを指定する
    glBindVertexArray(vao);
}

