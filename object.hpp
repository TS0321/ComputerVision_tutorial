#pragma once
#include <array>
#include <GL/glew.h>

//
// 図形データ
//
class Object
{
    // 頂点配列オブジェクト名
    GLuint vao;

    // 頂点バッファオブジェクト名
    GLuint vbo;

public:

    // 頂点属性
    struct Vertex
    {
        // 位置
        GLfloat position[2];
    };

    // コンストラクタ
    //   size: 頂点の位置の次元
    //   vertexcount: 頂点の数
    //   vertex: 頂点属性を格納した配列
    Object(GLint size, GLsizei vertexcount, const Vertex* vertex);

    // デストラクタ
    virtual ~Object();

private:

    // コピーコンストラクタによるコピー禁止
    Object(const Object& o);

    // 代入によるコピー禁止
    Object& operator=(const Object& o);

public:

    // 頂点配列オブジェクトの結合
    void bind() const;
};
