#pragma once
#include <memory>

// 図形データ
#include "object.hpp"

//
// 図形の描画
//
class Shape
{
    // 図形データ
    std::shared_ptr<const Object> object;

protected:

    // 描画に使う頂点の数
    const GLsizei vertexcount;

public:

    // コンストラクタ
    //   size: 頂点の位置の次元
    //   vertexcount: 頂点の数
    //   vertex: 頂点属性を格納した配列
    Shape(GLint size, GLsizei vertexcount, const Object::Vertex* vertex);

    // 描画
    void draw() const;

    // 描画の実行
    virtual void execute() const;
};
