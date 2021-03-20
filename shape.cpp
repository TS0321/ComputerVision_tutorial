#include "shape.hpp"

Shape::Shape(GLint size, GLsizei vertexcount, const Object::Vertex* vertex)
    : object(new Object(size, vertexcount, vertex))
    , vertexcount(vertexcount)
{
}

// 描画
void Shape::draw() const
{
    // 頂点配列オブジェクトを結合する
    object->bind();

    // 描画を実行する
    execute();
}

// 描画の実行
void Shape::execute() const
{
    // 折れ線で描画する
    glDrawArrays(GL_LINE_LOOP, 0, vertexcount);
}