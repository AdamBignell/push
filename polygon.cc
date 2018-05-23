#include "push.hh"

Polygon::Polygon(std::vector<Vertex> newV)
{
    auto it = std::next(newV.begin(), newV.size());
    std::move(newV.begin(), it, std::back_inserter(vertices));
}

void Polygon::addVertex(double x, double y)
{
    Vertex point(x, y);
    vertices.push_back(point);
}

void Polygon::translate(double dx, double dy)
{
    for (auto vertex : vertices)
    {
        vertex.x += dx;
        vertex.y += dy;
    }
}

// [s]cale, (cx,cy) center we scale with respect to
void Polygon::scale(double s, double cx, double cy)
{
    for (auto vertex : vertices)
    {
        vertex.x = (vertex.x - cx)*s + cx;
        vertex.y = (vertex.y - cy)*s+ cy;
    }
}

void Polygon::scale(double s)
{
    for (auto vertex : vertices)
    {
        vertex.x *= s;
        vertex.y *= s;
    }
}
