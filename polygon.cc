#include "push.hh"
#include <limits>

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
    for (auto &vertex : vertices)
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

double Polygon::getArea()
{
    double area = 0;
    int i;
    for (i = 0; i < vertices.size()-1; ++i)
    {
        // Shoelace formula
        area += vertices[i].x * vertices[i+1].y - vertices[i+1].x * vertices[i].y; 
    }
    area += vertices[i].x * vertices[0].y - vertices[0].x * vertices[i].y;
    return fabs(area) / 2.0f; 
}

// x,y is the point
double Polygon::getDistFromPoint(double x, double y)
{
    // Calculate the distance from the point to the polygon
    // Loop over all line segments and take the min

    // We could do this in a lot fewer variables
    // but this algorithm is kind of arcane
    double A, B, C, D, dot, lenSq, check, xx, yy, x1, y1, x2, y2, dx, dy, dist;
    double minDistance = std::numeric_limits<double>::infinity();

    int length = vertices.size(); // save a function call each loop
    int iplus1;
    for (int i = 0; i < vertices.size(); ++i)
    {
        // This accounts for the fact that we want i+1 = 0 at the last i
        iplus1 = (i+1) % vertices.size();
        x1 = vertices[i].x;
        y1 = vertices[i].y;
        x2 = vertices[iplus1].x;
        y2 = vertices[iplus1].y;

        A = x - x1;
        B = y - y1;
        C = x2 - x1;
        D = y2 - y1;

        dot = A*C + B*D;
        lenSq = C*C + D*D;
        check = -1;

        if (lenSq != 0)
            check = dot / lenSq;

        if (check < 0) { // Closest to first point
            xx = x1;
            yy = y1;
        }
        else if (check > 1) { // Closest to seconds
            xx = x2;
            yy = y2;
        }
        else { // Closest to segment on the line
            xx = x1 + check * C;
            yy = y1 + check * D;
        }

        dx = x - xx;
        dy = y - yy;
        dist = sqrt(dx * dx + dy * dy);
        if(dist < minDistance)
        {
            minDistance = dist;
        }
    }
    return minDistance;
}
