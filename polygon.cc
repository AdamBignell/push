#include "push.hh"
#include <limits>

Polygon::Polygon(double newCx, double newCy)
{
    cx = newCx;
    cy = newCy;
}

Polygon::Polygon(std::vector<Vertex> newV, double newCx, double newCy)
{
    cx = newCx;
    cy = newCy;
    auto it = std::next(newV.begin(), newV.size());
    std::move(newV.begin(), it, std::back_inserter(vertices));
}

void Polygon::addVertex(double x, double y)
{
    Vertex point(x, y);
    vertices.push_back(point);
}

void Polygon::translate(double dx, double dy, bool recenter)
{
    for (auto &vertex : vertices)
    {
        vertex.x += dx;
        vertex.y += dy;
    }
    if (recenter)
    {
        cx += dx;
        cy += dy;
    }
}

// [s]cale, (cx,cy) center we scale with respect to
void Polygon::scale(double s, double newCx, double newCy)
{
    for (auto &vertex : vertices)
    {
        vertex.x = (vertex.x - newCx)*s + newCx;
        vertex.y = (vertex.y - newCy)*s + newCy;
    }
}

// Default to user center
void Polygon::scale(double s)
{
    for (auto &vertex : vertices)
    {
        vertex.x = (vertex.x - cx)*s + cx;
        vertex.y = (vertex.y - cy)*s + cy;
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

// Used in centroid calculation
double Polygon::getSignedArea()
{
    double area = 0;
    int i;
    for (i = 0; i < vertices.size()-1; ++i)
    {
        // Shoelace formula
        area += vertices[i].x * vertices[i+1].y - vertices[i+1].x * vertices[i].y; 
    }
    area += vertices[i].x * vertices[0].y - vertices[0].x * vertices[i].y;
    return area / 2.0f; 
}

void Polygon::setOrigin(double newCx, double newCy)
{
    cx = newCx;
    cy = newCy;
}

Vertex Polygon::getCentroid()
{
    double A = getSignedArea();

    double Cx = 0;
    double Cy = 0;

    // Calculate x of centroid
    int i, j;
    for (i = 0; i < vertices.size()-1; ++i)
    {
        Cx += (vertices[i].x + vertices[i+1].x) * (vertices[i].x*vertices[i+1].y - vertices[i+1].x*vertices[i].y);
    }
    Cx += (vertices[i].x + vertices[0].x) * (vertices[i].x*vertices[0].y - vertices[0].x*vertices[i].y);
    Cx /= (6*A);

    // Calculate y of centroid
    for (j = 0; j < vertices.size()-1; ++j)
    {
        Cy += (vertices[j].y + vertices[j+1].y) * (vertices[j].x*vertices[j+1].y - vertices[j+1].x*vertices[j].y);
    }
    Cy += (vertices[j].y + vertices[0].y) * (vertices[j].x*vertices[0].y - vertices[0].x*vertices[j].y);
    Cy /= (6*A);

    Vertex centroid(Cx, Cy);
    return centroid;
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

double Polygon::getAvgDistFromPoint(double x, double y)
{
    double totalDist = 0;
    for (auto &vertex : vertices)
    {
        totalDist += sqrt((vertex.x - x)*(vertex.x - x) + (vertex.y - y)*(vertex.y - y));
    }
    return totalDist/vertices.size();
}

void Polygon::primeCorners()
{
    Vertex ab(0,0), cb(0,0);
    int next, prev;
    double dot, cross, alpha, angle, scale;
    std::vector<Vertex> vertCopy(vertices); // Need to use the unchanges state
    std::vector<Vertex> newVerts;

    double scales[vertCopy.size()];

    for (int i = 0; i < vertCopy.size(); ++i)
    {
        // Calculate angle
        // Points A, B, C => i-1, i, i+1
        next = (i+1) % vertCopy.size();
        prev = (i-1) % vertCopy.size();

        ab.x = vertCopy[i].x - vertCopy[prev].x;
        ab.y = vertCopy[i].y - vertCopy[prev].y;

        cb.x = vertCopy[i].x - vertCopy[next].x;
        cb.y = vertCopy[i].y - vertCopy[next].y;

        dot = (ab.x * cb.x + ab.y * cb.y); // dot product
        cross = (ab.x * cb.y - ab.y * cb.x); // cross product

        alpha = atan2(cross, dot);
        angle = fabs(floor(alpha * 180. / M_PI + 0.5));
        
        scale = 1/(angle/157.5); //  Gives f(90) = 2, f(180) = 1
        if (scale < 1)
            scale = 1;

        // Calculate new vertices as inbetween points
        Vertex aPrime((vertCopy[next].x + vertCopy[i].x) / 2, (vertCopy[next].y + vertCopy[i].y) / 2);
        // Note that if we let each vertex handle only the next, eventually we loop back to the beginning
        // Vertex cPrime((vertCopy[prev].x + vertCopy[i].x) / 2, (vertCopy[prev].y + vertCopy[i].y) / 2);

        newVerts.push_back(aPrime);
        // Save the scale. Note that we need to insert all
        // the new evrtices first otherwise we will be calculating
        // their positions using the wrong values
        scales[i] = scale;       
    }

    for (int i=0; i < newVerts.size(); ++i)
    {
        vertices.insert(vertices.begin() + (2*i + 1) % (vertices.size()), newVerts[i]);
        //vertices.insert(vertices.begin() + prev, cPrime);
    }

    for (int i=0; i < vertCopy.size(); ++i)
    {
        vertices[(2*i) + 1].x *= scales[i];
        vertices[(2*i) + 1].y *= scales[i];
    }
}