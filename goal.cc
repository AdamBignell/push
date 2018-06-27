#include <iostream>

#include "push.hh"

Goal::Goal(World *world, double x, double y, double size, goal_shape_t shape) : x(x), y(y), size(size), shape(shape), body(NULL)
{
  b2PolygonShape staticBox;
  b2CircleShape staticCircle;

  switch (shape)
  {
  case SHAPE_RECT:
    staticBox.SetAsBox(size / 2.0, size / 2.0);
    break;
  case SHAPE_HEX:
  {
    b2Vec2 verts[6];

    for (int i = 0; i < 6; i++)
    {
      verts[i].x = size / 2.0 * cos((2.0 * M_PI * i / 6.0) + M_PI/6.0);
      verts[i].y = size / 2.0 * sin((2.0 * M_PI * i / 6.0) + M_PI/6.0);
    }

    staticBox.Set(verts, 6);
    break;
  }
  case SHAPE_CIRC:
  {
    staticCircle.m_p.Set(0.0f, 0.0f);         //position, relative to body position
    staticCircle.m_radius = size / 2.0; //radius
    break;
  }
  default:
    std::cout << "invalid shape number " << shape << std::endl;
    break;
  }

  b2FixtureDef fixtureDef;
  if (shape != SHAPE_CIRC)
  {
    fixtureDef.shape = &staticBox;
  }
  else
  {
    fixtureDef.shape = &staticCircle;	
  }
  fixtureDef.density = 0;
  fixtureDef.friction = 0;
  fixtureDef.restitution = 0;

  fixtureDef.filter.categoryBits = GOAL;
  fixtureDef.filter.maskBits = 0; //everything
  // BOX | ROBOT | BOXBOUNDARY | ROBOTBOUNDARY; // everything

  b2BodyDef bodyDef;
  bodyDef.type = b2_staticBody;

  body = world->b2world->CreateBody(&bodyDef);
  body->SetTransform(b2Vec2(x, y), 0);

  body->CreateFixture(&fixtureDef);
}
