#include "push.hh"

/** Normalize an angle to within +/_ M_PI. */
double AngleNormalize(double a)
{
  while (a < -M_PI)
    a += 2.0 * M_PI;
  while (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

// constructor
Robot::Robot(World &world,
             double x,
             double y,
             double a,
             double size,
             double drive_gain,
             double turn_gain,
             double charge,
             double charge_max,
             double input_efficiency,
             double output_metabolic,
             double output_efficiency) : world(world),
                                         size(size),
                                         drive_gain(drive_gain),
                                         turn_gain(turn_gain),
                                         charge(charge),
                                         charge_max(charge_max),
                                         charge_delta(0),
                                         input_efficiency(input_efficiency),
                                         output_metabolic(output_metabolic),
                                         output_efficiency(output_efficiency),
                                         body(NULL)
//bumper( NULL ),
//joint( NULL )
{
  memset(targets, 0, sizeof(targets));

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  body = world.b2world->CreateBody(&bodyDef);
  //bumper = world.b2world->CreateBody(&bodyDef);

  //Use square robots
  // b2PolygonShape botShape;
  // botShape.SetAsBox(size / 2.0, size / 2.0);

  //Use circular robots
  b2CircleShape botShape;
  botShape.m_p.Set(0, 0);
  botShape.m_radius = size/2.0; //use class variable

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &botShape;
  fixtureDef.density = 10;
  fixtureDef.friction = 1.0;

  // prevent collision with puck-retaining strings
  fixtureDef.filter.categoryBits = ROBOT;
  fixtureDef.filter.maskBits = ROBOT | BOX | ROBOTBOUNDARY; // not box boundary

  body->CreateFixture(&fixtureDef);

  // place assembled robot in the world
  body->SetTransform(b2Vec2(x, y), a);
}

double Robot::GetLightIntensity(void) const
{
  const b2Vec2 here = body->GetWorldCenter();
  return world.GetLightIntensityAt(here.x, here.y);
}

double Robot::GetLightIntensityAt(double x, double y) const
{
  b2Vec2 here = body->GetWorldPoint(b2Vec2(x, y));
  return world.GetLightIntensityAt(here.x, here.y);
}

// void Robot::GetNeighbors( double pixels[8] )
// {
//   // measure the distance to the nearest robot in each direction
//   // TODO: do this faster

//   double MAXRANGE = 1.0;

//   const double rad_per_pixel = M_PI*2.0 / 8.0;

//   for( int i=0; i<8; ++i )
//     pixels[i] = MAXRANGE;

//   return;

//   const b2Vec2 mypose = body->GetPosition();
//   const double angle = body->GetAngle();

//   // check every robot in the world to see if it is detected
//   for( auto& other : world.robots )
//     {
//       // discard if it's the same robot
//       if( other == this )
//   continue;

//       // discard if it's out of range. We put off computing the
//       // hypotenuse as long as we can, as it's relatively expensive.

//       const b2Vec2 hispose = other->body->GetPosition();
//       const  double dx =  hispose.x - mypose.x;
//       const double dy = hispose.y - mypose.y;
//       const int32_t range = hypot( dx, dy );

//       if( range > MAXRANGE )
//   continue;

//       // discard if it's out of field of view
//       const double absolute_heading = atan2( dy, dx );
//       const double relative_heading = AngleNormalize(absolute_heading - angle) + M_PI;
//       const uint32_t pixel = floor( relative_heading / rad_per_pixel );

//       //printf( "relative heading %.3f rpp %.4f  pixel %u\n",
//       //      relative_heading, rad_per_pixel, pixel );

//       assert( pixel >= 0 );
//       assert( pixel < 8 );

//       // discard if we've seen something closer in this pixel already.
//       if( pixels[pixel] < range)
//   continue;

//       // if we made it here, we see this other robot in this pixel.
//       pixels[pixel] = range;
//     }
// }

// double* Robot::GetTargets( void )
// {
//   return targets;
// }

// void Robot::UpdateTargetSensor()
// {
//   // measure the distance to the nearest robot in each direction
//   // TODO: do this faster

//   double MAXRANGE = 3.0;

//   const double rad_per_pixel = M_PI*2.0 / 7.0;

//   for( int i=0; i<7; ++i )
//     targets[i] = MAXRANGE;

//   b2Vec2 mypose = body->GetPosition();
//   double angle = body->GetAngle();

//   // check every robot in the world to see if it is detected
//   for( auto& other : world.boxes )
//     {
//       // discard if it's out of range. We put off computing the
//       // hypotenuse as long as we can, as it's relatively expensive.

//       b2Vec2 hispose = other->body->GetPosition();

//       double dx =  hispose.x - mypose.x;

//       // if( fabs(dx) > MAXRANGE )
//       //   continue; // out of range

//       double dy = hispose.y - mypose.y;

//       // if( fabs(dy) > MAXRANGE )
//       //   continue; // out of range

//       // size of the box
//       // assert( other->body->m_shapeList->m_type == e_polyShape);

//       // todo - box size hack
//       double range = hypot( dx, dy ) - 0.15;
//       if( range > MAXRANGE )
//   continue;

//       // discard if it's out of field of view
//       double absolute_heading = atan2( dy, dx );
//       double relative_heading = AngleNormalize(absolute_heading - angle);

//       // find which pixel it falls in
//       relative_heading += M_PI;

//       const uint32_t pixel = floor( relative_heading / rad_per_pixel );

//       //printf( "relative heading %.3f rpp %.4f  pixel %u\n",
//       //      relative_heading, rad_per_pixel, pixel );

//       assert( pixel >= 0 );
//       assert( pixel < 7 );

//       // discard if we've seen something closer in this pixel already.
//       if( targets[pixel] < range)
//   continue;

//       // if we made it here, we see this other robot in this pixel.
//       targets[pixel] = range;
//     }
// }

// bool Robot::GetBumperPressed( void )
// {
//   return( joint->GetJointTranslation() < 0.01 );
// }

// set body speed in body-local coordinate frame
void Robot::SetSpeed(double x, double y, double a)
{
  body->SetLinearVelocity(body->GetWorldVector(b2Vec2(x, y)));
  body->SetAngularVelocity(a);
}

void Robot::Update(double timestep)
{
  //UpdateTargetSensor();

  // absorb energy from lights
  charge_delta = input_efficiency * GetLightIntensity(); // gather power from light

  // expend energy just living
  charge_delta -= output_metabolic;

  // expend energy by moving
  charge_delta -= output_efficiency * body->GetAngularVelocity();
  charge_delta -= output_efficiency * body->GetLinearVelocity().Length();

  charge += charge_delta;

  if (charge <= 0.0)
  {
    charge = 0.0;
    SetSpeed(0, 0, 0); // can't move
  }
  else if (charge > charge_max)
    charge = charge_max; // full up
}
