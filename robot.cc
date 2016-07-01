#include <iostream>

#include "robot.hh"

std::vector<Light> Robot::lights(2);

float Box::size = 0.32;
float Robot::size = 0.2;

// constructor
  Robot::Robot( World& world, const float x, const float y, const float a ) : 
  body( NULL ),
  joint( NULL )
{
  lights[0].x = 2;
  lights[0].y = 2;
  lights[0].intensity = 0.5;

  lights[1].x = 5;
  lights[1].y = 5;
  lights[1].intensity = 0.3;

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  body = world.b2world->CreateBody(&bodyDef);
  bumper = world.b2world->CreateBody(&bodyDef);
  
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( size/2.0, size/2.0 );
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;    
  fixtureDef.density = 10;
  fixtureDef.friction = 1.0;
  body->CreateFixture(&fixtureDef);
  
  // bumper has same settings the body but different size
  dynamicBox.SetAsBox( size/10.0, size/2.0 );
  bumper->CreateFixture(&fixtureDef);

  
  b2PrismaticJointDef jointDef;
  
  jointDef.Initialize( body, 
		       bumper, 
		       body->GetWorldCenter(), 
		       b2Vec2( 1.0f, 0.0f )
		       ); 
  
  jointDef.lowerTranslation = 0;//-0.2;
  jointDef.upperTranslation = 0.04f;
  jointDef.enableLimit = true;
  jointDef.maxMotorForce = 0.8f;
  jointDef.motorSpeed = 1.0f;
  jointDef.localAnchorA.Set( size/2.0, 0); // on the nose
  jointDef.localAnchorB.Set( 0,0 );
  
  jointDef.enableMotor = true;
  //jointDef.collideConnected = true;
  
  joint = (b2PrismaticJoint*)world.b2world->CreateJoint( &jointDef );

  // place assembled robot in the world
  body->SetTransform( b2Vec2( x, y ), a );	
  bumper->SetTransform( body->GetWorldPoint( b2Vec2( size/2,0) ), a );	
}

float Robot::GetLightIntensity( void )
{
  b2Vec2 here = body->GetWorldCenter();
  
  // integrate brightness over all light sources
  float brightness = 0.0;
  for( std::vector<Light>::iterator it = Robot::lights.begin(); 
       it != Robot::lights.end(); 
       it++ )
    {
      const float distanceToLightSqrd = 
	pow( here.x - it->x, 2.0 ) + 
	pow( here.y - it->y, 2.0 );
      
      brightness += it->intensity / distanceToLightSqrd;
    }

  return brightness;
}

bool Robot::GetBumperPressed( void )
{
  return( joint->GetJointTranslation() < 0.01 );
}

// set body speed in body-local coordinate frame
void Robot::SetSpeed( float x, float y, float a )
{  
  body->SetLinearVelocity( body->GetWorldVector(b2Vec2( x, y )));
  body->SetAngularVelocity( a );
}
