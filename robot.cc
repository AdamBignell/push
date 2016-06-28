#include <iostream>

#include "robot.hh"

// static members
const int Robot::PUSH = 40;
const int Robot::BACKUP = 5;
const int Robot::TURNMAX = 20;
const float Robot::maxspeedx = 0.5;
const float Robot::maxspeeda = M_PI/2.0;
const float Robot::size = 0.3;

// constructor
Robot::Robot( b2World& world, float x, float y, float a ) : 
  pushTime( PUSH ),
  backupTime( BACKUP ),
  turnTime( random() % TURNMAX ),
  state( (control_state_t)(random() % S_COUNT) ), // choose start state at random
  speedx( 0 ),
  speeda( 0 ),
  body( NULL )
{
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  body = world.CreateBody(&bodyDef);
  
  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox( size/2.0, size/2.0 );
  
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;    
  fixtureDef.density = 10;
  fixtureDef.friction = 2.0;
  body->CreateFixture(&fixtureDef);
  
  // place in the world
  body->SetTransform( b2Vec2( x, y ), a );	
  
  // any need for this?
  //bodies[0]->SetLinearDamping( 5.0 );
  //bodies[0]->SetAngularDamping( 10.0 );
}

// called once per GUI update
void Robot::Update()
{
  // implement the robot behaviour with a little state machine
  switch( state )
    {
    case S_PUSH: // push
      speedx = maxspeedx;
	speeda = 0;	    
	if( --pushTime < 1 )
	  {
	    state = S_BACKUP;
	    pushTime = PUSH;
	  }
	break;
	
    case S_BACKUP: // backup
      speedx = -maxspeedx;
      speeda = 0;	    
      if( --backupTime < 1 )
	{
	  state = S_TURN;
	  backupTime = BACKUP;
	}
      break;
      
    case S_TURN: // turn
      speedx = 0;
      speeda = maxspeeda;	    
      if( --turnTime < 1 )
	{
	  state = S_PUSH;
	  turnTime = random() % TURNMAX;
	}
      break;
    default:
      std::cout << "invalid control state: " << state << std::endl;
      exit(1);
    }
  
  // set body speed in body-local coordinate frame
  body->SetLinearVelocity( body->GetWorldVector(b2Vec2( speedx, 0 )));
  body->SetAngularVelocity( speeda );
}

