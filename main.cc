
#include <getopt.h>
#include <stdio.h>
#include <unistd.h> // for usleep(3)

#include <iostream>
#include <math.h>
#include <vector>

#include "push.hh"
#include <sstream>
#include <string>
#include <fstream>
#include <sstream>

template <class T>
int sign(T x)
{
  return (x > 0) - (x < 0);
}

double letterL[] = {10, 6, 10, 7, 10, 8, 10, 9, 10, 10, 11, 10, 12, 10, 13, 10, 14, 10};

// Utilities here
// If there are more than a few functions here
// Break out into a separate file
double dist(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

class Pusher : public Robot
{
private:
  typedef enum
  {
    S_PUSH = 0,
    S_BACKUP,
    S_TURN,
    S_COUNT,
    S_ESCAPE
  } control_state_t;

  static const double PUSH, BACKUP, TURNMAX;
  static const double SPEEDX, SPEEDA;
  static const double maxspeedx, maxspeeda;

  double timeleft;
  control_state_t state;
  double speedx, speeda;

  double lastintensity;
  double latch;

  int count;
  bool escape;

  int phase;

public:
  // constructor
  Pusher(World &world,
         robot_shape_t shape,
         double size,
         double x,
         double y,
         double a)
      : Robot(world,
              x, y, a,
              shape,
              size,
              5,    // drive gain
              20,   // turn gain
              0,    // charge start
              20,   // charge max
              0.4), // input efficiency
                    //0.1,
                    //0), // stay charged forever
        state(S_PUSH),
        timeleft(drand48() * TURNMAX),
        speedx(0),
        speeda(0),
        lastintensity(0),
        latch(0),
        count(drand48() * 1000.0),
        escape(false),
        phase(random() % 50)
  {
  }

  virtual void Update(double timestep)
  {
    if (world.steps % 50 == phase)
    {
      const double fleft = GetLightIntensityAt(+0.1, -0.1);
      const double fright = GetLightIntensityAt(+0.1, +0.1);

      const double bleft = GetLightIntensityAt(-0.1, -0.1);
      const double bright = GetLightIntensityAt(-0.1, +0.1);

      speedx = drive_gain * ((fright + fleft) - (bright + bleft));
      speeda = turn_gain * (fright - fleft);

      SetSpeed(speedx, 0, speeda);
    }

    Robot::Update(timestep); // inherit underlying behaviour to handle charge/discharge
  }
}; // class Pusher

// static members
const double Pusher::PUSH = 15.0; // seconds
const double Pusher::BACKUP = 0.5;
const double Pusher::TURNMAX = 2;
const double Pusher::SPEEDX = 0.5;
const double Pusher::SPEEDA = M_PI / 2.0;
const double Pusher::maxspeedx = 0.5;
const double Pusher::maxspeeda = M_PI / 2.0;

int main(int argc, char *argv[])
{
  double WIDTH = 32;
  double HEIGHT = 32;
  size_t ROBOTS = 128;
  size_t BOXES = 512;
  size_t LIGHTS = 32 * 32;
  double timeStep = 1.0 / 30.0;
  double robot_size = 0.35;
  double box_size = 0.25;
  Pusher::robot_shape_t robot_type = Pusher::SHAPE_RECT;
  Box::box_shape_t box_type = Box::SHAPE_RECT;
  int GUITIME = 1;

  // This is the file holding the polygon vertices
  // and the output file of the execution
  std::string pFileName = "";
  std::string outputFileName = "";
  std::string inputFileName = "";

  /* options descriptor */
  static struct option longopts[] = {
      {"robots", required_argument, NULL, 'r'},
      {"boxes", required_argument, NULL, 'b'},
      {"robotsize", required_argument, NULL, 'z'},
      {"boxsize", required_argument, NULL, 's'},
      {"robottype", required_argument, NULL, 't'},
      {"boxtype", required_argument, NULL, 'y'},
      {"guitime", required_argument, NULL, 'g'},
      {"outputfile", required_argument, NULL, 'o'},
      {"inputfile", required_argument, NULL, 'i'},
      //  { "help",  optional_argument,   NULL,  'h' },
      {NULL, 0, NULL, 0}};

  int ch = 0, optindex = 0;
  char firstChar;
  char optArgProxy[50];
  std::vector<std::string> tokens;
  while ((ch = getopt_long(argc, argv, "w:h:r:b:z:s:t:y:p:g:o:i:", longopts, &optindex)) != -1 || optindex < tokens.size())
  {
    if (argv)
      strcpy(optArgProxy, optarg);
    else 
    {
      // Once we get the input file, switch over to the options from there
      ch = tokens[optindex][1];
      strcpy(optArgProxy, tokens[optindex+1].c_str());
      optindex += 2;
    }
    switch (ch)
    {
    case 0: // long option given
      printf("option %s given", longopts[optindex].name);
      if (optArgProxy)
        printf(" with arg %s", optArgProxy);
      printf("\n");
      break;
    case 'w':
      WIDTH = atof(optArgProxy);
      break;
    case 'h':
      HEIGHT = atof(optArgProxy);
      break;
    case 'r':
      ROBOTS = atoi(optArgProxy);
      break;
    case 'b':
      BOXES = atoi(optArgProxy);
      break;
    case 'z':
      robot_size = atof(optArgProxy);
      break;
    case 's':
      box_size = atof(optArgProxy);
      break;
    case 't':
      firstChar = optArgProxy[0];
      if (firstChar == 'C' || firstChar == 'c')
        robot_type = Pusher::SHAPE_CIRC;
      else if (firstChar == 'R' || firstChar == 'r')
        robot_type = Pusher::SHAPE_RECT;
      else
        printf("unhandled robot shape %c\n", firstChar);
      break;
    case 'y':
      firstChar = optArgProxy[0];
      if (firstChar == 'H' || firstChar == 'h')
        box_type = Box::SHAPE_HEX;
      else if (firstChar == 'C' || firstChar == 'c')
        box_type = Box::SHAPE_CIRC;
      else if (firstChar == 'R' || firstChar == 'r')
        box_type = Box::SHAPE_RECT;
      else
        printf("unhandled box shape %c\n", firstChar);
      break;
    case 'p':
      pFileName = optArgProxy;
      break;
      // case 'h':
      // case '?':
      //   puts( USAGE );
      //   exit(0);
      //   break;
    case 'g':
      GUITIME = atoi(optArgProxy);
      break;
    case 'o':
      outputFileName = optArgProxy;
      break;
    case 'i':
    { // necessary since we initialize
      // As soon as we have an input file
      // we begin using the options from there
      inputFileName = optArgProxy;
      std::string headerStr;
      std::ifstream file(inputFileName);
      getline(file, headerStr);
      if (headerStr == "HEADER:")
        getline(file, headerStr);
      else
        break;
      std::stringstream ss(headerStr);
      std::string item;
      while (getline(ss, item, ' ')) {
        tokens.push_back(item);
      }
      argv = NULL;
      argc = 0;
      optindex = 0;
    }
      break;
    default:
      printf("unhandled option %c\n", ch);
      //puts( USAGE );
      exit(0);
    }
  }

  GuiWorld world(WIDTH, HEIGHT, GUITIME, LIGHTS);

  // Create objects
  for (int i = 0; i < BOXES; i++)
    world.AddBox(new Box(world, box_type, box_size,
                         WIDTH / 4.0 + drand48() * WIDTH * 0.5,
                         HEIGHT / 4.0 + drand48() * HEIGHT * 0.5,
                         drand48() * M_PI));

  for (int i = 0; i < ROBOTS; i++)
  {
    double x = WIDTH / 2.0;
    double y = HEIGHT / 2.0;

    while (x > WIDTH * 0.2 && x < WIDTH * 0.8 && y > HEIGHT * 0.2 && y < HEIGHT * 0.8)
    {
      x = drand48() * WIDTH;
      y = drand48() * HEIGHT;
    }

    world.AddRobot(new Pusher(world, robot_type, robot_size, x, y, drand48() * M_PI));
  }

  // fill the world with a grid of lights, all off
  // (width, height, height above arena, brightness)
  world.AddLightGrid(sqrt(LIGHTS), sqrt(LIGHTS), 2.0, 0.0);

  // This is used in both while loops to
  // display the world states more cleanly
  int updateRate = 100;
  bool running = true;

  // If we have an input file we don't need to calculate states
  // The while here becomes the whole main loop
  if (inputFileName != "")
  {
    // It doesn't make sense to skip frames here
    world.draw_interval = 1;
    std::ifstream file(inputFileName);
    while (!world.RequestShutdown() && running)
    {
      if (world.steps % updateRate == 1) // every now and again
      {
        running = world.loadNextState(file);
      }
      world.Step(timeStep);
      world.paused = true;
    }
    return 0; // Finished reading the file, close
  }

  if (outputFileName != "")
    world.saveWorldHeader(outputFileName);

  // Read the polygon from the input file if we have one
  if (pFileName != "")
  {
    std::ifstream infile(pFileName);
    if (!world.loadPolygonFromFile(infile))
    {
      printf("The input file was invalid or did not define a polygon\n.");
      exit(0);
    }
  }

  // These lines translate the polygon such that its 
  // centroid is at the origin (0,0)
  if (world.havePolygon)
  {
    Vertex centroid = world.polygon.getCentroid();
    world.polygon.translate(-1*centroid.x, -1*centroid.y, false);

    // Move the polygon into the arena's coordinate system, with (0,0) in the bottom left
    world.polygon.translate((WIDTH-1)/2.0, (HEIGHT-1)/2.0, true);
  }


  // Various declarations for main loop
  
  double delta = 0.6;
  double sdelta = 0.95;
  double xdelta = 0;
  double ydelta = 0;

  double lside = sqrt(LIGHTS);
  double lx = WIDTH / lside; // distance between lights
  double ly = HEIGHT / lside;

  // The thickness of the contracting pattern
  // No real intelligence here, but wider bands are a little more unwieldy
  double PATTWIDTH = fmax(lx,ly)/2; 

  // Note that we don't want the center of the
  // ring perimeter to actually hit the wall.
  double RADMAX = (WIDTH / 2.0);

  // Default is 5
  // We should set the RAD-Min intelligently
  // by matching the desired area (implicitly defined)
  double boxArea;
  if (box_type == Box::SHAPE_RECT)
    boxArea = box_size*box_size;
  else if (box_type == Box::SHAPE_CIRC)
    boxArea = M_PI*((box_size/2)*(box_size/2));
  else // box_type = HEX
  {
    double perimeter = 6*(box_size/2);
    double apothem = sqrt((box_size/2)*(box_size/2) - (box_size/4)*(box_size/4));
    boxArea = (0.5)*apothem*perimeter;
  }

  uint64_t maxsteps = 100000L;

  // This is the center of the contracting shape
  double goalx = WIDTH / 2.0;
  double goaly = HEIGHT / 2.0;

  // We need to adjust the user polygon to fit the arena
  // I'm a little uncomfortable switching from dilating to
  // contracting using radius. It's more 
  // intuitive to use area directly, but doing so would require
  // many more calculations. Keep in mind there are alternatives.
  double radius = RADMAX;
  if (world.havePolygon)
  {
    RADMAX = world.GetSetRadMax(world.polygon);
    radius = world.polygon.getDistFromPoint(goalx, goaly);
  }

  double RADMIN = world.GetRadMin(BOXES, boxArea, robot_size, world.polygon);

  // Can stop the holding behaviour by setting this to false
  // holdFor is set automatically below; it should be 0 here to begin
  bool holdAtMin = true;
  int holdFor = 0;

  /* Loop until the user closes the window */
  // Note that for irregular polygons we define the radius as the shortest distance
  // to any point on the polygon
  int writeState = GUITIME;
  while (!world.RequestShutdown() && world.steps < maxsteps)
  {
    if (world.steps % updateRate == 1) // every now and again
    {
      if (holdFor != 0 && holdAtMin)
      {
        holdFor--;
        world.UpdateLightPattern(goalx, goaly, 1, radius, PATTWIDTH);
        if (holdFor == 0)
        {
          if (world.havePolygon)
          {
            world.polygon.scale(sdelta);
            while(radius < RADMIN)
              radius *= sdelta; // Get us above the threshold to grow
          }
          else
            radius += delta;
        }
      }
      else
      {
        if (radius < RADMIN)
        {
          delta = -delta; // * 2.0;
          sdelta = 2-sdelta;

          //xdelta = 0.1;
          if (holdAtMin)
            // Trial and error: this is a decent heuristic
            holdFor = 1000/updateRate;
          continue;
        }

        else if (radius > RADMAX)
        {
          delta = -delta; //downdelta;
          sdelta = 2-sdelta;
        }

        // This shouldn't be an else despite the above
        if (radius <= RADMAX)
        {
          // Turns all necessary lights on for a specific amount of contraction (radius)
          // The polygon will automatically be used if it is well defined
          world.UpdateLightPattern(goalx, goaly, 1, radius, PATTWIDTH);
        }
#if 0
              for( int i=0; i<18; i+=2 )
                {
                  size_t index = letterL[i] + letterL[i+1] * lside;      
                  world.SetLightIntensity( index, 1 );       
                }
#endif

        // This handles both contractions and dilation
        if (holdFor == 0) // If we aren't staying contracted
        {
          if (world.havePolygon)
          {
            world.polygon.scale(sdelta);
            radius *= sdelta;
          }
          else
            radius += delta;
        }

        // Optionally move the collected resources
        // goalx += xdelta;
        // goaly += ydelta;
      }
    }

    if (--writeState == 0)
    {
      world.appendWorldStateToFile(outputFileName);
      writeState = GUITIME;
    }

    world.Step(timeStep);
  }

  printf("Completed %lu steps.", world.steps);

  return 0;
}
