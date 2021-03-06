
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
  double WIDTH = 64;
  double HEIGHT = 64;
  size_t ROBOTS = 128;
  size_t BOXES = 512;
  size_t LIGHTS = WIDTH * HEIGHT;
  double timeStep = 1.0 / 30.0;
  double robot_size = 0.35;
  double box_size = 0.25;
  double flare = -1.0;
  double drag = 0;
  bool switchToCircle = false;
  Pusher::robot_shape_t robot_type = Pusher::SHAPE_RECT;
  Box::box_shape_t box_type = Box::SHAPE_RECT;
  int GUITIME = 1;
  bool useGui = true;

  // This is the file holding the polygon vertices
  // and the output file of the execution
  std::string pFileName = "";
  std::string outputFileName = "";
  std::string performanceFileName = "";
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
      {"flare", required_argument, NULL, 'f'},
      {"drag", required_argument, NULL, 'd'},
      {"circleswitch", required_argument, NULL, 'c'},
      //  { "help",  optional_argument,   NULL,  'h' },
      {NULL, 0, NULL, 0}};

  int ch = 0, optindex = 0;
  char firstChar;
  char optArgProxy[50];
  std::vector<std::string> tokens;
  // Catch the argument-less option
  for (int i = 0; i < argc ; ++i)
  {
    if (strcmp(argv[i], "-x") == 0)
    {
      useGui = false;
      //argv[i] = NULL;
      //
      if (i < argc-1)
      {
        if (argv[i+1][0] != '-')
        {
          fprintf(stderr, "The '-x' flag does not take an argument. Aborting...\n");
          return 0;
        }
      }
      argc--;
      // This allows us to accept the argumentless -x flag
      for (;i < argc;i++)
      {
        strcpy(argv[i], argv[i+1]);
      }
    }
  }
  // Parse all other options
  while ((ch = getopt_long(argc, argv, "w:h:r:b:z:s:t:y:p:g:o:i:f:d:c:", longopts, &optindex)) != -1 || optindex < tokens.size())
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
      outputFileName = "Results_Replays/" + std::string(optArgProxy) + ".txt";
      performanceFileName = "Results/" + std::string(optArgProxy) + "_PerfData.txt";
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
    case 'f':
    {
      flare = atof(optArgProxy);
      break;
    }
    case 'd':
    {
      drag = atof(optArgProxy);
      break;
    }
    case 'c':
    {
      double stoC = atof(optArgProxy);
      if (stoC > 0)
        switchToCircle = true;
      break;
    }
    default:
      printf("unhandled option %c\n", ch);
      //puts( USAGE );
      exit(0);
    }
  }

  // Reset the number of lights in case width and height changed
  LIGHTS = WIDTH * HEIGHT;

  World* world = NULL;
  double replayWorld = false;
  if (inputFileName != "")
    replayWorld = true;
  if (useGui)
  {
    world = new GuiWorld(WIDTH, HEIGHT, LIGHTS, GUITIME, flare, drag, switchToCircle, replayWorld);
  }
  else
  {
    world = new World(WIDTH, HEIGHT, LIGHTS, GUITIME, flare, drag, switchToCircle, replayWorld);
  }

  // Create objects
  // Zoomed In
  // for (int i = 0; i < BOXES; i++)
  //   world->AddBox(new Box(*world, box_type, box_size,
  //                        WIDTH / 4.0 + drand48() * WIDTH * 0.5,
  //                        HEIGHT / 4.0 + drand48() * HEIGHT * 0.5,
  //                        drand48() * M_PI));

  // Zoomed Out
  double ldx = sqrt(LIGHTS)/WIDTH/2.0;
  double ldy = sqrt(LIGHTS)/HEIGHT/2.0;
  for (int i = 0; i < BOXES; i++)
    world->AddBox(new Box(*world, box_type, box_size,
                         WIDTH * (3/8.0) + drand48() * WIDTH * 0.25 + ldx,
                         HEIGHT * (3/8.0) + drand48() * HEIGHT * 0.25 + ldy,
                         drand48() * M_PI));

  for (int i = 0; i < ROBOTS; i++)
  {
    double x = WIDTH / 2.0;
    double y = HEIGHT / 2.0;

    // Zoomed In
    // while (x > WIDTH * 0.2 && x < WIDTH * 0.8 && y > HEIGHT * 0.2 && y < HEIGHT * 0.8)
    // {
    //   x = drand48() * WIDTH;
    //   y = drand48() * HEIGHT;
    // }

    //Zoomed Out
    double lhBoxBound = (WIDTH * 3/8.0);
    double rhBoxBound= (WIDTH - lhBoxBound);
    double bottomBoxBound = (HEIGHT * 3/8.0);
    double topBoxBound = (HEIGHT - bottomBoxBound);
    while ((x >= lhBoxBound && x <= rhBoxBound && y >= bottomBoxBound && y <= topBoxBound))
    {
      x = drand48() * (WIDTH * 4/8.0) + (WIDTH * 2/8.0);
      y = drand48() * (HEIGHT * 4/8.0) + (HEIGHT * 2/8.0);
    }

    world->AddRobot(new Pusher(*world, robot_type, robot_size, x + ldx, y + ldy, drand48() * M_PI));
  }

  // fill the world with a grid of lights, all off
  // (width, height, height above arena, brightness)
  world->AddLightGrid(sqrt(LIGHTS), sqrt(LIGHTS), 2.0, 0.0);

  // This is used in both while loops to
  // display the world states more cleanly
  int updateRate = 100;
  bool running = true;
  double numCorrect = 0;
  int checkSuccess = 10;

  // Read the polygon from the input file if we have one
  if (pFileName != "")
  {
    std::ifstream infile(pFileName);
    if (!world->loadPolygonFromFile(infile))
    {
      printf("The input file was invalid or did not define a polygon\n.");
      exit(0);
    }
  }

  // This is the center of the contracting shape
  double goalx = ceil((WIDTH)/2.0);
  double goaly = ceil((HEIGHT)/2.0);

  // These lines prime the polygon
  if (world->havePolygon)
  {
    // Make the centroid the origin
    Vertex centroid = world->polygon->getCentroid();
    world->polygon->translate(-1*centroid.x, -1*centroid.y, false);

    // Move the polygon into the arena's coordinate system, with (0,0) in the bottom left
    world->polygon->translate(goalx, goaly, true);
  }


  // Various declarations for main loop
  double delta = 0.6;
  double sdelta = 0.975; // 'scale' delta. Multiplicative delta, not additive
  double xdelta = 0;
  double ydelta = 0;

  double lside = sqrt(LIGHTS);
  double lx = WIDTH / lside; // distance between lights
  double ly = HEIGHT / lside;

  // The thickness of the contracting pattern
  // No real intelligence here, but wider bands are a little more unwieldy
  double PATTWIDTH = fmax(lx,ly)/10; 

  // Note that we don't want the center of the
  // ring perimeter to actually hit the wall.
  double RADMAX = (WIDTH / 2.0) * (0.75);

  // Set RAD-Min by matching the desired area (implicitly defined)
  double boxArea;
  if (box_type == Box::SHAPE_RECT)
    boxArea = box_size*box_size;
  else if (box_type == Box::SHAPE_CIRC)
    boxArea = M_PI*((box_size/2)*(box_size/2));
  else // box_type = HEX
  {
    //double perimeter = 6*(box_size/2);
    double apothem = sqrt((box_size/2)*(box_size/2) - (box_size/4)*(box_size/4));
    boxArea = (apothem * (box_size/4.0)) * 6.0;
  }

  // Also use this to make contraction a little more exact
  double robotArea;
  if (robot_type == Robot::SHAPE_RECT)
    robotArea = robot_size*robot_size;
  //else if (robot_type == Robot::SHAPE_CIRC)
   // robotArea = M_PI*((robot_size/2)*(robot_size/2));
  else // robot_size = HEX
  {
    //double perimeter = 6*(robot_size/2);
    double apothem = sqrt((robot_size/2)*(robot_size/2) - (robot_size/4)*(robot_size/4));
    robotArea = (apothem * (robot_size/4.0)) * 6.0;
  }

  uint64_t maxsteps = 100000L;

  fprintf(stderr, "Initializing.");
  std::vector<Goal*> tempGoals; //For recursion purposes
  //world->populateGoals(RADMIN, 0, tempGoals);
  printf("\nNumber of goals: %i\n", (int)world->numGoals);

  // This gets the goal polygon center dead on with
  // the center of convergence; important for measuring success
  if (world->havePolygon)
  {
    world->populateGoalPolygon(boxArea, robotArea, robot_size, world->polygon);
    world->centerGoalPolygonAgainstLights();
  }

  // Must do this after populating goals
  if (outputFileName != "")
  {
    world->saveWorldHeader(outputFileName);
    world->saveGoalsToFile(outputFileName);
    world->savePerformanceFileHeader(performanceFileName, outputFileName, maxsteps);
  }

  // These lines prime the polygon
  if (world->havePolygon && flare > 0)
  {
    // Adjust the polygon to account for corners
    // Note that we need to be centered around the origin
    world->polygon->translate(-goalx, -goaly, true);
    world->polygon->primeCorners(flare);
    world->polygon->translate(goalx, goaly, true);
  }

  double RADMIN = world->GetRadMin(boxArea, robotArea, robot_size, world->polygon);

  // We need to adjust the user polygon to fit the arena
  double radius = RADMAX;
  if (world->havePolygon)
  {
    world->polygon->markConcavePoints();
    RADMAX = world->GetSetRadMax(world->polygon);
    radius = world->polygon->getDistFromPoint(goalx, goaly);
  }

  double CIRCLERADMAX = (WIDTH / 2.0) * 0.75;

  // Can stop the holding behaviour by setting this to false
  // holdFor is set automatically below; it should be 0 here to begin
  bool holdAtMin = true;
  double holdTime = 1;
  //holdTime = 2500/updateRate; // THIS WAS THE OLD VALUE OF HOLD TIME
  
  // This is not a parameter leave it at 0
  double holdFor = 0;


  double GoalRadCircle = sqrt((world->boxes.size()*boxArea)/M_PI);
  if (!world->havePolygon)
    world->minimumRad = GoalRadCircle;

  // If we have an input file we don't need to calculate states
  // The while here becomes the whole main loop
  if (inputFileName != "")
  {
    // It doesn't make sense to skip frames here
    world->draw_interval = 1;
    std::ifstream file(inputFileName);
    world->loadGoalPolygon(inputFileName);
    while (!world->RequestShutdown() && running)
    {
      if (!world->replay_paused)
      {
        if (world->steps % updateRate == 1) // every now and again
        {
          running = world->loadNextState(file);
          checkSuccess--;
        }
        if (checkSuccess == 0) // We do not need to do this very frequently
        {
          checkSuccess = 100;
          numCorrect = 0;
          for (auto b : world->boxes)
          {
            if (b->insidePoly)
              numCorrect++;
          }
          printf("%f%% boxes correct.\n", (numCorrect/world->boxes.size()) * 100.00);
        }
      }
      world->Step(timeStep);
      world->paused = true;
    }
    world->replay_paused = true;
    while(world->replay_paused)
    {
      world->Step(timeStep);
    }
    return 0; // Finished reading the file, close
  }

  /* Loop until the user closes the window */
  // Note that for irregular polygons we define the radius as the shortest distance
  // to any point on the polygon
  int writeState = GUITIME;
  fprintf(stderr, "\nRunning...");
  while (!world->RequestShutdown() && world->steps < maxsteps)
  {
    // TODO: Refactor this monster of a loop
    if (world->steps % updateRate == 1) // every now and again
    {
      // Are we staying contracted?
      if (holdFor != 0 && holdAtMin)
      {
        // Don't change radius
        world->UpdateLightPattern(goalx, goaly, 1, radius, PATTWIDTH, drag);
        holdFor--;
        // If we are done contracting, we need to grow above RadMin threshold
        if (holdFor == 0)
        {
          if (world->havePolygon)
            while(radius < RADMIN)
            {
              world->polygon->scale(sdelta);
              radius *= sdelta;
            }
          else
            radius += delta;
        }
      }
      else // We aren't staying contracted
      {
        if (radius < RADMIN)
        {
          //delta = -delta; // * 2.0;
          sdelta = 2-sdelta; // Switch to expansion

          //xdelta = 0.1;
          if (holdAtMin)
            // Trial and error: this is a decent heuristic
            holdFor = holdTime;
          continue;
        }

        else if (((radius > RADMAX) && world->usePolygon) || (radius > CIRCLERADMAX && !world->usePolygon))
          sdelta = 2-sdelta; // Switch to contraction
          //delta = -delta; //downdelta;

        // This shouldn't be an else despite the above
        if (((radius <= RADMAX) && world->usePolygon) || (radius <= CIRCLERADMAX && !world->usePolygon))
        {
          if (world->havePolygon && switchToCircle) // proxy to determine if a polygon was supplied
          {
            // These switch between circle and polygon
            // Can opt to use e.g. 0.25 instead of 0.5 to make switching radius tighter
            if (radius > WIDTH/8.0 && world->usePolygon && sdelta > 1)
            {
              world->usePolygon = false;
              radius = world->polygon->getDistFromPoint(world->polygon->cx, world->polygon->cy);
            }
            else if (radius < WIDTH/8.0 && !world->usePolygon && sdelta < 1)
            {
              world->usePolygon = true;
              radius = world->polygon->getDistFromPoint(world->polygon->cx, world->polygon->cy);
            }
          }
          // Turns all necessary lights on for a specific amount of contraction (radius)
          // The polygon will automatically be used if it is well defined
          if (drag != 0)
            world->UpdateLightPattern(goalx, goaly, 1, radius, PATTWIDTH, drag);
          else
            world->UpdateLightPattern(goalx, goaly, 1, radius, PATTWIDTH, 0);
        }
#if 0
              for( int i=0; i<18; i+=2 )
                {
                  size_t index = letterL[i] + letterL[i+1] * lside;      
                  world->SetLightIntensity( index, 1 );       
                }
#endif

        // This handles both contractions and dilation
        if (holdFor == 0) // If we aren't staying contracted
        {
          if (world->havePolygon && world->usePolygon)
            world->polygon->scale(sdelta);
          radius *= sdelta;
        }

        // Optionally move the collected resources
        // goalx += xdelta;
        // goaly += ydelta;
      }
    }

    if (--writeState == 0)
    {
      world->appendWorldStateToFile(outputFileName);
      writeState = GUITIME;
    }

    if (world->steps % (updateRate*10) == 1) // We do not need to do this very frequently
    {
      double successRate = world->evaluateSuccessInsidePoly(GoalRadCircle, performanceFileName);
      printf("%ld steps: %f%% boxes correct.\n", world->steps, successRate * 100);
    }

    world->Step(timeStep);
  }

  printf("\nCompleted %lu steps.\n", world->steps);
  double successRate = world->evaluateSuccessInsidePoly(GoalRadCircle, performanceFileName);
  if (outputFileName != "")
    world->saveSuccessMeasure(outputFileName);
  printf("%f%% of the boxes are in the right position.\n", successRate * 100);

  return 0;
}
