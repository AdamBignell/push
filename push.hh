#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>
//#include "b2dJson/b2dJson.h"
#include <vector>
#include <string>

// Note that the headers for are all push source files are found here

enum _entityCategory
{
  BOXBOUNDARY = 0x1,
  ROBOTBOUNDARY = 0x2,
  ROBOT = 0x4,
  BOX = 0x8,
  GOAL = 0x10
};

class Robot;

class Light
{
public:
  double intensity;
  double x, y, z; // location (z is height above the x,y plane of the world)
  int index;

  Light(double x, double y, double z, double intensity, int index ): x(x), y(y), z(z), intensity(intensity), index(index)
  {
  }
};

class Vertex
{
public:
  double x, y;
  bool userVert; // Is the vertex user-supplied or added as part of our control strategy?

  Vertex(double x, double y, bool userVert) : x(x), y(y), userVert(userVert)
  {
  }

  Vertex(double x, double y) : x(x), y(y)
  {
    userVert = false;
  }
};

class Polygon
{
public:
  std::vector<Vertex> vertices;

  double cx, cy; // center

  Polygon()
  {
    cx = 0;
    cy = 0;
  }

  Polygon(double cx, double cy);
  
  Polygon(std::vector<Vertex> newV, double cx, double cy);

  void addVertex(double x, double y, bool userVert);
  void translate(double dx, double dy, bool recenter);
  void scale(double s, double cx, double cy);
  void scale(double d);
  double getArea();
  double getSignedArea();

  // Just sets the point that scaling is done with respect to
  void setOrigin(double cx, double cy);
  Vertex getCentroid();

  // Calculates shortest distance from point
  // to line connecting vertices
  double getDistFromPoint(double x, double y);

  // The average distance between the vertices and a point
  double getAvgDistFromPoint(double x, double y);

  // Skew the corners so that you end up with the right shape
  // This attempts to fix the 'rounded square' effect
  void primeCorners(double flare);

  // Uses ray-casting algorithm
  bool pointInsidePoly(double x, double y);
};

class Robot;
class Box;
class Goal;

class World
{
public:
  b2World *b2world;

  //b2dJson jsonWorld;
  std::string worldString;

  double width, height;
  int numLights;
  int draw_interval;

  b2Body *boxWall[4];
  b2Body *robotWall[4];

  Polygon polygon;

  // This defines the desired shape upon completion
  Polygon goalPolygon;

  bool havePolygon;

  // Just for convenience
  // Doesn't make sense to pause a non-gui world
  static bool paused;

  size_t steps;
  std::vector<Light *> lights;
  std::vector<Box *> boxes;
  std::vector<Robot *> robots;
  std::vector<Goal *> goals;

  World(double width, double height, int numLights, int drawInterval);

  virtual void AddRobot(Robot *robot);
  virtual void AddBox(Box *box);
  virtual void AddLight(Light *light);
  virtual void AddLightGrid(size_t xcount, size_t ycount, double height, double intensity);
  virtual void AddGoal(Goal *goal);

  // set the intensity of the light at @index. If @index is out of
  // range, the call has no effect
  virtual void SetLightIntensity(size_t index, double intensity);

  // Just for convenience. Doesn't make sense to shut-down a non-gui-world
  virtual bool RequestShutdown() { return false; };

  // Pull the next world state from the file
  bool loadNextState(std::ifstream& file);

  bool loadPolygonFromFile(std::ifstream& infile);

  // Lets us update the pattern of light in one function call with a few parameters
  // (goalx, goaly): center of contraction
  // probOn: How likely an 'on' is likely to actually be on. Useful for robot spacing
  // radius: Determines how contracted we are
  // pattwidth: The width of the contracting patten (note this isn't diameter)
  // conrerRate: Sort of sophisticated. This defines the percentage of the total distance between the corner
  //             and the midpoint to the next vertex that will be on. So if 1, there's no effect. If 0.5, the half of the
  //             lights closer to the vertex will be turned off. We use a percentage to acount for arbitrary polygons
  void UpdateLightPattern(double goalx, double goaly, double probOn, double radius, double pattwidth, double cornerRate);

  // return instantaneous light intensity from all sources
  double GetLightIntensityAt(double x, double y);

  // perform one simulation step
  virtual void Step(double timestep);

  // Get the minimum contracted size
  // Use total box area to estimate
  double GetRadMin(double numBoxes, double boxArea, double robot_size, Polygon tempPoly);

  // Approximately Match arena Size,
  // and actually set the polgyon to this size
  double GetSetRadMax(Polygon& tempPoly);

  // Saves the world state to a JSON file
  void saveWorldHeader(std::string saveFileName);
  void appendWorldStateToFile(std::string saveFileName);

  void updateRobotsFromString(std::string &robotStr);
  void updateBoxesFromString(std::string &boxStr);
  void updateLightsFromString(std::string &lightStr);

  // Note that all the information we need is part of the world already
  // Hence the argumentless call
  bool populateGoals(double RADMIN);
};

class GuiWorld : public World
{
public:
  //static bool paused;
  static bool step;
  static int skip;

  bool lights_need_redraw;
  std::vector<double> bright;

  GLFWwindow *window;

  GuiWorld(double width, double height, int draw_interval, int numLights);
  ~GuiWorld();

  virtual void Step(double timestep);

  virtual void AddLight(Light *light)
  {
    lights_need_redraw = true;
    World::AddLight(light);
  }

  virtual void AddLightGrid(size_t xcount, size_t ycount, double height, double intensity)
  {
    lights_need_redraw = true;
    World::AddLightGrid(xcount, ycount, height, intensity);
  }

  // set the intensity of the light at @index. If @index is out of
  // range, the call has no effect
  virtual void SetLightIntensity(size_t index, double intensity)
  {
    lights_need_redraw = true;
    World::SetLightIntensity(index, intensity);
  }

  bool RequestShutdown();
};

class Robot
{
public:
  World &world;
  double size;
  char cshape;

  typedef enum
  {
    SHAPE_RECT = 0,
    SHAPE_CIRC
  } robot_shape_t;

  double drive_gain; // Scale driving speed
  double turn_gain;  // Scale turning speed

  double charge;       // joules
  double charge_max;   // maximum storage capacity
  double charge_delta; // rate of change of charge

  double input_efficiency;  // scale input from light sources
  double output_metabolic;  // cost per step of just being alive
  double output_efficiency; // scale output due to motion

  double targets[7];

  static std::vector<Light> lights;

  b2Body *body; //, *bumper;
  //  b2PrismaticJoint* joint;

  Robot(World &world,
        double x, double y, double a, // pose
        robot_shape_t shape = SHAPE_RECT, // rectangle
        double size = 0.5,
        double drive_gain = 5.0,
        double turn_gain = 5.0,
        double charge = 100.0,
        double charge_max = 100.0,
        double input_efficiency = 0.5,
        double output_metabolic = 0.01,
        double output_efficiency = 0.1);

  virtual void Update(double timestep);

  //protected:
  // get sensor data
  double GetLightIntensity(void) const;
  double GetLightIntensityAt(double x, double y) const;

  // send commands
  void SetSpeed(double x, double y, double a);

private:
  void UpdateTargetSensor(void);
};

class Box
{
public:
  double size;
  char cshape;

  typedef enum
  {
    SHAPE_RECT = 0,
    SHAPE_HEX,
    SHAPE_CIRC
  } box_shape_t;

  b2Body *body;

  Box(World &world, box_shape_t shape, double size, double x, double y, double a);
};

class Goal
{
public:
  double x, y;
  double size;

  b2Body *body;

  typedef enum
  {
    SHAPE_RECT = 0,
    SHAPE_HEX,
    SHAPE_CIRC
  } goal_shape_t;

  goal_shape_t shape;

  Goal(World &world, double x, double y, double size, goal_shape_t shape);
};