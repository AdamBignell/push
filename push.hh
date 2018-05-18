#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>
#include <vector>

// Note that the headers for are all push source files are found here

enum _entityCategory
{
  BOXBOUNDARY = 0x1,
  ROBOTBOUNDARY = 0x2,
  ROBOT = 0x4,
  BOX = 0x8
};

class Robot;

class Light
{
public:
  double intensity;
  double x, y, z; // location (z is height above the x,y plane of the world)

  Light(double x, double y, double z, double intensity) : x(x), y(y), z(z), intensity(intensity)
  {
  }
};

class Robot;
class Box;

class World
{
public:
  b2World *b2world;

  double width, height, numLights;

  b2Body *boxWall[4];
  b2Body *robotWall[4];

  size_t steps;
  std::vector<Light *> lights;
  std::vector<Box *> boxes;
  std::vector<Robot *> robots;

  World(double width, double height, double numLights);

  virtual void AddRobot(Robot *robot);
  virtual void AddBox(Box *box);
  virtual void AddLight(Light *light);
  virtual void AddLightGrid(size_t xcount, size_t ycount, double height, double intensity);

  // set the intensity of the light at @index. If @index is out of
  // range, the call has no effect
  virtual void SetLightIntensity(size_t index, double intensity);

  // Lets us update the pattern of light in one function call with a few parameters
  // (goalx, goaly): center of contraction
  // probOn: How likely an 'on' is likely to actually be on. Useful for robot spacing
  // radius: Determines how contracted we are
  // pattwidth: The width of the contracting patten (note this isn't diameter)
  void UpdateLightPattern(double goalx, double goaly, double probOn, double radius, double pattwidth);

  // return instantaneous light intensity from all sources
  double GetLightIntensityAt(double x, double y);

  // perform one simulation step
  void Step(double timestep);

  // Get the minimum contracted size
  // Use total box area to estimate
  virtual double GetRadMin(double numBoxes, double boxSize);
};

class GuiWorld : public World
{
public:
  static bool paused;
  static bool step;
  static int skip;

  bool lights_need_redraw;
  std::vector<double> bright;

  GLFWwindow *window;
  int draw_interval;

  GuiWorld(double width, double height, double numLights);
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

  typedef enum
  {
    SHAPE_RECT = 0,
    SHAPE_HEX,
    SHAPE_CIRC
  } box_shape_t;

  b2Body *body;

  Box(World &world, box_shape_t shape, double size, double x, double y, double a);
};
