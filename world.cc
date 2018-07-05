#include "push.hh"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <stdlib.h>

World::World(double width, double height, int numLights, int drawInterval, double flare, double drag, bool switchToCircle) : steps(0),
                                                              width(width),
                                                              height(height),
                                                              numLights(numLights),
                                                              draw_interval(drawInterval),
                                                              flare(flare),
                                                              drag(drag),
                                                              switchToCircle(switchToCircle),
                                                              havePolygon(false),
                                                              b2world(new b2World(b2Vec2(0, 0))), // gravity
                                                              lights()                            //empty vector
{
  
  //set interior box container
  b2BodyDef boxWallDef;
  b2PolygonShape groundBox;

  polygon = new Polygon();
  goalPolygon = new Polygon();

  //groundBox.SetAsBox(width / 4.0, 0.01f); // Zoomed in
  groundBox.SetAsBox(width / 8.0, 0.01f); // Zoomed Out

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &groundBox;

  // prevent collision with puck-retaining strings
  fixtureDef.filter.categoryBits = BOXBOUNDARY;
  fixtureDef.filter.maskBits = BOX; // contain only boxes

  for (int i = 0; i < 4; i++)
  {
    boxWall[i] = b2world->CreateBody(&boxWallDef);
    boxWall[i]->CreateFixture(&fixtureDef);
  }

  success = -1;

  // Populate the goals
  numGoals = 0;
  for (int i = 0; i < width; ++i)
  {
    std::vector<std::vector<Goal*>> goalCol;
    for (int j = 0; j < height; ++j)
    {
      std::vector<Goal*> goalList;
      goalCol.push_back(goalList);
    }
    goals.push_back(goalCol);
  }

  // Zoomed in
  // boxWall[0]->SetTransform(b2Vec2(width / 2, height / 4.0), 0);
  // boxWall[1]->SetTransform(b2Vec2(width / 2, height - height / 4.0), 0);
  // boxWall[2]->SetTransform(b2Vec2(width / 4.0, height / 2), M_PI / 2.0);
  // boxWall[3]->SetTransform(b2Vec2(width - width / 4.0, height - height / 2.0), M_PI / 2.0);

  // Zoomed Out
  double ldx = (sqrt(numLights)/width)/2.0;
  double ldy = (sqrt(numLights)/height)/2.0;
  boxWall[0]->SetTransform(b2Vec2((width / 2) + ldx, height * (3/8.0) + ldy), 0);
  boxWall[1]->SetTransform(b2Vec2((width / 2) + ldx, height - (height * (3/8.0)) + ldy), 0);
  boxWall[2]->SetTransform(b2Vec2(width * (3/8.0) + ldx, height / 2  + ldy), M_PI / 2.0);
  boxWall[3]->SetTransform(b2Vec2(width - (width * (3/8.0)) + ldx, height - (height / 2.0) + ldy), M_PI / 2.0);

  // set exterior box container
  b2BodyDef boxWallDef1;
  b2PolygonShape groundBox1;
  groundBox1.SetAsBox(width, 0.01f);

  b2FixtureDef fixtureDef1;
  fixtureDef1.shape = &groundBox1;

  // prevent collision with puck-retaining strings
  fixtureDef1.filter.categoryBits = ROBOTBOUNDARY;
  fixtureDef1.filter.maskBits = ROBOT | BOX; // contain everthing

  for (int i = 0; i < 4; i++)
  {
    robotWall[i] = b2world->CreateBody(&boxWallDef1);
    robotWall[i]->CreateFixture(&fixtureDef1);
  }

  robotWall[0]->SetTransform(b2Vec2(width / 2, 0), 0);
  robotWall[1]->SetTransform(b2Vec2(width / 2, height), 0);
  robotWall[2]->SetTransform(b2Vec2(0, height / 2), M_PI / 2.0);
  robotWall[3]->SetTransform(b2Vec2(width, height / 2), M_PI / 2.0);
}

void World::AddLight(Light *l)
{
  lights.push_back(l);
}

void World::AddLightGrid(size_t xcount, size_t ycount, double z, double intensity)
{
  double xspace = width / (double)xcount;
  double yspace = height / (double)ycount;

  for (size_t y = 0; y < ycount; y++)
    for (size_t x = 0; x < xcount; x++)
      AddLight(new Light(x * xspace + xspace / 2.0,
                         y * yspace + yspace / 2.0,
                         z,
                         intensity, x + y * xcount));
} // We assume that xcount=ycount above

void World::AddRobot(Robot *r)
{
  robots.push_back(r);
}

void World::AddBox(Box *b)
{
  boxes.push_back(b);
}

void World::AddGoal(Goal *g)
{
  goals[floor(g->x)][floor(g->y)].push_back(g);
  numGoals++;
}

void World::SetLightIntensity(size_t index, double intensity)
{
  if (index < lights.size())
    lights[index]->intensity = intensity;
}

void World::UpdateLightPattern(double goalx, double goaly, double probOn, double radius, double PATTWIDTH, double cornerRate)
{
  double lside = sqrt(lights.size());
  double lx = width / lside;
  double ly = height / lside;
  double r2 = radius * radius;
  double randOn, cx, cy, c, c2;
  double dist;
  std::vector<std::tuple<double, int>> lightsOn;
  for (int x = 0; x < lside; x++)
    for (int y = 0; y < lside; y++)
    {
      int on = 0;
      if (usePolygon) // Use the polygon
      {
        on = (fabs(polygon->getDistFromPoint(x, y) < fmax(fmax(lx,ly)/2,PATTWIDTH)));
        if (on && drag != 0)
        {
          double minDist = width*height;
          for (auto vertex: polygon->vertices)
          {
            if (vertex.userVert)
            {
              // Use squared distance since we only care about order
              dist = ((x - vertex.x) * (x - vertex.x)) + ((y - vertex.y) * (y - vertex.y));
              if (dist < minDist)
                minDist = dist;
            }
          }
          std::tuple<double, int> lightTuple(minDist, x + y * lside);
          lightsOn.push_back(lightTuple);
        }
      }
      else // Use the circle
      {
        // (Number of lights between) * (distance between lights)
        cx = (x - goalx) * lx;
        cy = (y - goaly) * ly;

        c = sqrt(cx * cx + cy * cy);

        //c2 = cx * cx + cy * cy; // Square distance
        // int on = (fabs(c2 - r2) < PATTWIDTH*PATTWIDTH);

        // We turn the light on if the light is within the specified closeness
        // or within 1 light away if this value is greater than the PATTWIDTH

        on = (fabs(c - radius) < fmax(fmax(lx,ly)/2,PATTWIDTH));
      }

      randOn = ((double)rand() / (RAND_MAX));
      // Use 1D indexing
      // Note that if on == 0, we just turn the light off regardless of randOn
      SetLightIntensity(x + y * lside,
                        on * (randOn <= probOn));
      // (fabs( c2 - r2 ) < lside) ); Old version: Why is this lside?
    }
  
  // This lexicographically sorts the tuples by distance
  if (drag != 0)
  {
    std::sort(lightsOn.begin(), lightsOn.end());
    double lightsTurnedOff = 0;
    int i = 0;
    while (lightsTurnedOff / lightsOn.size() < cornerRate*drag || lightsTurnedOff / lightsOn.size() == 1.0)
    {
      SetLightIntensity(std::get<1>(lightsOn[lightsTurnedOff]), 0);
      lightsTurnedOff += 1.0;
    }
  }
}

double World::GetLightIntensityAt(double x, double y)
{
  // integrate brightness over all light sources
  double total_brightness = 0.0;

  const double maxdist = width / 5.0;

  // only inspect lights that are less than maxdist away
  const size_t numlights = lights.size();
  const int lwidth = sqrt(numlights);
  const int lheight = lwidth;

  //printf( "%lu numlights, %d lwidth %d lheight\n", numlights, lwidth, lheight );

  // find the light grid position of x,y
  const double scale = (double)lwidth / (double)width;
  const int lx = x * scale;
  const int ly = y * scale;

  // find the half-width of the neighborhood we're interested in
  const int halfwidth = maxdist * scale;

  //printf( "%.2f,%.2f  is cell %d,%d and halfwidth is %d\n", x, y, lx, ly, halfwidth );

  for (int yy = std::max(0, ly - halfwidth); yy < std::min(lheight - 1, ly + halfwidth); yy++)
    for (int xx = std::max(0, lx - halfwidth); xx < std::min(lwidth - 1, lx + halfwidth); xx++)
    {
      const size_t index = xx + yy * lwidth;

      //	printf( "  %d,%d %lu\n", xx, yy, index );

      assert(index < lights.size());

      auto &l = lights[index];

      if (l->intensity == 0.0)
        continue;

      // horizontal and vertical distances
      const double dx = x - l->x;
      const double dy = y - l->y;

      if (fabs(dx) > maxdist || fabs(dy) > maxdist)
        continue;

      const double dz = l->z;
      const double distsquared = dx * dx + dy * dy + dz * dz;
      const double dist = sqrt(distsquared);

      // brightness as a function of distance
      const double brightness = l->intensity / distsquared;

      // now factor in the angle to the light
      const double theta = atan2(dz, hypot(dx * dx, dy * dy));

      // and integrate
      double noise = (drand48() - 0.5) * 0.00;
      total_brightness += brightness * sin(theta) + noise;
    }

  return total_brightness;
}

void World::Step(double timestep)
{
  for (auto &r : robots)
    r->Update(timestep);

  const int32 velocityIterations = 6;
  const int32 positionIterations = 2;

  // Instruct the world to perform a single step of simulation.
  // It is generally best to keep the time step and iterations fixed.
  b2world->Step(timestep, velocityIterations, positionIterations);

  steps++;
}

// Get the minimum contracted size
// Use total box area to estimate
double World::GetRadMin(double boxArea, double robotArea, double robot_size, Polygon* realPoly)
{
  Polygon tempPoly(*realPoly);
  double totalBoxArea = boxes.size() * boxArea;
  double totalRobotArea = robots.size() * robotArea; // 0.75 makes the robots put pressure on the boxes
  double desiredArea = totalBoxArea + totalRobotArea;
  double testScale = 0.975;

  if (havePolygon)
  {
    double currRad = tempPoly.getDistFromPoint(tempPoly.cx,tempPoly.cy);
    double currArea = tempPoly.getArea();
    double ratioArea = desiredArea / currArea;
    double oneDRatio = sqrt(ratioArea);

    // NOTE THAT THE MIN RADIUS != THE RADIUS OF THE POLYGON
    // We stop contracting to account for robot area
    // but any box that is in this robot-band does not count as a hit
    tempPoly.scale(sqrt(totalBoxArea/currArea));
    goalPolygon->vertices = tempPoly.vertices;
    goalPolygon->cx = tempPoly.cx;
    goalPolygon->cy = tempPoly.cy;

    double minRad = currRad * oneDRatio;
    double correctRad = tempPoly.getDistFromPoint(tempPoly.cx,tempPoly.cy);

    fprintf(stderr, "Total Box Area = %f, Goal Polygon Area = %f\n", totalBoxArea, goalPolygon->getArea());

    return minRad;
  }

  //Circle case
  double radMin = sqrt(totalBoxArea / M_PI) + robot_size; // Need literal wiggle room
  return radMin;
}

double World::GetSetRadMax(Polygon* tempPoly){
  //TODO: This still feels buggy for user-defined polygons
  if (havePolygon)
  {
    double arenaArea = (width*height) * (0.50);
    double testScale = 1.05;
    double area = tempPoly->getArea();
    // Can we do this loop algebraically?
    while (area < arenaArea) // Want to be sufficiently wide
    {
      tempPoly->scale(testScale);
      area *= testScale*testScale;
    }
    return tempPoly->getDistFromPoint(tempPoly->cx,tempPoly->cy);
  }

  //Circle case
  double radMax = sqrt(width*height / M_PI);
  return radMax;
}

void World::saveWorldHeader(std::string saveFileName)
{
  std::ofstream outfile;
  outfile.open(saveFileName);

  outfile << "HEADER:\n";
  outfile << "-w " << width << " -h " << height;
  outfile << " -r " << robots.size() << " -b " << boxes.size();
  outfile << " -z " << robots[0]->size << " -s " << boxes[0]->size;
  outfile << " -t " << robots[0]->cshape << " -y " << boxes[0]->cshape;
  outfile << " -f " << flare;
  outfile << " -g " << "1" << '\n';
  outfile << "$\n";
}

void World::savePerformanceFileHeader(std::string saveFileName, std::string userFileName)
{
  std::ofstream outfile;
  outfile.open(saveFileName);

  outfile << saveFileName << "\n";
  outfile << "HEADER:\n";
  outfile << "Replay: " << userFileName << "\n";
  outfile << "Width: " << width << "\n";
  outfile << "Height: " << height << "\n";
  outfile << "#Robots: " << robots.size() << "\n";
  outfile << "#Boxes: " << boxes.size() << "\n";
  outfile << "RobotSize: " << robots[0]->size << "\n";
  outfile << "BoxSize: " << boxes[0]->size << "\n";
  outfile << "RobotShape: " << robots[0]->cshape << "\n";
  outfile << "BoxShape: " << boxes[0]->cshape << "\n";
  if (flare > 0)
    outfile << "Flare: " << flare << "\n";
  else
    outfile << "Flare: NONE\n";
  if (drag > 0)
    outfile << "Drag: " << drag << "\n";
  else
    outfile << "Drag: NONE\n";
  outfile << "SwitchToCircle: " << switchToCircle << "\n";
  outfile << "TargetShape: ";
  if (goalPolygon->vertices.size() == 0)
    outfile << "Circle\n";
  else
  {
    for (int i = 0; i < goalPolygon->vertices.size(); ++i)
    {
      Vertex v = goalPolygon->vertices[i];
      outfile << v.x << " " << v.y;
      if (i != goalPolygon->vertices.size() -1)
        outfile << ", ";
    }
  }
  outfile << "\n$\n";
}

void World::saveSuccessMeasure(std::string saveFileName)
{
  std::ofstream outfile;
  outfile.open(saveFileName, std::ios_base::app);

  outfile << "Success:\n";
  outfile << "!\n";
  outfile << success << '\n';
  outfile << "!\n";
  outfile << "$\n";
}

void World::saveGoalsToFile(std::string saveFileName)
{
  std::ofstream outfile;
  outfile.open(saveFileName, std::ios_base::app);

  outfile << "!\n"; // Write a delimeter
  outfile << "GOALS:\n" << "!\n";
  for (auto &col: goals)
  {
    for (auto &row : col)
    {
      for (auto &g : row)
      {
        // x, y, a, charge
        outfile << g->x << ' ' << g->y << ' ' << g->size << ' ' <<  g->shape << '\n';
      }
    }
  }

  outfile << "!\n"; // Write a delimeter
}

// Saves the world state to a JSON file
void World::appendWorldStateToFile(std::string saveFileName)
{
  // Initial attempt using box2dJson
  // worldString = jsonWorld.writeToString(b2world);
  std::ofstream outfile;
  outfile.open(saveFileName, std::ios_base::app);

  // The goals don't change position!
  // We write them separately and only once

  b2Vec2 pose;
  // Write the robot information
  outfile << "ROBOTS:\n" << "!\n";
  for (auto bot : robots)
    {
        // x, y, a, charge
        pose = bot->body->GetPosition();
        outfile << pose.x << ' ' << pose.y << ' ' << bot->body->GetAngle()<< ' ' <<  bot->charge << '\n';
    }
  outfile << "!\n"; // Write a delimeter
  // Write the box information
  outfile << "BOXES:\n" << "!\n";
  for (auto box : boxes)
    {
        // x, y, a
        pose = box->body->GetPosition();
        outfile << pose.x << ' ' << pose.y << ' ' << box->body->GetAngle() << ' ' << box->insidePoly << '\n';
    }
  outfile << "!\n";
  // Write the light information
  outfile << "LIGHTS:\n"  << "!\n";
  for (auto light : lights)
    {
      if (light->intensity != 0)
      {
        // x, y, a, intensity
        outfile << light->index << ' ' << light->intensity << '\n';
      }
    }
  // outfile << worldString; // Write the world state
  outfile << "!\n";
  outfile << "$\n";
}

// Takes a section of robots and updates the positions/charges in the world
void World::updateRobotsFromString(std::string &robotStr)
{
  std::istringstream iss(robotStr);
  std::string robot;
  int rDex = 0;
  getline(iss, robot, '\n'); // dump the first
  float x, y, a, charge;
  while (getline(iss, robot, '\n'))
  {
    std::istringstream poseStr(robot);
    poseStr >> x >> y >> a >> charge;
    b2Vec2 pose(x,y);
    robots[rDex]->body->SetTransform(pose, a);
    robots[rDex]->charge = charge;
    rDex++;
  }
}

// Takes a section of boxes and updates the positions in the world
void World::updateBoxesFromString(std::string &boxStr)
{
  std::istringstream iss(boxStr);
  std::string box;
  int bDex = 0;
  getline(iss, box, '\n'); // dump the first
  float x, y, a;
  bool insidePoly;
  while (getline(iss, box, '\n'))
  {
    std::istringstream poseStr(box);
    poseStr >> x >> y >> a >> insidePoly;
    b2Vec2 pose(x,y);
    boxes[bDex]->body->SetTransform(pose, a);
    boxes[bDex]->insidePoly = insidePoly;
    bDex++;
  }
}

// Takes the section of goals
void World::updateGoalsFromString(std::string &goalStr)
{
  std::istringstream iss(goalStr);
  std::string goal;
  double x, y, size;
  getline(iss, goal, '\n'); // dump the first
  int shape;
  while (getline(iss, goal, '\n'))
  {
    if (goal == "")
      continue;
    std::istringstream indexStr(goal);
    indexStr >> x >> y >> size >> shape;
    goals[x][y].push_back(new Goal(this, x, y, size, (Goal::goal_shape_t)shape));
    ++numGoals;
  }
}

// Takes a section of boxes and updates the brightness in the world
void World::updateLightsFromString(std::string &lightStr)
{
  double lside = sqrt(numLights);
  std::istringstream iss(lightStr);
  std::string light;
  getline(iss, light, '\n'); // dump the first
  float index, intensity;
  for (int i = 0; i < lights.size(); ++i)
  {
    // Zero everything
    // We save a ton of space in the state file this way
    // since on average we can assume a light is off
    lights[i]->intensity = 0;
  }
  double xdex, ydex;
  while (getline(iss, light, '\n'))
  {
    std::istringstream indexStr(light);
    indexStr >> index >> intensity;
    SetLightIntensity(index, intensity);
  }
}

// Takes a section of boxes and updates the brightness in the world
void World::updateSuccessFromString(std::string &succStr)
{
  double lside = sqrt(numLights);
  std::istringstream iss(succStr);
  std::string light;
  getline(iss, light, '\n'); // dump the first
  double xdex, ydex;
  while (getline(iss, light, '\n')) // should only be one
  {
    std::istringstream indexStr(light);
    indexStr >> success;
  }
}

// Given an already opened file, reads in the next state
bool World::loadNextState(std::ifstream& file)
{
  std::string sectionStr;
  std::string lineStr;
  bool running = false;
  if(getline(file, sectionStr, '$')) // WorldStates
  {
    running = true;
  }

  std::istringstream sectionStream(sectionStr);
  while(getline(sectionStream, lineStr, '!')) // Sections
  {
    char section = lineStr.c_str()[1]; // ignore the newline
    switch (section)
    {
    case 'H':
      // This shouldn't happen. Just dump the options
      getline(sectionStream, lineStr, '!');
    break;
    case 'R':
      getline(sectionStream, lineStr, '!');
      updateRobotsFromString(lineStr);
    break;
    case 'B':
      getline(sectionStream, lineStr, '!');
      updateBoxesFromString(lineStr);
    break;
    case 'L':
      getline(sectionStream, lineStr, '!');
      updateLightsFromString(lineStr);
    break;
    case 'G':
      getline(sectionStream, lineStr, '!');
      updateGoalsFromString(lineStr);
    break;
    case 'S':
      getline(sectionStream, lineStr, '!');
      updateSuccessFromString(lineStr);
    default:
    ;
    }
  }
  return running;
}

bool World::loadPolygonFromFile(std::ifstream& infile)
{
  std::string line;
  while (std::getline(infile, line))
  {
    std::istringstream vertPair(line);
    double x, y;
    if (!(vertPair >> x >> y)) { break; } // error
    polygon->addVertex(x, y, true);
  }

  if (polygon->vertices.size() < 3)
  {
    // Invalid polygon
    return false;
  }
  
  if (polygon->vertices.size() > 2)
  {
    // Used throughout to detect if we are in the circle or poly case
    havePolygon = true;
    usePolygon = true;
    return true;
  }
}

bool World::populateGoals(double RADMIN, int callNum, std::vector<Goal*>& tempGoals)
{
  // The recursive sections marked below by ** are pretty inefficient
  // but are extremely general. They will pack as well as possible
  // given the parameters we have been given. I think this is
  // reasonable since we only call this function once.
  fprintf(stderr, ".");

  // The below assumes we are packing hexagons
  double cx = ceil(width/2.0);
  double cy = ceil(height/2.0);
  double d = boxes[0]->size; // diameter
  double r = d/2.0; //radius
  double apothem = sqrt(d*d - r*r)/2.0;
  double oneBoxArea = (apothem * (r/2.0)) * 6.0;

  // bounding box
  double bbmaxx = -1 * std::numeric_limits<double>::infinity();
  double bbmaxy = -1 * std::numeric_limits<double>::infinity();
  double bbminx = std::numeric_limits<double>::infinity();
  double bbminy = std::numeric_limits<double>::infinity();

  getBoundingBox(goalPolygon->vertices, bbmaxx, bbmaxy, bbminx, bbminy, RADMIN);

  bbminx += apothem;
  bbminy += r;

  // This looks like a scary double for-loop but it's just
  // iterating through the centers of the hexagons that tile the arena
  // The i increment is how far up from hexagon center the next row of centers is
  double j, i = 0;
  int row;
  bool finished = false;
  for (i = bbminy; i < bbmaxy; i += (r + (r/2.0)))
  {
    // The j increment is the horizontal distance to the next center
    if (row % 2 == 0)
      j = bbminx;
    else
      j = bbminx - apothem;
    for (; j < bbmaxx; j += 2*apothem)
    {
      if (havePolygon) // User supplied a polygon
      {
        if (goalPolygon->pointInsidePoly(j, i))
        {
          tempGoals.push_back(new Goal(this, j, i, d, Goal::SHAPE_HEX));
          if (tempGoals.size() == boxes.size())
          {
            // Limit this so we don't loop forever
            if (goalPolygon->getArea() - tempGoals.size() * oneBoxArea > oneBoxArea && callNum != 20)
            {
              goalPolygon->scale(0.99);
              tempGoals.clear();
              populateGoals(RADMIN, callNum + 1, tempGoals); /**/
              return true;
            }
            else
              finished = true;
          }
        }
      }
      else // Goal is a circle
      {
        if (sqrt((cx-j)*(cx-j) + (cy-i)*(cy-i)) < RADMIN)
        {
          tempGoals.push_back(new Goal(this, j, i, d, Goal::SHAPE_HEX));
          if (tempGoals.size()  == boxes.size())
          {
            // Limit this so we don't loop forever
            if (M_PI*(RADMIN*RADMIN) - tempGoals.size() * oneBoxArea > oneBoxArea && callNum != 20)
            {
              // We underfilled. Try again
              RADMIN *= 0.99;
              tempGoals.clear();
              populateGoals(RADMIN, callNum + 1, tempGoals); /**/
              return true;
            }
            else
              finished = true;
          }
        }
      }
      if (finished)
        break;
    }
    if (finished)
      break;
    ++row;
  }

  // We overfilled. Try again.
  if (tempGoals.size() != boxes.size() && callNum < 20)
  {
    if (havePolygon) // Polygon
    {
      goalPolygon->scale(1.00 + (rand() % 10)/100);
      tempGoals.clear();
      populateGoals(RADMIN, callNum + 1, tempGoals); /**/
      return true;
    }
    else // Circle
    {
      RADMIN *= 1.00 + (rand() % 10)/100;
      tempGoals.clear();
      populateGoals(RADMIN, callNum + 1, tempGoals); /**/
      return true;
    }
  }

  // We only make it here if we are finalizing

  // Matches the goals to the center of contraction
  // Often this is a fractional value
  recenterGoals(tempGoals);

  // Ensures that the only goals we add are post adjustments
  for (auto &g : tempGoals)
  {
    AddGoal(new Goal(this, g->x, g->y, g->size, g->shape));
  }
  tempGoals.clear();

  while (boxes.size() != numGoals)
  {
    // Equalize boxes and goals for fairness
    boxes.pop_back();
  }

  return true;
}

  // Just clear all the innermost vectors
void World::clearGoals()
{
  for (auto &col: goals)
  {
    for (auto &row : col)
    {
      row.clear();
    }
  }
  numGoals = 0;
}

// Let's check how well we did
double World::evaluateSuccessNumGoals()
{
  unfulfillGoals();

  double d = boxes[0]->size; // diameter
  double r = d/2.0; //radius
  double apothem = sqrt(d*d - r*r)/2.0;
  double oneBoxArea = (apothem * (r/2.0)) * 6.0;
  double numCorrect = 0;
  double dist = 0;

  bool gotSuccess = false;

  double xmin,xmax,ymin,ymax;
  for (auto &box : boxes)
  {
    b2Vec2 pos = box->body->GetPosition();
    xmax = floor(pos.x) + 1;
    ymax = floor(pos.y) + 1;
    xmin = floor(pos.x) - 1;
    ymin = floor(pos.y) - 1;
    for (int j = ymin; j <= ymax; ++j)
    {
      for (int i = xmin; i <= xmax; ++i)
      {
        for (int k = 0; k < goals[i][j].size(); ++k)
        {
          Goal* g = goals[i][j][k];
          if (!g->fulfilled)
          {
            dist = sqrt((g->x - pos.x)*(g->x - pos.x) + (g->y - pos.y)*(g->y - pos.y));
            if (dist <= apothem)
            {
              // At most one goal will be fulfilled by a box since goalSize = boxSize
              numCorrect++;
              g->fulfilled = true;
              gotSuccess = true;
            }
          }
          if (gotSuccess)
            break;
        }
        if (gotSuccess)
          break;
      }
      if (gotSuccess)
        break;
    }
    if (gotSuccess)
    {
      gotSuccess = false;
      continue;
    }
  }
  // Capture the success so we can write it out later if need be
  success = numCorrect / numGoals;
  return numCorrect / numGoals;
}

// Let's check how well we did
double World::evaluateSuccessInsidePoly(double MINRAD, std::string perfFile)
{
  std::ofstream outfile;
  if (perfFile != "")
  {
    outfile.open(perfFile, std::ios_base::app);
    outfile << "!\n";
    outfile << "Step#: " << steps << "\n!" << "\n";
  }

  double d = boxes[0]->size; // diameter
  double r = d/2.0; //radius
  double apothem = sqrt(d*d - r*r)/2.0;
  double oneBoxArea = (apothem * (r/2.0)) * 6.0;
  double numCorrect = 0;
  double dist = 0;

  double ldx = sqrt(numLights)/width/2.0;
  double ldy = sqrt(numLights)/height/2.0;
  double trueCx = (width / 2.0) + ldx;
  double trueCy = (height / 2.0) + ldy;

  bool gotSuccess = false;

  double xmin,xmax,ymin,ymax;
  double totalDist;
  for (auto &box : boxes)
  {
    b2Vec2 pos = box->body->GetPosition();
    if (havePolygon)
    {
      if (goalPolygon->pointInsidePoly(pos.x,pos.y))
      {
        ++numCorrect;
        box->insidePoly = true;
        if (perfFile != "")
          outfile << 0 << "\n";
      }
      else
      {
        box->insidePoly = false;
        if (perfFile != "")
        {
          double dist = goalPolygon->getDistFromPoint(pos.x, pos.y);
          totalDist += dist;
          outfile << dist << "\n";
        }
      }
    }
    else
    {
      dist = sqrt((trueCx - pos.x)*(trueCx - pos.x) + (trueCy - pos.y)*(trueCy - pos.y));
      if (dist < MINRAD)
      {
        box->insidePoly = true;
        ++numCorrect;
        if (perfFile != "")
          outfile << 0 << "\n";
      }
      else
      {
        box->insidePoly = false;
        if (perfFile != "")
        {
          totalDist += dist;
          outfile << dist << "\n";
        }
      }
    }
  }
  // Capture the success so we can write it out later if need be
  success = numCorrect / boxes.size();

  if (perfFile != "")
  {
    outfile << "!\n";
    outfile << "TotalInsideGoal: " << numCorrect << "/" << boxes.size() << "\n";
    outfile << "Percentage: " << success << "\n";
    outfile << "TotalAverageDistance: " << totalDist / boxes.size() << "\n";
    outfile << "OutsideAverageDistance: " << totalDist / (boxes.size() - numCorrect) << "\n";
    outfile << "!\n";
    outfile << "$\n";
  }
  return success;
}

void World::recenterGoals(std::vector<Goal*>& tempGoals)
{
  double ldx = sqrt(numLights)/width/2.0;
  double ldy = sqrt(numLights)/height/2.0;
  double trueCx = (width / 2.0) + ldx;
  double trueCy = (height / 2.0) + ldy;

  // Find the center of the goal centers
  double bbmaxx = -1 * std::numeric_limits<double>::infinity();
  double bbmaxy = -1 * std::numeric_limits<double>::infinity();
  double bbminx = std::numeric_limits<double>::infinity();
  double bbminy = std::numeric_limits<double>::infinity();

  double size = 0;
  for (auto &g: tempGoals)
  {
    if (g->x > bbmaxx)
      bbmaxx = g->x;
    if (g->y > bbmaxy)
      bbmaxy = g->y;
    if (g->x < bbminx)
      bbminx = g->x;
    if (g->y < bbminy)
      bbminy = g->y;
  }

  // Adjust to match the center of contraction for the lights
  double dx = trueCx - (bbmaxx+bbminx)/2.0;
  double dy = trueCy - (bbmaxy+bbminy)/2.0;
  for (auto &g: tempGoals)
  {
    g->x += dx;
    g->y += dy;
    g->body->SetTransform(b2Vec2(g->x, g->y), 0);
  }
}

void World::centerGoalPolygonAgainstLights()
{
  double ldx = sqrt(numLights)/width/2.0;
  double ldy = sqrt(numLights)/height/2.0;
  double trueCx = (width / 2.0) + ldx;
  double trueCy = (height / 2.0) + ldy;

  // Adjust to match the center of contraction for the lights
  Vertex center = goalPolygon->getCentroid();
  double dx = trueCx - center.x;
  double dy = trueCy - center.y;

  // Also recenter the goalPolygon 
  goalPolygon->translate(dx, dy, true);
}

template<typename T>
void World::getBoundingBox(std::vector<T> vector, double& bbmaxx, double& bbmaxy, double& bbminx, double& bbminy, double radius)
{
  // Note that this requires a pretty specific format of vector
  if (havePolygon)
  {
    for (auto v : vector)
    {
      if (v.x > bbmaxx)
        bbmaxx = v.x;
      if (v.y > bbmaxy)
        bbmaxy = v.y;
      if (v.x < bbminx)
        bbminx = v.x;
      if (v.y < bbminy)
        bbminy = v.y;
    }
  }
  else
  {
    bbmaxx = width/2.0 + radius;
    bbmaxy = height/2.0 + radius;
    bbminx = width/2.0 - radius;
    bbminy = height/2.0 - radius;
  }
}

// Turn all goals off
// Used to reset on check
void World::unfulfillGoals()
{
  for (auto &col: goals)
  {
    for (auto &row : col)
    {
      for (auto &g : row)
      {
        g->fulfilled = false;
      }
    }
  }
}