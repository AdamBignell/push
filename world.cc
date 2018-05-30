#include "push.hh"
#include <fstream>

World::World(double width, double height, double numLights) : steps(0),
                                                              width(width),
                                                              height(height),
                                                              numLights(numLights),
                                                              b2world(new b2World(b2Vec2(0, 0))), // gravity
                                                              lights()                            //empty vector
{
  havePolygon = false;
  
  //set interior box container
  b2BodyDef boxWallDef;
  b2PolygonShape groundBox;
  groundBox.SetAsBox(width / 4.0, 0.01f);

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

  boxWall[0]->SetTransform(b2Vec2(width / 2, height / 4.0), 0);
  boxWall[1]->SetTransform(b2Vec2(width / 2, height - height / 4.0), 0);
  boxWall[2]->SetTransform(b2Vec2(width / 4.0, height / 2), M_PI / 2.0);
  boxWall[3]->SetTransform(b2Vec2(width - width / 4.0, height - height / 2.0), M_PI / 2.0);

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
                         intensity));
}

void World::AddRobot(Robot *r)
{
  robots.push_back(r);
}

void World::AddBox(Box *b)
{
  boxes.push_back(b);
}

void World::SetLightIntensity(size_t index, double intensity)
{
  if (index < lights.size())
    lights[index]->intensity = intensity;
}

void World::UpdateLightPattern(double goalx, double goaly, double probOn, double radius, double PATTWIDTH)
{
  double lside = sqrt(numLights); // Questionable
  double lx = width / lside;
  double ly = height / lside;
  double r2 = radius * radius;
  double randOn, cx, cy, c, c2;
  for (int x = 0; x < lside; x++)
    for (int y = 0; y < lside; y++)
    {
      int on = 0;
      if (havePolygon) // Use the polygon
      {
        on = (fabs(polygon.getDistFromPoint(x, y) < fmax(fmax(lx,ly)/2,PATTWIDTH)));
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
double World::GetRadMin(double numBoxes, double boxArea, double robot_size, Polygon tempPoly)
{
  //TODO: This still feels buggy for user-defined polygons
  double totalBoxArea = numBoxes * boxArea;
  double testScale = 0.95;
  if (havePolygon)
  {
    double area = tempPoly.getArea();
    // Can we do this loop algebraically?
    // Stop one iteration early to account for
    // the bodies of the robots themselves
    while (area * testScale > totalBoxArea)
    {
      tempPoly.scale(testScale);
      area *= testScale*testScale; // Area grows by the square of the scaling
    }
    return tempPoly.getDistFromPoint(tempPoly.cx,tempPoly.cy);
  }

  //Circle case
  double radMin = sqrt(totalBoxArea / M_PI) + robot_size; // Need literal wiggle room
  return radMin;
}

double World::GetSetRadMax(Polygon& tempPoly){
  //TODO: This still feels buggy for user-defined polygons
  if (havePolygon)
  {
    double arenaArea = width*height;
    double testScale = 1.05;
    double area = tempPoly.getArea();
    while (area < arenaArea) // Can we do this loop algebraically?
    {
      tempPoly.scale(testScale);
      area *= testScale*testScale;
    }
    return tempPoly.getDistFromPoint(tempPoly.cx,tempPoly.cy);
  }

  //Circle case
  double radMax = sqrt(width*height / M_PI);
  return radMax;
}

void World::saveWorldHeader(std::string saveFileName)
{
  std::ofstream outfile;
  outfile.open(saveFileName, std::ios_base::app);

  outfile << "HEADER:\n";
  outfile << "-w " << width << " -h " << height;
  outfile << " -r " << robots.size() << " -b " << boxes.size();
  outfile << " -z " << robots[0]->size << " -s " << boxes[0]->size;
  outfile << " -t " << robots[0]->cshape << " -y " << boxes[0]->cshape << '\n';
  outfile << "$\n";
}

// Saves the world state to a JSON file
void World::appendWorldStateToFile(std::string saveFileName)
{
  // Initial attempt using box2dJson
  // worldString = jsonWorld.writeToString(b2world);
  std::ofstream outfile;
  outfile.open(saveFileName, std::ios_base::app);

  b2Vec2 pose;
  // Write the robot information
  outfile << "ROBOTS:\n";
  for (auto bot : robots)
    {
        // x, y, a, charge
        pose = bot->body->GetPosition();
        outfile << pose.x << ' ' << pose.y << ' ' << bot->body->GetAngle()<< ' ' <<  bot->charge << '\n';
    }

  outfile << "!\n"; // Write a delimeter

  // Write the box information
  outfile << "BOXES:\n";
  for (auto box : boxes)
    {
        // x, y, a
        pose = box->body->GetPosition();
        outfile << pose.x << ' ' << pose.y << ' ' << box->body->GetAngle()<< '\n';
    }

  // Write the light information
  outfile << "LIGHTS:\n";
  for (auto light : lights)
    {
      if (light->intensity != 0)
      {
        // x, y, a, intensity
        outfile << light->x << ' ' << light->y << ' ' << light->z << ' ' << light->intensity << '\n';
      }
    }
    
  // outfile << worldString; // Write the world state
  outfile << "!\n"; // Write a delimeter
}
