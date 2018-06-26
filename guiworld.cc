#include <iostream>

#include "push.hh"

const double c_yellow[3] = {1.0, 1.0, 0.0};
const double c_red[3] = {1.0, 0.0, 0.0};
const double c_darkred[3] = {0.8, 0.0, 0.0};
const double c_tan[3] = {0.8, 0.6, 0.5};
const double c_gray[3] = {0.9, 0.9, 1.0};
const double c_royalblue[3] = {0.20, 0.55, 0.90};

bool World::paused = false;
bool GuiWorld::step = false;
int GuiWorld::skip = 10;

void DrawDisk(double cx, double cy, double r, const double color[3]);

double RTOD(double rad)
{
	return rad * 180 / M_PI;
}

// void checkmouse( GLFWwindow* win, double x, double y)
// {
//   //std::cout << x << ' ' << y << std::endl;
//   mousex = x/10.0;
//   mousey = -y/10.0;
// }

void key_callback(GLFWwindow *window,
				  int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS)
		switch (key)
		{
		case GLFW_KEY_SPACE:
			GuiWorld::paused = !GuiWorld::paused;
			break;

		case GLFW_KEY_S:
			GuiWorld::paused = true;
			GuiWorld::step = true; //!GuiWorld::step;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			if (mods & GLFW_MOD_SHIFT)
				GuiWorld::skip = 0;
			else
				GuiWorld::skip = std::max(0, --GuiWorld::skip);
			break;
		case GLFW_KEY_RIGHT_BRACKET:
			if (mods & GLFW_MOD_SHIFT)
				GuiWorld::skip = 500;
			else
				GuiWorld::skip++;
			break;
		default:
			break;
		}
}

void DrawBody(b2Body *b, const double color[3], double size)
{
	for (b2Fixture *f = b->GetFixtureList(); f; f = f->GetNext())
	{
		switch (f->GetType())
		{
		case b2Shape::e_circle:
		{
			b2CircleShape *circle = (b2CircleShape *)f->GetShape();

			b2Vec2 pos = b->GetPosition();
			// The size was hardcoded all along
			DrawDisk(pos.x, pos.y, size / 2.0, color);
		}
		break;
		case b2Shape::e_polygon:
		{
			b2PolygonShape *poly = (b2PolygonShape *)f->GetShape();

			const int count = poly->GetVertexCount();

			glColor3dv(color);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBegin(GL_POLYGON);

			for (int i = 0; i < count; i++)
			{
				const b2Vec2 w = b->GetWorldPoint(poly->GetVertex(i));
				glVertex2f(w.x, w.y);
			}
			glEnd();

			glLineWidth(2.0);
			glColor3f(color[0] / 5, color[1] / 5, color[2] / 5);
			//glColor3dv( color );
			//glColor3f( 0,0,0 );
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glBegin(GL_POLYGON);

			for (int i = 0; i < count; i++)
			{
				const b2Vec2 &v = poly->GetVertex(i);
				const b2Vec2 w = b->GetWorldPoint(v);
				glVertex2f(w.x, w.y);
			}
			glEnd();
		}
		break;
		default:
			break;
		}
	}
}

void DrawDisk(double cx, double cy, double r, const double color[3])
{
	const int num_segments = 32.0 * sqrtf(r);

	const double theta = 2 * M_PI / double(num_segments);
	const double c = cosf(theta); //precalculate the sine and cosine
	const double s = sinf(theta);
	double t;

	double x = r; //we start at angle = 0
	double y = 0;
	if (color != NULL)
	{
		glColor3dv(color);
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLE_STRIP);
	for (int ii = 0; ii < num_segments; ii++)
	{
		if (color != NULL)
			glColor3f(color[0] / 5, color[1] / 5, color[2] / 5);
		glVertex2f(x + cx, y + cy);
		if (color != NULL)
			glColor3dv(color);
		glVertex2f(cx, cy);

		//apply the rotation matrix
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}

	glBegin(GL_POLYGON);
	if (color != NULL)
		glColor3f(color[0] / 5, color[1] / 5, color[2] / 5);
	glVertex2f(r + cx, 0 + cy); // first point again to close disk

	glEnd();
}

GuiWorld::GuiWorld(double width, double height, int drawinterval, int numLights) : World(width, height, numLights, draw_interval),
																	window(NULL),
																	lights_need_redraw(true)
{
	srand48(time(NULL));
	skip = drawinterval;

	/* Initialize the gui library */
	if (!glfwInit())
	{
		std::cout << "Failed glfwInit()" << std::endl;
		exit(1);
	}

	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(800, 800, "S3", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(2);
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// scale the drawing to fit the whole world in the window, origin
	// at bottom left
	glScalef(2.0 / width, 2.0 / height, 1.0);
	glTranslatef(-width / 2.0, -height / 2.0, 0);

	// get mouse/pointer events
	//glfwSetCursorPosCallback( window, checkmouse );

	// get key events
	glfwSetKeyCallback(window, key_callback);
}

void GuiWorld::Step(double timestep)
{
	if (!paused || step)
	{
		World::Step(timestep);

		step = false;
		// paused = true;
	}

	if (--draw_interval < 1)
	{
		draw_interval = skip;

		glClearColor(0.8, 0.8, 0.8, 1.0);
		glClear(GL_COLOR_BUFFER_BIT);

		// draw the floor
		int count = 0;
		glColor3f(0.7, 0.7, 0.7);
		for (double i = 0; i < width; ++i)
		{
			for (double j = 0; j < height; ++j)
			{
				//if( count++ % 2 == 0) // if i + j is even
				if ((int(i) + int(j)) % 2 == 0)
					glRectf(i, j, i + 1, j + 1); // draw the rectangle
			}
		}

		// Testing the centers
		// bounding box
		double bbmaxx = -1 * std::numeric_limits<double>::infinity();
		double bbmaxy = -1 * std::numeric_limits<double>::infinity();
		double bbminx = std::numeric_limits<double>::infinity();
		double bbminy = std::numeric_limits<double>::infinity();

		// draw the goals
		int once = 0;
		double size = 0;
		for (auto &g : goals)
		{
			size = g->size;
			DrawBody(g->body, c_royalblue, g->size);
			if (g->x > bbmaxx)
			bbmaxx = g->x;
			if (g->y > bbmaxy)
			bbmaxy = g->y;
			if (g->x < bbminx)
			bbminx = g->x;
			if (g->y < bbminy)
			bbminy = g->y;
		}

		b2CircleShape staticCircle;
		staticCircle.m_p.Set(0.0f, 0.0f);         //position, relative to body position
    	staticCircle.m_radius = 0.6 / 2.0; //radius
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &staticCircle;	
		
		fixtureDef.density = 0;
		fixtureDef.friction = 0;
		fixtureDef.restitution = 0;

		fixtureDef.filter.categoryBits = GOAL;
		fixtureDef.filter.maskBits = 0; //everything
		// BOX | ROBOT | BOXBOUNDARY | ROBOTBOUNDARY; // everything

		b2BodyDef bodyDef;
		bodyDef.type = b2_staticBody;

		b2Body* body = b2world->CreateBody(&bodyDef);
		body->SetTransform(b2Vec2((bbminx + bbmaxx)/2.0, (bbminy + bbmaxy)/2.0), 0);

		body->CreateFixture(&fixtureDef);

		// draw the walls
		for (int i = 0; i < 4; i++)
			DrawBody(boxWall[i], c_gray, -1);

		for (int i = 0; i < 4; i++)
			DrawBody(robotWall[i], c_gray, -1);

		// draw the boxes
		for (auto &b : boxes)
			DrawBody(b->body, c_gray, b->size);

		// Draw the robots
		for (auto &r : robots)
		{
			double col[3];
			col[0] = (r->charge_max - r->charge) / r->charge_max;
			col[1] = r->charge / r->charge_max;
			col[2] = 0;

			DrawBody(r->body, col, r->size);
		}

		DrawBody(body, c_red, size);
		printf("The goalcenter is %f, %f\n", (bbminx + bbmaxx)/2.0, (bbminy + bbmaxy)/2.0);

		// draw a nose on the robot
		glColor3f(1, 1, 1);
		glPointSize(12);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_TRIANGLES);

		for (auto &r : robots)
		{
			const b2Transform &t = r->body->GetTransform();
			const double a = t.q.GetAngle();

			glVertex2f(t.p.x + r->size / 2.0 * cos(a),
					   t.p.y + r->size / 2.0 * sin(a));
			glVertex2f(t.p.x + r->size / 3.0 * cos(a + 0.5),
					   t.p.y + r->size / 3.0 * sin(a + 0.5));
			glVertex2f(t.p.x + r->size / 3.0 * cos(a - 0.5),
					   t.p.y + r->size / 3.0 * sin(a - 0.5));
		}
		glEnd();

		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(width, 0);
		glVertex2f(0, 0);
		glVertex2f(0, height);
		glEnd();

		const size_t side = 64;
		const double dx = width / (double)side;
		const double dy = height / (double)side;

		if (lights_need_redraw)
		{
			lights_need_redraw = false;

			// draw grid of light intensity
			//double bright[side][side];

			bright.resize(side * side);

			double max = 0;
			for (int y = 0; y < side; y++)
			{
				for (int x = 0; x < side; x++)
				{
					// find the world position at this grid location
					double wx = x * dx + dx / 2.0;
					double wy = y * dy + dy / 2.0;

					bright[y * side + x] = GetLightIntensityAt(wx, wy);

					// keep track of the max for normalizatioon
					if (bright[y * side + x] > max)
						max = bright[y * side + x];
				}
			}

			// scale to normalize brightness
			for (int y = 0; y < side; y++)
				for (int x = 0; x < side; x++)
					bright[y * side + x] /= (1.5 * max); // actually a little less than full alpha
		}

		// Find the light center
		bbmaxx = -1 * std::numeric_limits<double>::infinity();
		bbmaxy = -1 * std::numeric_limits<double>::infinity();
		bbminx = std::numeric_limits<double>::infinity();
		bbminy = std::numeric_limits<double>::infinity();

		// draw the light sources
		for (const auto &l : lights)
		{
			if (l->x > bbmaxx)
			bbmaxx = l->x;
			if (l->y > bbmaxy)
			bbmaxy = l->y;
			if (l->x < bbminx)
			bbminx = l->x;
			if (l->y < bbminy)
			bbminy = l->y;
			glColor4f(1, 1, 0, l->intensity);
			DrawDisk(l->x, l->y, 0.05, NULL);
		}

		b2CircleShape lightCircle;
		lightCircle.m_p.Set(0.0f, 0.0f);         //position, relative to body position
    	lightCircle.m_radius = 0.6 / 2.0; //radius
		b2FixtureDef fixtureDeflight;
		fixtureDeflight.shape = &lightCircle;	
		
		fixtureDeflight.density = 0;
		fixtureDeflight.friction = 0;
		fixtureDeflight.restitution = 0;

		fixtureDeflight.filter.categoryBits = GOAL;
		fixtureDeflight.filter.maskBits = 0; //everything
		// BOX | ROBOT | BOXBOUNDARY | ROBOTBOUNDARY; // everything

		b2BodyDef bodyDeflight;
		bodyDeflight.type = b2_staticBody;

		b2Body* lightbody = b2world->CreateBody(&bodyDef);
		lightbody->SetTransform(b2Vec2((bbminx + bbmaxx)/2.0 + 0.5, (bbminy + bbmaxy)/2.0 + 0.5), 0);

		lightbody->CreateFixture(&fixtureDeflight);
		DrawBody(lightbody, c_yellow, size);

		printf("The lightcenter is %f, %f\n", (bbminx + bbmaxx)/2.0 + 0.5, (bbminy + bbmaxy)/2.0 + 0.5);

		for (int y = 0; y < side; y++)
			for (int x = 0; x < side; x++)
			{
				// find the world position at this grid location
				double wx = x * dx + dx / 2.0;
				double wy = y * dy + dy / 2.0;

				glColor4f(1, 1, 0, bright[y * side + x]);

				glRectf(wx - dx / 2.0, wy - dy / 2.0,
						wx + dx / 2.0, wy + dy / 2.0);
			}

		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();
	}
}

GuiWorld::~GuiWorld(void)
{
	glfwTerminate();
}

bool GuiWorld::RequestShutdown(void)
{
	return glfwWindowShouldClose(window);
}
