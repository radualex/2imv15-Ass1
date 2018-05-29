// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "SpringForce.h"
#include "GravityForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "imageio.h"
#include "Constraint.h"
#include "System.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;

/* macros */

System* sys = new System();

/* external definitions (from solver) */
extern void simulation_step(System* sys, float dt, int solver);

/* global variables */


static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	sys->addParticle(new Particle(center + offset, 10.0f));
	sys->addParticle(new Particle(center + offset + offset, 3.0f));
	sys->addParticle(new Particle(center + offset + offset + offset, 120.0f));

	sys->addForce(new SpringForce(sys->pVector, 0, 1, dist, 1.0, 1.0));
	sys->addForce(new SpringForce(sys->pVector, 1, 2, dist, 1.0, 1.0));
	sys->addForce(new SpringForce(sys->pVector, 0, 2, dist, 1.0, 1.0));
	sys->addForce(new GravityForce(sys->pVector, Vec2f(0, -9.81f))); //apply gravity to all particles

	sys->addConstraint(new RodConstraint(sys->pVector[1], sys->pVector[2], dist));
	sys->addConstraint(new CircularWireConstraint(sys->pVector[0], center, dist));
}


/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	// Write frames if necessary.
	if (dump_frames)
	{
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0)
		{
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char *buffer = (unsigned char *)malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);

			free(buffer);
		}
	}
	frame_number++;

	glutSwapBuffers();
}

static void draw_particles(void)
{
	int size = sys->pVector.size();

	for (int ii = 0; ii < size; ii++)
	{
		sys->pVector[ii]->draw();
	}
}

static void draw_forces(void)
{
	for (Force *f : sys->fVector)
	{
		f->draw();
	}
}

static void draw_constraints(void)
{
	for (Constraint *c : sys->cVector)
	{
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

Particle *springParticle;
static void get_from_UI()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0] && !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx / (float)win_x) * N);
	j = (int)(((win_y - my) / (float)win_y) * N);

	if (i < 1 || i > N || j < 1 || j > N)
		return;

	if (mouse_down[0])
	{
	}

	if (mouse_down[2])
	{
		//std::cout <<"Mouse Right Down";
	}

	hi = (int)((hmx / (float)win_x) * N);
	hj = (int)(((win_y - hmy) / (float)win_y) * N);

	if (mouse_release[0])
	{
		//std::cout <<"Mouse release";
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = sys->pVector.size();
	for (ii = 0; ii < size; ii++)
	{
		sys->pVector[ii]->m_Position[0] = sys->pVector[ii]->m_ConstructPos[0];
		sys->pVector[ii]->m_Position[1] = sys->pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'c':
	case 'C':
		sys->clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		sys->free_data();
		exit(0);
		break;
	case ' ':
		dsim = !dsim;
		break;
	
	/*
	case '1':
		printf("Using Explicit Euler\n");
		sys->solver = new Euler(Euler::EXPLICIT);
		break;
	case '2':
		printf("Using Semi Explicit Euler\n");
		sys->solver = new Euler(Euler::SEMI);
		break;
	case '3':
		printf("Using Implicit Euler\n");
		sys->solver = new Euler(Euler::IMPLICIT);
		break;
	case '4':
		printf("Using Midpoint\n");
		sys->solver = new Midpoint();
		break;
	case '5':
		printf("Using Runge-Kutta\n");
		sys->solver = new RungeKutta();
		break;
		
	*/
	}
}

static double *getObjectPositionFromScreenCoords(int mx, int my)
{
	GLdouble modelMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	GLdouble projectionMatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
	GLint viewMatrix[4];
	glGetIntegerv(GL_VIEWPORT, viewMatrix);

	double *position = new double[3];

	gluUnProject(mx, my, 0, modelMatrix, projectionMatrix, viewMatrix,
				 &position[0], &position[1], &position[2]);

	return position;
}

static Particle *getClosestParticle(int x, int y)
{
	GLdouble modelMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	GLdouble projectionMatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
	GLint viewMatrix[4];
	glGetIntegerv(GL_VIEWPORT, viewMatrix);

	double closestDistance = 10000000;
	Particle *closestParticle;
	for (int i = 0; i < sys->pVector.size(); i++)
	{
		Vec2f position = sys->pVector[i]->m_Position;
		double screenCoordinates[3];
		gluProject(position[0], position[1], 0, modelMatrix, projectionMatrix, viewMatrix,
				   &screenCoordinates[0], &screenCoordinates[1], &screenCoordinates[2]);
		double distance = abs(x - screenCoordinates[0]) + abs(y - (win_y - screenCoordinates[1]));
		if (distance < closestDistance)
		{
			closestDistance = distance;
			closestParticle = sys->pVector[i];
		}
	}
	return closestParticle;
}

static void mouse_func(int button, int state, int x, int y)
{
	omx = mx = x;
	omx = my = y;

	if (!mouse_down[0])
	{
		hmx = x;
		hmy = y;
	}
	if (mouse_down[button])
		mouse_release[button] = state == GLUT_UP;
	if (mouse_down[button])
		mouse_shiftclick[button] = glutGetModifiers() == GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;

	if (state == GLUT_DOWN)
	{
		double *position = getObjectPositionFromScreenCoords(mx, my);
		Particle *closestParticle = getClosestParticle(mx, my);

		const Vec2f mousePos(position[0], -position[1]);

		springParticle = new Particle(mousePos, 0.0f);
		sys->pVector.push_back(springParticle);

		const int positionClosesetPart = sys->getPositionOfParticle(closestParticle);
		const int positionStringParticle = sys->getPositionOfParticle(springParticle);

		sys->fVector.push_back(new SpringForce(sys->pVector, positionClosesetPart, positionStringParticle, 0.2, 1.0, 1.0));
	}
	else if (state == GLUT_UP)
	{
		sys->fVector.pop_back();
		sys->pVector.pop_back();
	}
}

static void motion_func(int x, int y)
{
	mx = x;
	my = y;

	double *position = getObjectPositionFromScreenCoords(mx, my);
	const Vec2f mousePos(position[0], -position[1]);
	springParticle->m_Position = mousePos;
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	if (dsim)
	{
		// change the number
		simulation_step(sys, dt, 1);
	}

	else
	{
		get_from_UI();
		remap_GUI();
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func(void)
{
	pre_display();

	draw_forces();
	draw_constraints();
	draw_particles();

	post_display();
	// std::cout << pVector[0]->m_Position[0] << pVector[0]->m_Position[1] << "|";
}

/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Particletoys!");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	if (argc == 1)
	{
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, dt, d);
	}
	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	init_system();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}