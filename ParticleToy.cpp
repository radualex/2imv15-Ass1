// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "SpringForce.h"
#include "AngularSpringForce.h"
#include "GravityForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "imageio.h"
#include "Constraint.h"
#include "Cloth.h"
#include "ParticleManipulator.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;

/* macros */

ParticleManipulator* pm = new ParticleManipulator();

/* external definitions (from solver) */
extern void simulation_step(ParticleManipulator* pm, float dt, int solver);

/* global variables */


static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
static int solverNr = 1;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void init_cloth(void)
{
	pm->clear_data();
	pm->free_data();
	auto cloth = new Cloth(10,10, pm->pVector, pm->fVector, pm->cVector);
}

static void init_basic(void)
{
	pm->clear_data();
	pm->free_data();
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.
	//pm->drawWall();
	pm->pVector.push_back(new Particle(center + offset, 0.05f));
	pm->pVector.push_back(new Particle(center + offset + offset, 0.05f));
	pm->pVector.push_back(new Particle(center + offset + offset + offset, 0.05f));

	pm->fVector.push_back(new SpringForce(pm->pVector, 0, 1, dist, 150.0, 1.50));
	pm->fVector.push_back(new SpringForce(pm->pVector, 1, 2, dist, 150.0, 1.50));
	pm->fVector.push_back(new SpringForce(pm->pVector, 0, 2, dist, 150.0, 1.50));
	pm->fVector.push_back(new GravityForce(pm->pVector, Vec2f(0, -9.81f))); //apply gravity to all particles

	pm->cVector.push_back(new RodConstraint(pm->pVector[1], pm->pVector[2], dist));
	pm->cVector.push_back(new RodConstraint(pm->pVector[0], pm->pVector[1], dist));

	pm->cVector.push_back(new CircularWireConstraint(pm->pVector[0], center, dist));
}

static void init_angular(void)
{
	pm->clear_data();
	pm->free_data();
	double dist = -0.2;
	const Vec2f start(0.0, 0.6);
	const Vec2f offset(0.0, dist);
	const Vec2f offset2(0.1,0.0);
	for(int i = 0; i < 8 ; i++){
		int k = 1;
		if(i%2 == 0)
		k = 1;
		else k = -1;
		pm->pVector.push_back(new Particle(start + i*offset + k*offset2, 10.0f));
	}
	for(int i = 0; i<6; i++){
		pm->fVector.push_back(new AngularSpringForce(pm->pVector, i, i+1, i+2, dist, 120.0, 100.0));
	}

	//pm->fVector.push_back(new GravityForce({pm->pVector[7]}, Vec2f(0, -9.81f)));

	//fVector.push_back(new AngularSpringForce(pVector, 0, 1, 2, dist, 1.0, 1.0));

}

static void init_system(void)
{
	init_basic();
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
	int size = pm->pVector.size();

	for (int ii = 0; ii < size; ii++)
	{
		pm->pVector[ii]->draw();
	}
}

static void draw_forces(void)
{
	for (Force *f : pm->fVector)
	{
		f->draw();
	}
}

static void draw_constraints(void)
{
	for (Constraint *c : pm->cVector)
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
	int ii, size = pm->pVector.size();
	for (ii = 0; ii < size; ii++)
	{
		pm->pVector[ii]->m_Position[0] = pm->pVector[ii]->m_ConstructPos[0];
		pm->pVector[ii]->m_Position[1] = pm->pVector[ii]->m_ConstructPos[1];
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
		pm->clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		pm->free_data();
		exit(0);
		break;
	case ' ':
		dsim = !dsim;
		break;
	case 'a':
	case 'A':
		//basic
		init_basic();

		glutMainLoop();

		exit(0);
		break;
	case 's':
	case 'S':
		//cloth
		init_cloth();

		glutMainLoop();

		exit(0);
		break;
	case 'w':
	case 'W':
		//angular spring
		init_angular();

		glutMainLoop();

		exit(0);
		break;
	case 'z':
	case 'Z':
	if(!pm->walls){
		pm->walls = true;
	}
	else
		pm->walls = false;
		break;

	case '1':
		solverNr = 1;
		std::cout<< "using explicit eulear"<<std::endl;
		break;
	case '2':
		solverNr = 2;
		std::cout<< "using midpoint"<<std::endl;
		break;
	case '3':
		solverNr = 3;
		std::cout<< "using rungekutta"<<std::endl;
		break;
	case '4':
		solverNr = 4;
		break;
		
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
	for (int i = 0; i < pm->pVector.size(); i++)
	{
		Vec2f position = pm->pVector[i]->m_Position;
		double screenCoordinates[3];
		gluProject(position[0], position[1], 0, modelMatrix, projectionMatrix, viewMatrix,
				   &screenCoordinates[0], &screenCoordinates[1], &screenCoordinates[2]);
		double distance = abs(x - screenCoordinates[0]) + abs(y - (win_y - screenCoordinates[1]));
		if (distance < closestDistance)
		{
			closestDistance = distance;
			closestParticle = pm->pVector[i];
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

		springParticle = new Particle(mousePos, 0.01f);
	
		pm->pVector.push_back(springParticle);

		const int positionClosesetPart = pm->getPositionOfParticle(closestParticle);
		const int positionStringParticle = pm->getPositionOfParticle(springParticle);
		pm->fVector.push_back(new SpringForce(pm->pVector, positionClosesetPart, positionStringParticle, 0.2, 150.0, 1.5));
	}
	else if (state == GLUT_UP)
	{
		pm->fVector.pop_back();
		pm->pVector.pop_back();
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
		//solver based on solverNr(default 1).
		simulation_step(pm, dt, solverNr);
	}

	else
	{
		get_from_UI();
		remap_GUI();
		pm->clear_data();
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
	pm->drawWalls();
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
		dt = 0.0005f;
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
	printf("\t Press 'a' to show Default example\n");
	printf("\t Press 's' to show Cloth example\n");
	printf("\t Press 'w' to show Angular springs example\n");
	printf("\t Press 'z' to toggle between resolving collision and not. USe for cloth");
	printf("\t Press '1' to apply Explicit Euler(Default)\n");
	printf("\t Press '2' to apply Mid-point\n");
	printf("\t Press '3' to apply Runge-kutta\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	solverNr = 1;

	init_system();

	win_x = 800;
	win_y = 800;
	open_glut_window();

	glutMainLoop();

	exit(0);
}