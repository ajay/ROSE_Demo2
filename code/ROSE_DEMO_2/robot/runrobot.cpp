#include <armadillo>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include "SDL/SDL.h"

#include "astar.h"
#include "chili_landmarks.h"
#include "draw.h"
#include "ipcdb.h"
#include "mathfun.h"
#include "pfilter.h"
#include "Rose.h"
#include "sdldef.h"
#include "sim_landmark.h"
#include "sim_map.h"
#include "sim_robot.h"
#include "dbconntwo.h"

using namespace arma;
using namespace std;

// threads in this file
void manual_input(void);
void chilitag_detect(void);
void localize_pose(void);
void robot_calcmotion(void);
void motion_plan(void);
void display_interface(void);

static dbconn db;

static void database_update(void)
{
	db.db_update();
}

static string state = "waiting";

static double secdiff(struct timeval &t1, struct timeval &t2);

static void stoprunning(int signum)
{
	stopsig = 1;
	rose.startStop = true;
	alarm(1); // hack to get this to force quit
}

static void forcequit(int signum)
{
	printf("Force quitting...\n");
	exit(1);
}

static SDL_Surface *initSDL(int w, int h)
{
	if (SDL_Init(SDL_INIT_EVERYTHING) == -1)
	{
		return NULL;
	}

	SDL_Surface *screen = SDL_SetVideoMode(w,h, 32, SDL_SWSURFACE);
	if (screen == NULL)
	{
		return NULL;
	}

	SDL_WM_SetCaption("rose", NULL);

	return screen;
}

static void screenblit(SDL_Surface *s, cube &frame)
{
	for (int i = 0; i < (int)frame.n_rows; i++)
	{
		for (int j = 0; j < (int)frame.n_cols; j++)
		{
			uint32_t color =
				((uint8_t)((int)round(frame(i,j,0) * 255)) << 16) |
				((uint8_t)((int)round(frame(i,j,1) * 255)) << 8) |
				((uint8_t)((int)round(frame(i,j,2) * 255)) << 0);
			((uint32_t *)s->pixels)[XY2P(j, i, s->w, s->h)] = color;
		}
	}
}

static void chilicamdetect_thread(void)
{
	chili.update();
}

static void print_data(void)
{
	while (!stopsig)
	{
		// cout << "autonomous_enabled:\t" << auto_enable << endl;
		// cout << "state:\t\t\t" << state << endl;
		usleep(100000);
	}
}

int main()
{
	// preemptive init
	printf("[main] preemptive init\n");
	signal(SIGINT, stoprunning);
	signal(SIGALRM, forcequit);
	screen = initSDL(500, 500);

	if (!screen)
	{
		printf("No screen found, please check your SDL configurations\n");
		return 1;
	}

	double initial_x = 75;
	double initial_y = 60;
	double initial_t = 90;
	robot_pose = vec({ initial_x, initial_y, initial_t });
	globalmap.load("ece_hallway_partial.jpg"); // lower corner is (0,0)

	// start up the threads
	printf("[main] start up the threads\n");
	rose.startStop = false;
	thread manual_thread(manual_input);
	thread chilicamthread(chilicamdetect_thread);
	thread chili_thread(chilitag_detect);
	thread pose_thread(localize_pose);
	thread path_thread(motion_plan);
	thread robot_thread(robot_calcmotion);
	thread display_thread(display_interface);
	thread print_data_thread(print_data);
	thread db_update_thread(database_update);

	// wait until program closes
	printf("[main] all threads started\n");
	while (!stopsig);
	alarm(1);

	// close all threads
	printf("[main] close all threads\n");
	rose.set_wheels(0, 0, 0, 0);
	rose.stop_arm();
	rose.startStop = true;
	rose.disconnect();
	chili_thread.join();
	chilicamthread.join();
	pose_thread.join();
	path_thread.join();
	robot_thread.join();
	display_thread.join();
	print_data_thread.join();
	db_update_thread.join();
	SDL_Quit();

	printf("Closed successfully.\n");
	return 0;
}

static bool arm_enabled = false;

static vec garm_init ({ 0, -15, 90, 20, 0, 0 });
static vec grab_pos	 ({ 0, 23, 114, -58, 0, 0 });
static vec rest_pos	 ({ 0, -21, 108, 88, 0, 70 });
static vec gray_stool_grab_pos ({ 0, 7, 16, 70, 0, 0 });
static vec low_stool_grab_pos ({0, 15, 6, 70, 0, 0});
static vec garm = garm_init;

void manual_input(void)
{
	struct timeval starttime;
	gettimeofday(&starttime, NULL);
	while (!stopsig)
	{
		SDL_PumpEvents();
		Uint8 *keystates = SDL_GetKeyState(NULL);

		// detect for exit
		if (keystates[SDLK_x])
		{
			auto_enable = false;
			auto_confirmed = false;
			manual_confirmed = true;
			rose.set_wheels(0, 0, 0, 0);
			rose.stop_arm();
			rose.startStop = true;
			kill(getpid(), SIGINT);
			continue;
		}

		// detect for autonomous enable or disable
		if (keystates[SDLK_m])
		{
			autonomous_lock.lock();
			auto_enable = true;
			auto_confirmed = false;
			manual_confirmed = true;
			autonomous_lock.unlock();
		}
		else if (keystates[SDLK_n])
		{
			autonomous_lock.lock();
			auto_enable = false;//true;
			auto_confirmed = false;
			manual_confirmed = true;
			autonomous_lock.unlock();
		}

		// input arm manual feedback
		struct timeval currtime;
		gettimeofday(&currtime, NULL);
		if (secdiff(starttime, currtime) > 0.05)
		{
			if (keystates[SDLK_p]) { garm(0) += 2; }
			if (keystates[SDLK_l]) { garm(0) -= 2; }
			if (keystates[SDLK_o]) { garm(1) += 2; }
			if (keystates[SDLK_k]) { garm(1) -= 2; }
			if (keystates[SDLK_i]) { garm(2) += 2; }
			if (keystates[SDLK_j]) { garm(2) -= 2; }
			if (keystates[SDLK_u]) { garm(3) += 2; }
			if (keystates[SDLK_h]) { garm(3) -= 2; }
			if (keystates[SDLK_y]) { garm(4) += 2; }
			if (keystates[SDLK_g]) { garm(4) -= 2; }
			if (keystates[SDLK_t]) { garm(5) += 10; }
			if (keystates[SDLK_f]) { garm(5) -= 10; }
			memcpy(&starttime, &currtime, sizeof(struct timeval));
		}

		// limit garm values
		for (int i = 0; i < garm.size(); i++)
		{
			garm(i) = limit_value(garm(i), rose.arm_mint(i), rose.arm_maxt(i));
		}

		if (keystates[SDLK_v])
		{
			arm_enabled = true;
		}
		else if (keystates[SDLK_b])
		{
			arm_enabled = false;
		}


		if 		(keystates[SDLK_0]) garm = rest_pos;
		else if (keystates[SDLK_1]) garm = garm_init;
		else if (keystates[SDLK_2]) garm = grab_pos;
		else if (keystates[SDLK_3]) garm = gray_stool_grab_pos;
		else if (keystates[SDLK_4]) garm = low_stool_grab_pos;

		// if not manual en, then continue
		if (auto_enable)
		{
			continue;
		}

		if (arm_enabled)
		{
			rose.set_arm(garm(0), garm(1), garm(2), garm(3), garm(4), garm(5));
			cout << garm << endl;
		}
		else
		{
			rose.stop_arm();
		}

		state = "chillin";

		double v = 1;

		// input manual feedback
		if 		(keystates[SDLK_q]) rose.set_wheels(-v,  v, -v,  v);
		else if (keystates[SDLK_e]) rose.set_wheels( v, -v,  v, -v);
		else if (keystates[SDLK_a]) rose.set_wheels(-v,  v,  v, -v);
		else if (keystates[SDLK_d]) rose.set_wheels( v, -v, -v,  v);
		else if (keystates[SDLK_s]) rose.set_wheels(-v, -v, -v, -v);
		else if (keystates[SDLK_w]) rose.set_wheels( v,  v,  v,  v);
		// else if (keystates[SDLK_v]) rose.set_wheels(v,0,0,0);
		// else if (keystates[SDLK_2]) rose.set_wheels(0,v,0,0);
		// else if (keystates[SDLK_3]) rose.set_wheels(0,0,v,0);
		// else if (keystates[SDLK_4]) rose.set_wheels(0,0,0,v);
		else if (keystates[SDLK_r]) rose.reset_encoders();
		else rose.set_wheels(0,0,0,0);

		// cout << rose.encoder[0] << " " << rose.encoder[1] << " " << rose.encoder[2] << " " << rose.encoder[3] << endl;

	}
}

void chilitag_detect(void)
{
	while (!stopsig)
	{
		// place the chilitags' positions into a matrix
		mat sv(3, 20, fill::zeros);
		for (int j = 0; j < 20; j++)
		{
			if (chili.tags[j][0] != 0.0)
			{
				sv.col(j) = vec({ chili.tags[j][1], chili.tags[j][2], chili.tags[j][0]/*, chili.tags[j][3] */ });
			}
		}

		for (int j = 0; j < 20; j++)
		{
			vec pt({ sv(0, j), sv(1, j) });
			sv(0, j) = eucdist(pt);
			sv(1, j) = angle(pt);
		}

		// store the matrix
		chili_lock.lock();
		chilitags = sv;
		chili_lock.unlock();
	}
}

void localize_pose(void)
{
	// create the landmarks (custom)
	landmarks.push_back(sim_landmark(8, 240-24));				// 00
	landmarks.push_back(sim_landmark(8, 480-24));				// 01
	landmarks.push_back(sim_landmark(8, 720-24));				// 02
	landmarks.push_back(sim_landmark(8, 960-24));				// 03
	landmarks.push_back(sim_landmark(8, 1200-24));				// 04
	landmarks.push_back(sim_landmark(146, 1200-24));			// 05
	landmarks.push_back(sim_landmark(146, 960-24));				// 06
	landmarks.push_back(sim_landmark(146, 720-24));				// 07
	landmarks.push_back(sim_landmark(146, 480-24));				// 08
	landmarks.push_back(sim_landmark(146, 240-24));				// 09
	landmarks.push_back(sim_landmark(8, 240));					// 10
	landmarks.push_back(sim_landmark(8, 480));					// 11
	landmarks.push_back(sim_landmark(8, 720));					// 12
	landmarks.push_back(sim_landmark(8, 960));					// 13
	landmarks.push_back(sim_landmark(8, 1200));					// 14
	landmarks.push_back(sim_landmark(146, 1200));				// 15
	landmarks.push_back(sim_landmark(146, 960));				// 16
	landmarks.push_back(sim_landmark(146, 720));				// 17
	landmarks.push_back(sim_landmark(146, 480));				// 18
	landmarks.push_back(sim_landmark(146, 240));				// 19

	// start the particle filter
	pose_lock.lock();
	int nparticles = 500;
	double initial_sigma = 100;
	double x = robot_pose(0);
	double y = robot_pose(1);
	double t = robot_pose(2);
	double vs = 0.1;
	double ws = 0.2;
	pf = pfilter(nparticles, &globalmap, landmarks, x, y, t, initial_sigma);
	pf.set_noise(vs, ws);
	pose_lock.unlock();

	// loop on the particle filter for updates on the location
	vec mu;
	mat sigma;
	while (!stopsig)
	{
		// move the robot
		pf.move(rose.recv());

		// get the chilitags
		chili_lock.lock();
		mat landmarks = chilitags;
		chili_lock.unlock();

		// observe and predict the robot's new location
		pf.observe(landmarks);
		pf.predict(mu, sigma);

		// store the new location
		pose_lock.lock();
		robot_pose = mu;
		pose_lock.unlock();
	}
}

void robot_calcmotion(void)
{
	double rotate_vel = 0.2;
	double approach_vel = 0.35;

	int coke_int = 4;
	int pepsi_int = 3;
	int table_zero_int = 7;
	int table_one_int = 1;

	int kitchen_chilitag = 0;
	int table_zero_chilitag = table_zero_int;
	int table_one_chilitag = table_one_int;
	int first_can_chilitag = coke_int;
	int second_can_chilitag = pepsi_int;

	double kitchen[] = {0, 0, 0};
	double first_can[] = {0, 0, 0};
	double second_can[] = {0, 0, 0};
	double table_zero[] = {0, 0, 0};
	double table_one[] = {0, 0, 0};

	struct timeval kitchen_detected;
	struct timeval first_can_detected;
	struct timeval table_zero_detected;
	struct timeval table_one_detected;
	struct timeval second_can_detected;
	struct timeval currtime;
	struct timeval start;

	gettimeofday(&start, NULL);
	static double STEP = 0;
/*

	//while (db.customer_order.parsed_items.size() == 0)
	//{
		cout << "Waiting for order..." << endl;
		usleep(1000000);
	//}

	cout << "Got an order: " << endl;
	string drink = db.customer_order.parsed_items[0][0];
	cout << "Drink: " << drink << endl;
	int table = db.customer_order.table;
	cout << "Table: " << table << endl;

	if (drink == "coke")
	{
		first_can_chilitag = coke_int;
		second_can_chilitag = pepsi_int;
	}
	else
	{
		first_can_chilitag = pepsi_int;
		second_can_chilitag = coke_int;
	}

	if (table == 0)
	{
		table_zero_chilitag = table_zero_int;
		table_one_chilitag = table_one_int;
	}
	else if (table == 1)
	{
		table_zero_chilitag = table_one_int;
		table_one_chilitag = table_zero_int;
	} */

	cout << "First can chilitag: " << first_can_chilitag << endl;
	cout << "Second can chilitag: " << second_can_chilitag << endl;
	cout << "First table chilitag: " << table_zero_chilitag << endl;
	cout << "Second table chilitag: " << table_one_chilitag << endl;

	for (int i = 10; i > 0; i--)
	{
		cout << "Launching robot in " << i << "..." << endl;
		usleep(1000000);
	}

// RUTGERS DAY HACK //
//////////////////////

  while (!stopsig)
  {
    autonomous_lock.lock();
    if (!auto_enable)
    {
      autonomous_lock.unlock();
      continue;
    }
    autonomous_lock.unlock();


    vec garm_init ({ 0, -15, 90, 20, 0, 0 });
    vec grab_pos	 ({ 0, 23, 114, -58, 0, 0 });
    vec rest_pos	 ({ 0, -21, 108, 88, 0, 70 });
    vec gray_stool_grab_pos ({ 0, 7, 16, 70, 0, 0 });
    vec low_stool_grab_pos ({0, 15, 6, 70, 0, 0});

    vec garm = garm_init;
    
    arm_enabled = true;

    usleep(20000000);
    rose.set_arm(garm(0), garm(1), garm(2), garm(3), garm(4), garm(5));

    garm = grab_pos;
    usleep(20000000);
    rose.set_arm(garm(0), garm(1), garm(2), garm(3), garm(4), garm(5));

    garm = gray_stool_grab_pos;
    usleep(20000000);
    rose.set_arm(garm(0), garm(1), garm(2), garm(3), garm(4), garm(5));


  }

  



	while (!stopsig)
	{
		cout << "STEP: " << STEP << endl;

		autonomous_lock.lock();
		if (!auto_enable)
		{
			autonomous_lock.unlock();
			continue;
		}
		autonomous_lock.unlock();
		state = "auton";

		if (chili.tags[first_can_chilitag][0])
		{
			first_can[0] = chili.tags[first_can_chilitag][1];
			first_can[1] = chili.tags[first_can_chilitag][3];
			first_can[2] = chili.tags[first_can_chilitag][2];

			// if (STEP == 2)
			// {
			cout << "first_can_chilitag location: \tx: " << first_can[0] << "\ty: " << first_can[1] << "\tz: " << first_can[2] << endl;
			// }
			gettimeofday(&first_can_detected, NULL);
		}

		if (chili.tags[table_zero_chilitag][0])
		{
			table_zero[0] = chili.tags[table_zero_chilitag][1];
			table_zero[1] = chili.tags[table_zero_chilitag][3];
			table_zero[2] = chili.tags[table_zero_chilitag][2];

			// if (STEP == 2)
			// {
			cout << "table_zero_chilitag location: \tx: " << table_zero[0] << "\ty: " << table_zero[1] << "\tz: " << table_zero[2] << endl;
			// }
			gettimeofday(&table_zero_detected, NULL);
		}

		if (chili.tags[table_one_chilitag][0])
		{
			table_one[0] = chili.tags[table_one_chilitag][1];
			table_one[1] = chili.tags[table_one_chilitag][3];
			table_one[2] = chili.tags[table_one_chilitag][2];

			// if (STEP == 2)
			// {
			cout << "table_one_chilitag location: \tx: " << table_one[0] << "\ty: " << table_one[1] << "\tz: " << table_one[2] << endl;
			// }
			gettimeofday(&table_one_detected, NULL);
		}

		if (chili.tags[second_can_chilitag][0])
		{
			second_can[0] = chili.tags[second_can_chilitag][1];
			second_can[1] = chili.tags[second_can_chilitag][3];
			second_can[2] = chili.tags[second_can_chilitag][2];

			// if (STEP == 2)
			// {
			cout << "table_zero_chilitag location: \tx: " << second_can[0] << "\ty: " << second_can[1] << "\tz: " << second_can[2] << endl;
			// }
			gettimeofday(&second_can_detected, NULL);
		}

		if (chili.tags[kitchen_chilitag][0])
		{
			kitchen[0] = chili.tags[kitchen_chilitag][1];
			kitchen[1] = chili.tags[kitchen_chilitag][3];
			kitchen[2] = chili.tags[kitchen_chilitag][2];
			// if (STEP == 0)
			// {
			cout << "kitchen_chilitag location: \tx: " << kitchen[0] << "\ty: " << kitchen[1] << "\tz: " << kitchen[2] << endl;
			// }
			gettimeofday(&kitchen_detected, NULL);
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP 0 --> Look for kitchen chilitag
		gettimeofday(&currtime, NULL);
		if ((STEP == 0) && (secdiff(kitchen_detected, currtime) < 0.05) /*&& (goeal_z > 58)*/)
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.01;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = 0.8 + k_p * kitchen[0];
			v_r = 0.8 - k_p * kitchen[0];

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 0)
		{
			rose.set_wheels(-rotate_vel, rotate_vel, -rotate_vel, rotate_vel);
		}

		if ((STEP == 0) && (chili.tags[kitchen_chilitag][0]) && (kitchen[2] <= 58))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 1;
			usleep(1000000); // 1 second
		}


		////////////////////////////////////////////////////////////////////////////////
		// STEP 1 --> Raise up arm
		if (STEP == 1)
		{
			arm_enabled = true;
			vec gray_stool_grab_pos ({ -2, 15, 6, 70, 0, 0 });
			rose.set_arm(gray_stool_grab_pos(0), gray_stool_grab_pos(1), gray_stool_grab_pos(2), gray_stool_grab_pos(3), gray_stool_grab_pos(4), gray_stool_grab_pos(5));
			// cout << "kitchen_chilitag location: \tx: " << kitchen_x << "\ty: " << kitchen_y << "\tz: " << kitchen_z << endl;
			STEP = 2;
			usleep(3000000); // 3 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 2 Move closer slowly
		gettimeofday(&currtime, NULL);
		if ((STEP == 2) && (secdiff(first_can_detected, currtime) < 0.05))
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.005;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = approach_vel + k_p * first_can[0];
			v_r = approach_vel - k_p * first_can[0];

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 2)
		{
			rose.set_wheels(0, 0, 0, 0);
		}

		if ((STEP == 2) && (secdiff(first_can_detected, currtime) < 0.05) && (chili.tags[first_can_chilitag][0]) && (first_can[2] <= 114))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 3;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 3 Close Claw
		gettimeofday(&currtime, NULL);
		if (STEP == 3)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 70, 0, 65 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 3.5;
			usleep(1000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 3.5 Lift Up a lil
		gettimeofday(&currtime, NULL);
		if (STEP == 3.5)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 65, 0, 65 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 4;
			usleep(1000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 4 Turn Around, go to customer
		gettimeofday(&currtime, NULL);
		if (STEP == 4)
		{
			double v = 0.5;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 5;
			usleep(3000000); // 3 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 5 STOP, DELAY
		gettimeofday(&currtime, NULL);
		if (STEP == 5)
		{
			double v = 0;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 6;
			usleep(1000000); // 1 second
		}


		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 6 go to table 0
		gettimeofday(&currtime, NULL);
		if ((STEP == 6) && (secdiff(table_zero_detected, currtime) < 0.05))
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.01;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = 0.8 + k_p * (table_zero[0] - 3);
			v_r = 0.8 - k_p * (table_zero[0] - 3);

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 6)
		{
			rose.set_wheels(-rotate_vel, rotate_vel, -rotate_vel, rotate_vel);
		}

		if ((STEP == 6) && (chili.tags[table_zero_chilitag][0]) && (table_zero[2] <= 43.5))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 7;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 7 Open Claw
		gettimeofday(&currtime, NULL);
		if (STEP == 7)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 68, 0, 0 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 8;
			usleep(2000000); // 2 seconds
		}



		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 8 Back off
		gettimeofday(&currtime, NULL);
		if (STEP == 8)
		{
			double v = 0.5;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 9;
			usleep(2000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 9 STOP, DELAY
		gettimeofday(&currtime, NULL);
		if (STEP == 9)
		{
			double v = 0;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 10;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 10, look for kitchen again
		gettimeofday(&currtime, NULL);
		if ((STEP == 10) && (secdiff(kitchen_detected, currtime) < 0.2) /*&& (goeal_z > 58)*/)
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.01;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = 0.8 + k_p * kitchen[0];
			v_r = 0.8 - k_p * kitchen[0];

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 10)
		{
			rose.set_wheels(rotate_vel, -rotate_vel, rotate_vel, -rotate_vel);
		}

		if ((STEP == 10) && (chili.tags[kitchen_chilitag][0]) && (kitchen[2] <= 58))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 11;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP 11 --> Set arm to correct again
		if (STEP == 11)
		{
			arm_enabled = true;
			vec gray_stool_grab_pos ({ -2, 15, 6, 70, 0, 0 });
			rose.set_arm(gray_stool_grab_pos(0), gray_stool_grab_pos(1), gray_stool_grab_pos(2), gray_stool_grab_pos(3), gray_stool_grab_pos(4), gray_stool_grab_pos(5));
			// cout << "kitchen_chilitag location: \tx: " << kitchen_x << "\ty: " << kitchen_y << "\tz: " << kitchen_z << endl;
			STEP = 12;
			usleep(2000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 12 Move closer slowly to second_can this time
		gettimeofday(&currtime, NULL);
		if ((STEP == 12) && (secdiff(second_can_detected, currtime) < 0.5))
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.005;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = approach_vel + k_p * second_can[0];
			v_r = approach_vel - k_p * second_can[0];

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 12)
		{
			rose.set_wheels(0, 0, 0, 0);
		}

		if ((STEP == 12) && (secdiff(second_can_detected, currtime) < 0.5) && (chili.tags[second_can_chilitag][0]) && (second_can[2] <= 114))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 13;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 13 Close Claw
		gettimeofday(&currtime, NULL);
		if (STEP == 13)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 69, 0, 65 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 13.5;
			usleep(2000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 13.5 Lift Up a lil
		gettimeofday(&currtime, NULL);
		if (STEP == 13.5)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 65, 0, 65 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 14;
			usleep(1000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 14 Turn Around,
		gettimeofday(&currtime, NULL);
		if (STEP == 14)
		{
			double v = 0.5;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 15;
			usleep(3000000); // 3 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 15 STOP, DELAY
		gettimeofday(&currtime, NULL);
		if (STEP == 15)
		{
			double v = 0;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 16;
			usleep(1000000); // 1 second
		}


		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 16 go to customer again, table 2 now
		gettimeofday(&currtime, NULL);
		if ((STEP == 16) && (secdiff(table_one_detected, currtime) < 0.2))
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.01;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = 0.8 + k_p * (table_one[0] - 5);
			v_r = 0.8 - k_p * (table_one[0] - 5);

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 16)
		{
			rose.set_wheels(-rotate_vel, rotate_vel, -rotate_vel, rotate_vel);
			if (chili.tags[table_one_chilitag][0])
			{
				usleep(100000); // 0.1 second
			}
		}

		if ((STEP == 16) && (chili.tags[table_one_chilitag][0]) && (table_one[2] <= 43.5))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 17;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 17 Open Claw
		gettimeofday(&currtime, NULL);
		if (STEP == 17)
		{
			vec gray_stool_grab_pos_CLOSED_CLAW ({ -2, 15, 6, 70, 0, 0 });
			rose.set_arm(gray_stool_grab_pos_CLOSED_CLAW(0), gray_stool_grab_pos_CLOSED_CLAW(1), gray_stool_grab_pos_CLOSED_CLAW(2), gray_stool_grab_pos_CLOSED_CLAW(3), gray_stool_grab_pos_CLOSED_CLAW(4), gray_stool_grab_pos_CLOSED_CLAW(5));

			STEP = 18;
			usleep(2000000); // 2 seconds
		}


		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 18 Back off
		gettimeofday(&currtime, NULL);
		if (STEP == 18)
		{
			double v = 0.5;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 19;
			usleep(2000000); // 2 seconds
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 19 STOP, DELAY
		gettimeofday(&currtime, NULL);
		if (STEP == 19)
		{
			double v = 0;

			rose.set_wheels(-v, -v, -v, -v);
			STEP = 20;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 20 ARM DOWN
		gettimeofday(&currtime, NULL);
		if (STEP == 20)
		{
			vec garm_init ({ 0, -15, 90, 20, 0, 0 });
			vec rest_pos ({ 0, -21, 108, 88, 0, 70 });

			rose.set_arm(garm_init(0), garm_init(1), garm_init(2), garm_init(3), garm_init(4), garm_init(5));
			usleep(2000000); // 2 seconds

			rose.set_arm(rest_pos(0), rest_pos(1), rest_pos(2), rest_pos(3), rest_pos(4), rest_pos(5));
			usleep(2000000); // 2 seconds

			STEP = 21;
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 21 Back to kitchen
		gettimeofday(&currtime, NULL);
		if ((STEP == 21) && (secdiff(kitchen_detected, currtime) < 0.2) /*&& (goeal_z > 58)*/)
		{
			double v_l = 0;
			double v_r = 0;

			double k_p = 0.01;
			double k_i = 0.01;
			double k_d = 0.01;

			v_l = 0.8 + k_p * kitchen[0];
			v_r = 0.8 - k_p * kitchen[0];

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 21)
		{
			rose.set_wheels(rotate_vel, -rotate_vel, rotate_vel, -rotate_vel);
		}

		if ((STEP == 21) && (chili.tags[kitchen_chilitag][0]) && (kitchen[2] <= 40))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 22;
			usleep(1000000); // 1 second
		}

		////////////////////////////////////////////////////////////////////////////////
		// STEP --> 22 spin until it sees table 0
		gettimeofday(&currtime, NULL);
		if ((STEP == 22) && (secdiff(table_zero_detected, currtime) < 0.05))
		{
			double v_l = 0;
			double v_r = 0;

			// double k_p = 0.01;
			// double k_i = 0.01;
			// double k_d = 0.01;

			// v_l = 0.8 + k_p * (table_zero[0] - 3);
			// v_r = 0.8 - k_p * (table_zero[0] - 3);

			rose.set_wheels(v_l, v_r, v_l, v_r);
		}
		else if (STEP == 22)
		{
			rose.set_wheels(-rotate_vel*2, rotate_vel*2, -rotate_vel*2, rotate_vel*1.5);
		}

		if ((STEP == 22) && (chili.tags[table_zero_chilitag][0]) && (table_zero[1] > 0))
		{
			rose.set_wheels(0, 0, 0, 0);
			STEP = 23;
			usleep(1000000); // 1 second
		}







	}

	// 	// get the current position of the robot
	// 	pose_lock.lock();
	// 	vec pose = robot_pose;
	// 	pose_lock.unlock();
	// 	vec pos = pose(span(0,1));
	// 	double theta = pose(2);

	// 	// get the current plan
	// 	path_lock.lock();
	// 	mat path_plan = pathplan;
	// 	vec pose_plan = poseplan;
	// 	bool do_pose = dopose;
	// 	double twist = twistplan;
	// 	double grab = grabplan;
	// 	path_lock.unlock();
	// 	int nwaypoints = (int)path_plan.n_cols;
	// 	if (nwaypoints < 2)
	// 	{
	// 		// if there is no path, stop the robot for now
	// 		rose.set_wheels(0, 0, 0, 0);
	// 		continue;
	// 	}
	// 	else if (nwaypoints == 2)
	// 	{
	// 		printf("Reached kitchen, autonomous stopping\n");
	// 		rose.set_wheels(0, 0, 0, 0);
	// 		continue;
	// 	}

	// 	// do calculation of the wheels
	// 	// first find the closest waypoint as the "start"
	// 	vec distance(nwaypoints);
	// 	mat diffs = path_plan - repmat(pos, 1, nwaypoints);
	// 	uword target_index = 0;
	// 	vec target = path_plan.col(0);
	// 	for (int j = 0; j < nwaypoints; j++)
	// 	{
	// 		vec a = diffs.col(j);
	// 		vec b = diffs.col(target_index);
	// 		distance[j] = eucdist(a);
	// 		if (distance[j] < eucdist(b))
	// 		{
	// 			target_index = j;
	// 			target = path_plan.col(target_index);
	// 		}
	// 	}

	// 	// acceptance radius && sector check
	// 	if (distance[target_index] < 20 || (distance[target_index] < 40 && !within_value(angle(target - pos) - theta, -45, 45)))
	// 	{
	// 		if (target_index >= nwaypoints-1)
	// 		{
	// 			printf("target reached\n");
	// 			rose.set_wheels(0, 0, 0, 0);
	// 		}
	// 		else
	// 		{
	// 			target_index++;
	// 			target = path_plan.col(target_index);
	// 		}
	// 	}

	// 	// once the target is found, then calculate the trajectory
	// 	double theta_k_p = 0.025;
	// 	double delta_theta = angle(target - pos) - theta;
	// 	double v_l, v_r;
	// 	v_l = (1.0 - theta_k_p * delta_theta);
	// 	// - theta_k_i * r.integral_error
	// 	// - theta_k_d * r.delta_theta_diff);
	// 	v_r = (1.0 + theta_k_p * delta_theta);
	// 	// + r.theta_k_i * r.integral_error
	// 	// + r.theta_k_d * r.delta_theta_diff);

	// 	cout << "current angle: " << theta << endl;
	// 	cout << "delta theta: " << delta_theta << endl;
	// 	cout << "left vel: " << v_l << endl << "right vel: " << v_r << endl;
	// 	cout << "waypoint: " << endl << path_plan.col(path_plan.n_cols-1) << endl;
	// 	cout << "current position: " << endl << pose << endl;
	// 	cout << endl;

	// 	// send the trajectory to the robot's wheels
	// 	rose.set_wheels(v_l, v_r, v_l, v_r);

	// 	// only if we want to do a pose do we activate the pose solver
	// 	if (!do_pose)
	// 	{
	// 		rose.stop_arm();
	// 		continue;
	// 	}

	// 	// get the list of all possible poses
	// 	bool feasible_found = false;
	// 	double baseangle = angle(poseplan.col(0).subvec(0,1));
	// 	vec enc(6);
	// 	for (int i = 0; i <= 90; i += 5)
	// 	{ // 5 degree granularity
	// 		mat R = rotationMat(0, 0, baseangle);
	// 		// if a negative pose proves to find a solution, stop
	// 		vec negpose({ 0, cos(deg2rad(-i)), sin(deg2rad(-i)) });
	// 		negpose = R * negpose;
	// 		if (rose.get_arm_position_placement(poseplan, negpose, twist, grab, enc))
	// 		{
	// 			feasible_found = true;
	// 			break;
	// 		}
	// 		// if a positive pose proves to find a solution, stop
	// 		vec pospose({ 0, cos(deg2rad(i)), sin(deg2rad(i)) });
	// 		pospose = R * pospose;
	// 		if (rose.get_arm_position_placement(poseplan, negpose, twist, grab, enc))
	// 		{
	// 			feasible_found = true;
	// 			break;
	// 		}
	// 	}

	// 	// send poses to the robot arm
	// 	if (feasible_found)
	// 	{
	// 		rose.set_arm(enc(0), enc(1), enc(2), enc(3), enc(4), enc(5));
	// 	}
	// 	else
	// 	{
	// 		rose.stop_arm();
	// 	}
	// }

	// clean the motion
	rose.send(zeros<vec>(10));
}

void motion_plan(void)
{
	// vec goal = vec({ 73, 1000 }); // this will have to change later somehow
	// // grab the map
	// mat localmap = globalmap.map;

	// AStar astar(localmap, goal);
	while (!stopsig)
	{
	// 	// try and see if we are allowed to go autonomous
	// 	bool en = false;
	// 	autonomous_lock.lock();
	// 	en = auto_enable && manual_confirmed;
	// 	auto_confirmed = true;
	// 	autonomous_lock.unlock();

	// 	// if we are indeed enabled, then we can compute the path to take
	// 	if (!en)
	// 	{
			continue;
	// 	}

	// 	// grab the current position
	// 	pose_lock.lock();
	// 	vec pose = robot_pose;
	// 	pose_lock.unlock();

	// 	// compute the new path
	// 	vector<MotionAction> actionpath;
	// 	vec curr = pose(span(0,1));
	// 	astar.compute(curr, actionpath);
	// 	if (astar.impossible())
	// 	{
	// 		// store an empty path
	// 		path_lock.lock();
	// 		pathplan = mat(2, 0);
	// 		dopose = false; // shut off the pose just in case
	// 		path_lock.unlock();
	// 		continue;
	// 	}

	// 	// prune bad motion vectors
	// 	vector<vec> prunedpath;
	// 	vec origin;
	// 	for (int i = 0; i < actionpath.size(); i++)
	// 	{
	// 		if (i == 0)
	// 		{
	// 			origin = vec({ actionpath[i].x, actionpath[i].y });
	// 			prunedpath.push_back(origin);
	// 		}
	// 		else if (i == actionpath.size() - 1)
	// 		{
	// 			prunedpath.push_back(vec({ actionpath[i].x, actionpath[i].y }));
	// 		}
	// 		else
	// 		{
	// 			vec target({ actionpath[i].x, actionpath[i].y });
	// 			if (eucdist(target - origin) >= 30)
	// 			{
	// 				prunedpath.push_back(target);
	// 				origin = target;
	// 			}
	// 		}
	// 	}

	// 	// store the pruned path
	// 	mat coalescedpath(2, prunedpath.size());
	// 	for (int i = 0; i < prunedpath.size(); i++)
	// 	{
	// 		coalescedpath.col(i) = vec({ prunedpath[i](0), prunedpath[i](1) });
	// 	}
	// 	path_lock.lock();
	// 	pathplan = coalescedpath;
	// 	dopose = false;
	// 	path_lock.unlock();
	}
}

void display_interface(void)
{
	cube frame(500, 500, 3, fill::zeros);

	while (!stopsig)
	{
		frame.zeros();

		// get the position of the robot
		pose_lock.lock();
		vec pose = robot_pose;
		pose_lock.unlock();

		// create a window around the pose
		int mux = (int)round(pose(0));
		int muy = (int)round(pose(1));
		double mut = pose(2);
		int sw2 = screen->w / 2;
		int sh2 = screen->h / 2;

		// draw the map
		globalmap.blit(frame, mux, muy);

		// draw the landmarks
		vec white({ 1, 1, 1 });
		for (int i = 0; i < landmarks.size(); i++)
		{
			sim_landmark &lm = landmarks[i];
			lm.blit(frame, mux, muy, chilitags.col(i));
		}

		// draw the particle filter
		pf.blit(frame, mux, muy);

		// draw the robot's position and pose
		int x, y;
		vec yellow({ 1, 1, 0 });
		draw_circle(frame, yellow, vec({ (double)sw2, (double)sh2 }), 20);
		for (int _i = -5; _i <= 5; _i++)
		{
			for (int _j = -5; _j <= 5; _j++)
			{
				x = sw2 + _j;
				y = sh2 + _i;
				if (x < 0 || x >= (int)frame.n_cols || y < 0 || y >= (int)frame.n_rows)
				{
					continue;
				}
				frame(y,x,0) = 1;
				frame(y,x,1) = 0;
				frame(y,x,2) = 0;
			}
		}
		x = (int)round(sw2 + (10 * cos(deg2rad(mut))));
		y = (int)round(sh2 + (10 * sin(deg2rad(mut))));
		vec color({ 0, 1, 1 });
		draw_line(frame, color, vec({(double)sw2,(double)sh2-1}), vec({(double)x,(double)y-1}));
		draw_line(frame, color, vec({(double)sw2,(double)sh2+1}), vec({(double)x,(double)y+1}));
		draw_line(frame, color, vec({(double)sw2-1,(double)sh2}), vec({(double)x-1,(double)y}));
		draw_line(frame, color, vec({(double)sw2+1,(double)sh2}), vec({(double)x+1,(double)y}));
		draw_line(frame, color, vec({(double)sw2,(double)sh2}), vec({(double)x,(double)y}));

		// draw A*
		path_lock.lock();
		mat path_plan = pathplan;
		path_lock.unlock();
		if (auto_enable)
		{
			vec purple({ 1, 0, 1 });
			for (int j = 0; j < (int)path_plan.n_cols; j++)
			{
				vec action = path_plan.col(j) + vec({ (double)(sw2 - mux), (double)(sh2 - muy) });
				draw_circle(frame, purple, vec({ action(0), action(1) }), 2.0);
			}
		}

		// push onto the screen
		screenblit(screen, frame);
		SDL_Flip(screen);
		SDL_Delay(25);
	}
}

static double secdiff(struct timeval &t1, struct timeval &t2)
{
	double usec = (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
	double sec = (double)(t2.tv_sec - t1.tv_sec);
	return sec + usec;
}
