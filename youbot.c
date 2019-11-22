/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */



#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>


int robot_angle = 0;
#define TIME_STEP 32


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void stop()
{
	base_reset();
}

void go_forward()
{
	base_forwards();
}

void go_backward()
{
	base_backwards();
}

void turn_left()
{
	base_turn_left();
	robot_angle = robot_angle + 90;
	if (robot_angle == 360)
		robot_angle = 0;

}

void turn_right()
{
	base_turn_right();
	robot_angle = robot_angle - 90;
	if (robot_angle == -90)
		robot_angle = 270;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ANGLE_UNIT (45) // what angle we should round to for turning commands
#define FOCAL_LENGTH (120) // camera focal length
#define MAX_OBJS (32) // max # of detected objects in total FOV

#define TYPES 4 // types of fruits (colors), uses the
#define FUNCS 4 // functions of fruits (+hp, +en, -en, +arm)
#define PRI 0.7 // probability of the primary function
#define SEC 0.3 // probability of the secondary function
#define FRUIT_BASE 4
// There are 4 indices before types of fruits are in the enum


// the ratio of time to turn vs time to travel a meter
#define TURN_FACTOR (60)
// discourage travelling backwards if possible
#define BACK_FACTOR (10)

int cameras[4] = {4, 8, 9, 10}; // cam ids of the cameras we're using
// front, back, left, right

/* TYPEDEFS */
enum types {blu_zombie, aqu_zombie, gre_zombie, pur_zombie,
          red_berry, yel_berry, ora_berry, pin_berry,
          stump, tree, wall, edge};

typedef struct img_observation {
  int type; // object type
  int cam; // camera id
  int x[2]; // bounding box
  int y[2];
	int y1[2]; // only used for walls
} BoundingBox;

typedef struct obj_position {
  int type;
  float x;
  float y;
  float x1; // x1, y1 are only used for walls
  float y1;
} Obj, * Obji;

typedef struct robot_position {
  int x;
  int y;
  int angle; // 0 up, 1 right, 2 down, 3 left
} RobotPos;

typedef struct rgbColor {
  float r;
  float g;
  float b;
} RgbColor;

typedef struct hsvcolor {
  float h;
  float s;
  float v;
} HsvColor;

typedef struct vector {
  float x;
  float y;
} * Vector;
//////////////////////////

/* intialize robot position */
RobotPos robot_pos;

/* UTILITY FUNCTIONS */
/* 3-input max and min functions (useful for perception math) */
float mmin(float a, float b, float c)
{
  a = fminf(a, b); c = fminf(a, c);
  return c;
}
float mmax(float a, float b, float c)
{
  a = fmaxf(a, b); c = fmaxf(a, c);
  return c;
}

/* Rounds angle (in degrees) to nearest multiple of ANGLE_UNIT */
// TODO remove?
int round_to_angle_setting(int angle)
{
  int remainder = angle % ANGLE_UNIT;
  if (remainder == 0)
    return angle;
  return (angle + ANGLE_UNIT - remainder);
}

/* returns vector of robot - obj in buffer v */
void vectorCalc(RobotPos robot, Obj obj, Vector v) {
	v->x = (robot.x - obj.x);
	v->y = (robot.y - obj.y);
}

/* returns magnitude of v */
float vectorMagnitude(Vector v) {
	return (sqrt(pow(v->x, 2.0) + pow(v->y, 2.0)));
}

/* Adds the given array of vectors. */
Vector vector_sum(Vector * vectors)
/* must be null-terminated. also frees the input vectors along the way */
{
  Vector sum = malloc(sizeof(struct vector));
  sum->x = 0;
  sum->y = 0;

  while (*vectors != NULL) {
    sum->x += (*vectors)->x;
    sum->y += (*vectors)->y;
    free(*vectors);
    vectors++;
  }
  return sum;
}

// compute the dot product between the 2 inputs, length also required
double dot(double* first, double* second, int len) {
  double ret = 0;
  for (int i = 0; i < len; i++) {
    ret += first[i] * second[i];
  }
  return ret;
}

// random number between lower and upper
int randomN(int lower, int upper)
{
    int num = (rand() % (upper - lower + 1)) + lower;
    return num;
}

// Return the angle from one position to another
double angle (double x1, double y1, double x2, double y2) {
	return atan((y2 - y1) / (x2 - x1));
}

// Return the magnitude of the given vector
double mag (Vector v) {
	return sqrt((v->x) * (v->x) + (v->y) * (v->y));
}

// Return the reverse direction through simple arithmetic
int reverse (int angle) {
	return (angle + 2) % 4;
}

// Change vector to be perpendicular to itself
void vectorPerpend(Vector v) {
	float new_x = v->y;
	float new_y = -1 * v->x;

	v->x = new_x;
	v->y = new_y;
}

// Do a simple projection and return the magnitude, assuming direction
// of angle with unit vector. Simple returns since only 4 directions.
double proj (Vector v, int angle) {
	if (angle == 0) {
		return round(v->y);
	} else if (angle == 1) {
		return round(v->x);
	} else if (angle == 2) {
		return round(-v->y);
	} else {
		return round(-v->x);
	}
}

////////////////

/* IMAGE PROCESSING FUNCTIONS */

/* converts rgb color codes to hsv */
void rgb_to_hsv( RgbColor  * rgb, HsvColor * hsv)
/* r, g, and b should be in range [0, 1]*/
{
  // HsvColor hsv;
  float min, max, delta;

  min = mmin(rgb->r, rgb->g, rgb->b);
  max = mmax(rgb->r, rgb->g, rgb->b);

  hsv->v = max;
  delta = max - min;

  if (max != 0)
    hsv->s = delta / max;
  else {
    hsv->s = 0;
    hsv->h = -1;
    return;
  }

  if (rgb->r == max)
    hsv->h = (rgb->g - rgb->b) / delta;
  else if (rgb->g == max)
    hsv->h = 2.0 + (rgb->b - rgb->r) / delta;
  else
    hsv->h = 4.0 + (rgb->r - rgb->g) / delta;
  hsv->h *= 60;

  if (hsv->h < 0)
    hsv->h += 360;
}

/* Update hsv with HSV data from pixel (xyz) */
void get_pixel_data(const unsigned char * image, int x, int y, HsvColor * hsv)
{
  int r = wb_camera_image_get_red(image, 128, x, y);
  int g = wb_camera_image_get_green(image, 128, x, y);
  int b = wb_camera_image_get_blue(image, 128, x, y);

  RgbColor rgb;

  rgb.r = (float) r / 255.0;
  rgb.g = (float) g / 255.0;
  rgb.b = (float) b / 255.0;

  rgb_to_hsv(&rgb, hsv);
}

/* Return object id that HSV color matches with (or -1 if none) */
int color_match(HsvColor * hsv)
{
  int type = -1;
	//
  if      (hsv->h >= 120 && hsv->h <= 132 && hsv->s >= 0.7) type = gre_zombie;
  else if (hsv->h >= 250 && hsv->h <= 280 && hsv->s >= 0.7) type = pur_zombie;
  else if (hsv->h >= 165 && hsv->h <= 185 && hsv->s >= 0.7) type = aqu_zombie;
  else if (hsv->h >= 200 && hsv->h <= 225 && hsv->s >= 0.7) type = blu_zombie;

  else if (hsv->h >= 315 && hsv->h <= 340 && hsv->s <= 0.5) type = pin_berry;
  else if (hsv->h >= 40  && hsv->h <= 60  && hsv->s >= 0.7) type = yel_berry;
  else if (hsv->h >= 15  && hsv->h <= 35)                   type = ora_berry;
  else if (hsv->h >= 0   && hsv->h <= 10  && hsv->s >= 0.7) type = red_berry;

	if (hsv->h >= 220 && hsv->h <= 230 && hsv->s <= 0.4) type = wall;
	else if (hsv->h >= 210 && hsv->h <= 230 && hsv->s <= 0.3 && hsv->v < 0.15) type = stump;
	else if ((hsv->h >= 220 || hsv->h < 30) && hsv->s < 0.4 && hsv->v < 0.15) type = tree;
	// if (hsv->h >= 230 && hsv->s <= 0.15 && hsv->v >= .25 && hsv->v <= 0.33) type = edge;

  return type;
}

/* Update bounding box information */
void obs_update(BoundingBox * box, int type, int x, int y)
{
  if(box->type == -1) {
    box->type = type;
    box->x[0] = box->x[1] = x;
    box->y[0] = box->y[1] = y;

	} else if (box->type == wall ){
		// walls have special procedures for tracking the xy data - we need 2 yranges
		if(x < box->x[0]) box->x[0] = x;
		if (x < box->x[1]) box->x[0] = y;

		if(x == box->x[0]) {
			if(y > box->y[1]) box->y[1] = y;
	    if (y < box->y[0]) box->y[0] = y;
		}

		if (x == box->x[1]) {
			if(y > box->y1[1]) box->y1[1] = y;
	    if (y < box->y1[0]) box->y1[0] = y;
		}

	} else if (box->type == tree ){
		// be extra careful to only look at the width of the stem
    if(y > box->y[1]) box->y[1] = y;
    if (y < box->y[0]) box->y[0] = y;

		if(y > 10) {
	    if(x > box->x[1]) box->x[1] = x;
    	if (x < box->x[0]) box->x[0] = x;
		}

  } else {
    // update bounding box info normally
    if(x > box->x[1]) box->x[1] = x;
    if (x < box->x[0]) box->x[0] = x;

    if(y > box->y[1]) box->y[1] = y;
    if (y < box->y[0]) box->y[0] = y;
  }
}

/* Calculate position information*/
float estimate_vertical_distance(BoundingBox * obj)
{
  float y_true = 0;
  int y_px = obj->y[1] - obj->y[0] + 1;

  if(obj->type == gre_zombie || obj->type == blu_zombie ||
        obj->type == pur_zombie || obj->type == aqu_zombie) {
      y_true = 1.77;

  } else if (obj->type == red_berry || obj->type == pin_berry ||
          obj->type == ora_berry || obj->type == yel_berry) {
      y_true = 0.1;

  } else if (obj->type == wall) {
		y_true = 2.0;

	} else if (obj->type == stump) {
		y_true = 0.1;

	} else if (obj->type == edge) {
		y_true = 0.1;
	}

	if(y_px >= 32 && obj->type != wall) {
			// cheat using width instead (just for zombies and trees)

			float x_true = 0;
			int x_px = obj->x[1] - obj->x[0];
			if(obj->type == tree) {
				x_true = 0.14;
			} else if (obj->type == gre_zombie || obj->type == blu_zombie ||
		        obj->type == pur_zombie || obj->type == aqu_zombie) {
				x_true = 0.4;
			}

			if(obj->type == tree){
				float d = x_true * FOCAL_LENGTH / x_px;
				// if(obj->type == tree) printf("%d %f\n", x_px, d);
				return d;
			}

	}

  return((float) y_true * FOCAL_LENGTH / y_px);
}
void calculate_XY_pos(BoundingBox * box, Obj * pos)
{
	// check if tree or stump
	if(box->type == stump) {
		int xrange = box->x[1] - box->x[0];
		int yrange = box->y[1] - box->y[0];

		if(yrange >= 32 && xrange <= yrange * 2) {
			// we probably misclassified a stump as a tree
			box->type = tree;
		}
	}

  int x_avg = (box->x[1] - box->x[0]) / 2 - 64;
  float z = estimate_vertical_distance(box);
  float theta = atan((float) x_avg / FOCAL_LENGTH);

  pos->type = box->type;
	switch (box->cam) {
				case 4: // front cam
					pos->x = z * sin(theta);
					pos->y = z * cos(theta);
					break;
				case 8: // back cam
					pos->x = -1 * z * sin(theta);
					pos->y = -1 * z * cos(theta);
					break;
				case 9: // right cam
					pos->x = z * cos(theta);
					pos->y = -1 * z * sin(theta);
					break;
				case 10: // left cam
					pos->x = -1 * z * cos(theta);
					pos->y = z * sin(theta);
					break;
	}

	if(!isnormal(pos->x)) pos->type = -2;
}

/* Process a wall's positional information */
void process_wall(BoundingBox * box, Obj * pos)
{
	pos->type = box->type;

	// get position data for each side of the wall
	Obj tmp_pos;
	BoundingBox tmp_box;

	tmp_pos.x = tmp_pos.y = 0; // to make the compiler happy

	tmp_box.type = wall;
	tmp_box.cam = box->cam;
	tmp_box.x[0] = tmp_box.x[1] = box->x[0];
	tmp_box.y[0] = box->y[0]; tmp_box.y[1] = box->y[1];
	calculate_XY_pos(&tmp_box, &tmp_pos);

	pos->x = tmp_pos.x;
	pos->y = tmp_pos.y;

	tmp_box.x[0] = tmp_box.x[1] = box->x[1];
	tmp_box.y[0] = box->y1[0]; tmp_box.y[1] = box->y1[1];
	calculate_XY_pos(&tmp_box, &tmp_pos);

	pos->x1 = tmp_pos.x;
	pos->y1 = tmp_pos.y;
}

/* Process data from a single camera input */
void process_single_image(int cam_id, BoundingBox * objs)
{
  // int len = 0;
  int type;
  const unsigned char * image = wb_camera_get_image(cam_id);
  HsvColor pxl;

	for(int i = 0; i < 12; i++) {
		objs[i].type = -1;
		objs[i].cam = cam_id;
	}

  for (int x = 0; x < 128; x++) {
    for (int y = 0; y < 64; y++) {
      get_pixel_data(image, x, y, &pxl);
      type = color_match(&pxl);

			// // TODO remove
			// if(type == tree) printf("%d: %d, %d. HSV %f, %f, %f\n", cam_id, x, y,
			// 		pxl.h, pxl.s, pxl.v);

      if (type >= 0) {
        obs_update(&(objs[type]), type, x, y);
      }
    }
  }
}

/* main image processing for 3 cams */
Obj ** process_input() {
  BoundingBox boxes[12];

  Obj ** results = malloc(3 * sizeof(Obj **));
  Obji zombies, berries, obstacles;

  zombies = results[0] = malloc(MAX_OBJS * sizeof(Obj));
  berries = results[1] = malloc(MAX_OBJS * sizeof(Obj));
  obstacles = results[2] = malloc(MAX_OBJS * sizeof(Obj));

  int zlen, blen, olen;
	zlen = blen = olen = 0;

  for(int j = 0; j < 4; j++) {
		process_single_image(cameras[j], boxes);
		for(int i = 0; i < 12; i++) {

	    if(boxes[i].type == wall) {
	      process_wall(&boxes[i], &(obstacles[olen++]));

	    } else if (boxes[i].type == blu_zombie || boxes[i].type == aqu_zombie ||
	    		boxes[i].type == gre_zombie || boxes[i].type == pur_zombie) {

	      calculate_XY_pos(&(boxes[i]), &(zombies[zlen++]));

	    } else if (boxes[i].type == red_berry || boxes[i].type == pin_berry ||
	    		boxes[i].type == yel_berry || boxes[i].type == ora_berry) {

	      calculate_XY_pos(&(boxes[i]), &(berries[blen++]));

	    } else if (boxes[i].type == edge || boxes[i].type == tree || boxes[i].type == stump) {

	      calculate_XY_pos(&(boxes[i]), &(obstacles[olen++]));
	    }
		}
  }

  zombies[zlen].type = berries[blen].type = obstacles[olen].type = -1;
  return results;
}
//////////////////////////////////////////

/* BEHAVIOR FUNCTIONS */

// fruit[int][int] double :
// An array containing each fruit type. The first index
// is based on the color of the fruit, by some deterministic
// way of organizing. The second index is based on the
// function of that color of the fruit. After accessing
// both indices, the data structure return the probability
// of that color of fruit resulting in that property.

// type: red, yel, ora, pink
// func: +40 energy, +20 health, -20 energy, 10 sec armour
double fruit[TYPES][FUNCS];
int fruitob[TYPES][FUNCS];

void fruit_init();
void compute();

// Initialize the fruit double array with uniform probability
// Initialize the fruitob double array with 0s
void fruit_init() {
  for (int i = 0; i < TYPES; i++) {
    for (int j = 0; j < FUNCS; j++) {
      fruit[i][j] = 1 / FUNCS;
      fruitob[i][j] = 0;
    }
  }
}

// Compute the probabilities according to observations
void fruit_compute() {
  for (int i = 0; i < TYPES; i++) {
    int types = 0;
    // Recrod which entries are non-zero
    int a = -1;
    int b = -1;
    for (int j = 0; j < FUNCS; j++) {
      if (fruitob[i][j] > 0) {
        if (a >= 0) {
          b = j;
        } else {
          a = j;
        }
        types++;
      }
    }
    if (types == 0) {
      // No data for this section, uniform distribution
      for (int j = 0; j < FUNCS; j++) {
        fruit[i][j] = 1 / FUNCS;
      }
    } else if (types == 1) {
      int p = fruitob[i][a];
      // Data should be recorded in a
      for (int j = 0; j < FUNCS; j++) {
        if (j == a) {
          double num = pow(PRI, (p + 1)) + pow(SEC, (p + 1));
          double den = pow(PRI, p) + pow(SEC, p);
          fruit[i][j] = num / den;
        } else {
          double num = pow(PRI, p) * SEC  + pow(SEC, p) * PRI;
          double den = pow(PRI, p) + pow(SEC, p);
          // Divide by number of things distributed in
          fruit[i][j] = num / (FUNCS - 1) / den;
        }
      }
    } else if (types == 2) {
      // 2 effects confirmed, other effects cannot happen
      int c1 = fruitob[i][a];
      int c2 = fruitob[i][b];
      for (int j = 0; j < FUNCS; j++) {
        // Probability a is primary
        double a1 = pow(PRI, c1) * pow(SEC, c2);
        double b1 = pow(PRI, c2) * pow(SEC, c1);
        double den = a1 + b1;
        if (j == a) {
          fruit[i][j] = a1 / den;
        } else if (j == b) {
          fruit[i][j] = b1 / den;
        } else {
          fruit[i][j] = 0;
        }
      }
    } else {
      // This program is wrong or the instructors
      // are trolling us
      printf("PANIC");
    }
  }
}

// Update the observation table
void fruit_update(int type, int func) {
  fruitob[type - FRUIT_BASE][func] += 1;
  fruit_compute();
}

// the utility value of a given type of fruit
double fruit_util(int health, int energy, int armour, RobotPos robot, Obji zombie, int type) {
  // compute the values for each function
  double f[4] = {0.0, 0.0, 0.0, 0.0};
  // f[0] for the value of 40 energy
  if (energy == 0) {
    if (health < 20) {
      f[0] = 10;
    } else {
      f[0] = 6;
    }
  } else if (energy < 20) {
    f[0] = 3;
  } else if (energy < 60) {
    f[0] = 2;
  } else {
    f[0] = (100.0 - energy) / 20.0;
  }
  // f[1] for the value of 20 health
  if (health < 25) {
    if (energy < 10) {
      f[1] = 10;
    } else {
      f[1] = 20;
    }
  } else if (health < 50) {
    f[1] = 3;
  } else if (health < 80) {
    f[1] = 2;
  } else {
    f[1] = (100.0 - health) / 10.0;
  }
  // f[2] for the value of -20 energy
  if (energy == 0) {
    f[2] = 0;
  } else if (energy < 20) {
    f[2] = - (energy / 10.0);
  } else if (energy < 40) {
    f[2] = -3;
  } else if (energy < 80) {
    f[2] = -2;
  } else {
    f[2] = - (100.0 - energy) / 10.0;
  }
  // f[3] for the value of armour
  while((*zombie).type != -1) {
		// Temporary function on the amount of influence a zombie has
		// given the distance
    int xd = 1.0 / (robot.x - (zombie)->x);
    int yd = 1.0 / (robot.y - (zombie)->y);
    f[3] += sqrt(xd * xd + yd * yd);
    zombie++;
  }
  return dot(f, fruit[type] - FRUIT_BASE, 4);
}

// Do similar things as zombies but for food
// So same structure but different parameters
double* findFood(RobotPos robot, Obji food, Obji zombie, int health, int energy, int armour)
{
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

	while((*food).type != -1) {
    double cons = fruit_util(health, energy, armour, robot, zombie, food->type);
    // printf("const is : %f\n", cons);
    double robx = robot.x - food->x;
    double roby = robot.y - food->y;
    if (robx >= 0) {
      robx += 1;
  		v->x += -cons / sqrt(robx);
    } else {
      robx -= 1;
  		v->x += cons / sqrt(-robx);
    }
    if (roby >= 0) {
      roby += 1;
  		v->y += -cons / sqrt(roby);
    } else {
      roby -= 1;
  		v->y += cons / sqrt(-roby);
    }
    food++;
	}

	double* seek = malloc(sizeof(double)*5);

	// formulas subjected to tweaking
	seek[0] = proj(v, robot.angle);
	seek[1] = proj(v, reverse(robot.angle)) - BACK_FACTOR;
	// convenient calculation for the sides
	seek[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	seek[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	seek[4] = 0;

	return seek;
}

// The vectors given by the food spread out in the map, rather than
// in the sensors. Mapped fruits are less accurate, and likely farther
// away, so they are given less weights.


// Compute vector of the given zombies,
// but a multiplying factor is at work as well.
double* avoidZombies(RobotPos robot, Obji zombie, int armour)
{
	double* avoid = malloc(sizeof(double)*5);

  // We don't care about zombies if we have armour
  if (armour > 2) {
    for (int i = 0; i < 5; i++) {
      avoid[i] = 0;
    }
    return avoid;
  }

  // wait to enable robot armour in the info screen
  // printf("robot armour info: %s\n", robot_info.armour);

  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  struct vector vbuf; // temporary info holding
	float mag;

	while((*zombie).type != -1) {
		// Temporary function on the amount of influence a zombie has
		// given the distance
    double robx = robot.x - zombie->x;
    double roby = robot.y - zombie->y;
    double fac = 0;
    if (zombie->type == blu_zombie) {
      fac = 1.4;
    } else if (zombie->type == aqu_zombie) {
      // can use 3 meter conversion, no need to worry if not near
      fac = 2.2;
    } else if (zombie->type == gre_zombie) {
      fac = 1.8;
    } else {
      fac = 1.7;
    }
    if (robx >= 0) {
      robx += 1;
      v->x += fac / sqrt(robx);
    } else {
      robx -= 1;
      v->x += -fac / sqrt(-robx);
    }
    if (roby >= 0) {
      roby += 1;
      v->y += fac / sqrt(roby);
    } else {
      roby -= 1;
      v->y += - fac / sqrt(-roby);
    }
    zombie++;
	}

	//  // Temporary function on the amount of influence a zombie has
	// 	// given the distance (Glenn)
  //   v->x += (2.0 / sqrt(robot.x - (zombie)->x + 1));
  //   v->y += (2.0 / sqrt(robot.y - (zombie)->y + 1));

	// printf("v->x: %f\n", v->x);
	// printf("v->y: %f\n", v->y);

	// formulas subjected to tweaking
	avoid[0] = proj(v, robot.angle);
	avoid[1] = proj(v, reverse(robot.angle));
	// convenient calculation for the sides
	avoid[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	avoid[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	avoid[4] = 0;

	return avoid;

}

double* knockBerryDown(void)
{
	double* knock = malloc(sizeof(double)*5);
	knock[0] = 0;
	knock[1] = 0;
	knock[2] = 0;
	knock[3] = 0;
	knock[4] = 0;
	// knock[4] = 10000;

	return knock;
}


double* explore(void)
{
	double* explore = malloc(sizeof(double)*5);

	// The exploring will most often be moving forwards or backwards,
	// but sometimes also turning
	explore[0] = 0.27 + randomN(1, 8) / 10.0;
	explore[1] = 0;
	explore[2] = 0.1 + randomN(1, 3) / 10.0;
	explore[3] = 0.1 + randomN(1, 3) / 10.0;
	explore[4] = 0;

	return explore;
}


double* avoidObstacles(RobotPos robot, Obji obs)
{
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

	// while(obs->type != -1) {
	// 	v->x += (0.8 / sqrt(robot.x - obs->x + 1));
	// 	v->y += (0.8 / sqrt(robot.y - obs->y + 1));
	// 	obs++;
	// }
	struct vector vbuf;
	Obj obs_point;
	float mag;

	while(obs->type != -1) {
		if(obs->type == wall || obs->type == edge){
			// obstacle is a flat plane; we want the opposing vector to be perpendicular to it
			// calculate that vector

			// first, see if there is a flat projection from the obstacle to us
			if(robot.x <= obs->x1 && robot.x >= obs->x) {
				obs_point.x = robot.x;
				obs_point.y = (obs->y + obs->y1) / 2; // quick shortcut, could improve

			} else if (robot.y <= obs->y1 && robot.y >= obs->y1) {
				obs_point.x = (obs->x + obs->x1) / 2;
				obs_point.y = robot.y;

			} else {
				// if not, nothing to worry about
				obs++;
				continue;
			}

			// get the vector
			vectorCalc(robot, obs_point, &vbuf);
			mag = vectorMagnitude(&vbuf);
			if(mag != 0) {
				v->x += vbuf.x / (pow(mag, 2.0));
				v->y += vbuf.y / (pow(mag, 2.0));
			}
		} else {
			// not a wall (could be a stump or tree)
			// get the vector
			vectorCalc(robot, obs_point, &vbuf);
			mag = vectorMagnitude(&vbuf);
			if(mag != 0) {
				v->x += vbuf.x / (pow(mag, 2.0));
				v->y += vbuf.y / (pow(mag, 2.0));
			}
		}
		obs++;
	}
	// printf("v->x: %f\n", v->x);
	// printf("v->y: %f\n", v->y);

	double* avoid = malloc(sizeof(double)*5);


	// formulas subjected to tweaking
	avoid[0] = proj(v, robot.angle);
	avoid[1] = proj(v, reverse(robot.angle));
	// convenient calculation for the sides
	avoid[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	avoid[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	avoid[4] = 0;

	free(v);

	return avoid;
}



void printVotes(double * votes) {
	for(int i = 0; i < 5; i++) {
			printf("%f ", votes[i]);
	}
	putchar('\n');
}
// Arbiter decides which action to take
// For now ties go to the first tie one in array *CAN BE CHANGED*
// [0]: forward, [1]: back, [2]: left, [3]: right, [4]: do nothing
//
// Gets all arguments from sensors and internal map
// Parameter can be further explained here
void arbiter(RobotPos robot, Obji zombie, Obji food, Obji obs, int health, int energy, int armour)
{
  // Make sure we commit to full turns before progressing through other commands
  static int turning = 0;

  if (turning != 0) {
    if (turning > 0) {
      // We were turning left
      turn_left();
      turning -= 1;
    } else {
      turn_right();
      turning += 1;
    }
    return;
  }

	double* foodVote = findFood(robot, food, zombie, health, energy, armour);
	double* avoidObstaclesVote = avoidObstacles(robot, obs);
	double* exploreVote = explore();
	double* knockBerryVote = knockBerryDown();
	double* avoidZombiesVote = avoidZombies(robot, zombie, armour);

	// for (int i = 0; i < 5; i++) {
	// 	printf("%d\n", avoidZombiesVote[i]);
	// }

  printf("vote results:\n");
  printVotes(foodVote);
  printVotes(avoidObstaclesVote);
  printVotes(exploreVote);
  printVotes(knockBerryVote);
  printVotes(avoidZombiesVote);
	// double* finalVotes = malloc(sizeof(double)*5);
	double finalVotes[5];
	int winningIndex = 0;
	for (int i = 0; i < 5; i++)
	{
		finalVotes[i] = foodVote[i] + avoidObstaclesVote[i] + exploreVote[i] + knockBerryVote[i] + avoidZombiesVote[i];
		if (finalVotes[i] > finalVotes[winningIndex])
		{
			winningIndex = i;
		}
	}

	if (winningIndex == 0)
	{
		printf("forwards won\n");
		go_forward();
	}
	else if (winningIndex == 1)
	{
		printf("backwards won\n");
		go_backward();
	}
	else if (winningIndex == 2)
	{
		printf("left won\n");
    turning = 150;
		turn_left();
	}
	else if (winningIndex == 3)
	{
		printf("right won\n");
    turning = -150;
		turn_right();
	}
	else if (winningIndex == 4)
	{
		printf("do nothing won\n");
		stop();
	}

	robot.angle = robot_angle;
}

void robot_control(int health, int energy, int armour)
{
  // TODO
	Obj ** detected = process_input();

	Obj * zombies;
	Obj * berries;
	Obj * obstacles;
	zombies = detected[0];
	berries = detected[1];
	obstacles = detected[2];

	Obj * tmp = obstacles;
	// for debugging
	// while((*tmp).type > 0) {
	// 	printf("%d, %f, %f\n", tmp->type, tmp->x, tmp->y);
	// 	tmp++;
	// }

	// call to arbiter

	// motor output
	arbiter(robot_pos, zombies, berries, obstacles, health, energy, armour);


	// motor output
	free(zombies); free(berries); free(obstacles);
	free(detected);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  struct Robot robot_info = {100,100,0};
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  //display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  get_all_berry_pos();

  int robot_not_dead = 1;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
	robot_pos.x = robot_pos.y = 0;
	robot_pos.angle = robot_angle;


  // wb_accelerometer_enable(1,1);
  wb_gps_enable(2,TIME_STEP); // gps
  // wb_compass_enable(3,TIME_STEP);
  wb_camera_enable(4,TIME_STEP); // front cam
  // wb_camera_enable(5,TIME_STEP);
  // wb_camera_enable(6,TIME_STEP);
  // wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP); // back cam
  wb_camera_enable(9,TIME_STEP); // right cam
  wb_camera_enable(10,TIME_STEP); // left cam
  // wb_camera_enable(11,TIME_STEP);
  // wb_gyro_enable(12,TIME_STEP);
  // wb_light_sensor_enable(13,TIME_STEP);
  // wb_receiver_enable(14,TIME_STEP);
  // wb_range_finder_enable(15,TIME_STEP);
  // wb_lidar_enable(16,1); //600

  //WbDeviceTag lidar = wb_robot_get_device("lidar");
  //wb_lidar_enable_point_cloud(lidar);

  //WbDeviceTag rec = wb_robot_get_device("receiver");

  //int i = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot_not_dead == 1)
  {

	if (robot_info.health < 0)
    {
		robot_not_dead = 0;
		printf("ROBOT IS OUT OF HEALTH\n");
	}

	if (timer % 2 == 0)
	{
		const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
		check_berry_collision(&robot_info, trans[0], trans[2]);
		check_zombie_collision(&robot_info, trans[0], trans[2]);
		//printf("%f\n", trans[0]);
	}
    if (timer == 16)
    {
        update_robot(&robot_info);
        timer = 0;
    }

    step();



    int c = keyboard(pc);
    pc = c;
    timer=timer+1;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // this is called everytime step.

    // if (i < 100)
    // {
    // 	base_forwards();
    // }
    // if (i == 100)
    // {
    // 	base_reset();
    // 	base_turn_left();
    // }
    // if (i == 300)
    // {
    // 	i = 0;
    // }
    // i++;

    robot_control(robot_info.health, robot_info.energy, robot_info.armour);
    // initialize fruit table
    fruit_init();


   //  if (wb_receiver_get_queue_length(rec) > 0)
  	// {
  	// 	const char *buffer = wb_receiver_get_data(rec);
   //      printf("Communicating: received \"%s\"\n", buffer);
   //  	wb_receiver_next_packet(rec);
   //  }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
