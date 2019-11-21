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

int cameras[3] = {4, 8, 9}; // cam ids of the cameras we're using
// front, back, left

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
  float x1;
  float y1;
} Obj;

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
int round_to_angle_setting(int angle)
{
  int remainder = angle % ANGLE_UNIT;
  if (remainder == 0)
    return angle;
  return (angle + ANGLE_UNIT - remainder);
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
  // if      (hsv->h >= 120 && hsv->h <= 132 && hsv->s >= 0.7) type = gre_zombie;
  // else if (hsv->h >= 250 && hsv->h <= 280 && hsv->s >= 0.7) type = pur_zombie;
  // else if (hsv->h >= 165 && hsv->h <= 185 && hsv->s >= 0.7) type = aqu_zombie;
  // else if (hsv->h >= 200 && hsv->h <= 225 && hsv->s >= 0.7) type = blu_zombie;
	//
  // else if (hsv->h >= 315 && hsv->h <= 340 && hsv->s <= 0.5) type = pin_berry;
  // else if (hsv->h >= 40  && hsv->h <= 60  && hsv->s >= 0.7) type = yel_berry;
  // else if (hsv->h >= 15  && hsv->h <= 35)                   type = ora_berry;
  // else if (hsv->h >= 0   && hsv->h <= 10  && hsv->s >= 0.7) type = red_berry;

  // TODO: add obstacles
  // obstacles go here
	// if (hsv->h >= 220 && hsv->h <= 230 && hsv->s <= 0.4) type = wall;
	if (hsv->h >= 220 && hsv->h <= 230 && hsv->s <= 0.35 && hsv->v < 0.1) type = stump;
	else if (hsv->s < 0.4 && hsv->v < 0.2) type = tree;


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
  int y_px = obj->y[1] - obj->y[0];

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
	}

	if(y_px >= 32 && obj->type != wall) {
			// cheat using width instead (just for zombies and trees)
			printf("Got here!\n");
			int x_true = 0;
			int x_px = obj->x[1] - obj->x[0];
			if(obj->type == tree) {
				x_true = 0.3;
			} else if (obj->type == gre_zombie || obj->type == blu_zombie ||
		        obj->type == pur_zombie || obj->type == aqu_zombie) {
				x_true = 0.4;
			}

			if(x_true > 0) return((float) x_true * FOCAL_LENGTH / x_px );
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
			// we misclassified a stump as a tree
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
}

/* Process a wall's positional information */
void process_wall(BoundingBox * box, Obj * pos)
{
	pos->type = wall;

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
Obj * process_input()
{
  BoundingBox boxes[12]; // TODO fix
  Obj * results;
  results = malloc(MAX_OBJS * sizeof(Obj));
  int len = 0;

  // each camera
  for(int j = 0; j < 3; j++) {

    process_single_image(cameras[j], boxes);

    // process info
    for(int i = 0; i < 12; i++) { // TODO fix for variable length?
			if(boxes[i].type == wall) {
				process_wall(&(boxes[i]), &(results[len++]));

			} else if(boxes[i].type != -1) {
        calculate_XY_pos(&(boxes[i]), &(results[len++]));
      }

			// TODO remove
			// if(boxes[i].type == stump) {
			// 	printf("stump: camera %d xrange [%d, %d], yrange [%d, %d]\n",
			//  		j, boxes[i].x[0], boxes[i].x[1], boxes[i].y[0], boxes[i].y[1]);
			// }
    }
  }
  results[len].type = -1;
  return results;
}

//////////////////////////////////////////

/* BEHAVIOR AND CONTROL FUNCTIONS */
/*
// Return vector computed by robot going directly away from the obstacle
Vector avoid_single_obstacle(RobotPos robot, Pos obstacle)
// currently assuming obstacle pos is absolute - could also be relative too
{
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = robot.x - obstacle.x;
  v->y = robot.y - obstacle.y;

  return v;
}

// Return vector computed by robot going directly away from the zombie,
// but a multiplying factor is at work as well.
Vector avoid_zombie(RobotPos robot, Posi* zombie, int factor)
{
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

  while(*zombie != NULL) {
    v->x += (robot.x - (*zombie)->x) * factor;
    v->y += (robot.y - (*zombie)->y) * factor;
    zombie++;
  }
  return v;
}

// TODO: Add a variation with mutliple inputs
// Return vector computed by robot going directly to the food
Vector looking_for_food(RobotPos robot, Pos food, int factor)
{
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = (robot.x - food.x) * factor;
  v->y = (robot.y - food.y) * factor;

  return v;
}

// TODO: update for new spec
// Given the parameters, execute commands for the actions
void arbiter(RobotPos robot, Pos obstacle, Posi* zombie, Pos food)
{
  // calculate behavior output vectors here
  Vector v_avoid_obstacle = avoid_single_obstacle(robot, obstacle);
  // Tweak factor based on health, type of zombie, and proximity
  // The tweaking might happen elsewhere
  Vector v_avoid_zombie = avoid_zombie(robot, zombie, 2);
  // Change factor when low energy, decrease when high energy
  Vector v_find_food = looking_for_food(robot, food, -1);

  // collect vectors
  Vector v_list[4];
  v_list[0] = v_avoid_obstacle;
  v_list[1] = v_avoid_zombie;
  v_list[2] = v_find_food;
  v_list[3] = NULL;

  // arbitration
  Vector v_output = vector_sum(v_list);

  // calculate output angle
  float angle = acos((v_output->x + v_output->y) / sqrt(v_output->x * v_output->x  + v_output->y * v_output->y));

  // convert to degrees
  angle = angle / M_PI * 180;

  int output_angle = round_to_angle_setting(angle);
  if (output_angle == 360)
    output_angle = 0;
  else if (output_angle < 0)
    output_angle += 360;

  // printf("%d\n", output_angle);
  free(v_output);

  // finally, execute command
  stop();
  rotate_robot(output_angle);
}
*/

void robot_control()
{
  // TODO
	Obj * detected = process_input();
	Obj * orig = detected;
	while((*detected).type != -1) {
		printf("Detected: type %d, position (%f, %f)\n",
						(*detected).type, (*detected).x, (*detected).y);

		detected++;
	}
	free(orig);
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  struct Robot robot_info = {100,100};
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

    robot_control();


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
