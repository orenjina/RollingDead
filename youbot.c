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

#include <webots/camera.h>
// #include <webots/accelerometer.h>
// #include <webots/lidar.h>
// #include <webots/compass.h>
#include <webots/gps.h>
// #include <webots/range_finder.h>
// #include <webots/gyro.h>
// #include <webots/light_sensor.h>
// #include <webots/receiver.h>
// #include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>

//void wb_camera_enable(WbDeviceTag tag, int sampling_period);
//void wb_camera_disable(WbDeviceTag tag);
//int wb_camera_get_sampling_period(WbDeviceTag tag);

int robot_angle = 0;
#define TIME_STEP 32

void rotate_robot(int angle)
{
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
    WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    double rotation[4];
    if (angle == 0) { rotation[0] = 1; rotation[1] = 0; rotation[2] = 0; rotation[3] = -1.57; }
    if (angle == 15) { rotation[0] = -0.985; rotation[1] = 0.126; rotation[2] = 0.122; rotation[3] = 1.59; }
    if (angle == 30) { rotation[0] = -0.938; rotation[1] = 0.247; rotation[2] = 0.244; rotation[3] = 1.63; }
    if (angle == 45) { rotation[0] = -0.866; rotation[1] = 0.355; rotation[2] = 0.352; rotation[3] = 1.71; }
    if (angle == 60) { rotation[0] = -0.778; rotation[1] = 0.445; rotation[2] = 0.443; rotation[3] = 1.82; }
    if (angle == 75) { rotation[0] = -0.681; rotation[1] = 0.519; rotation[2] = 0.516; rotation[3] = 1.94; }
    if (angle == 90) { rotation[0] = -0.581; rotation[1] = 0.577; rotation[2] = 0.572; rotation[3] = 2.09; }
    if (angle == 105) { rotation[0] = -0.48; rotation[1] = 0.621; rotation[2] = 0.619; rotation[3] = 2.24; }
    if (angle == 120) { rotation[0] = -0.381; rotation[1] = 0.654; rotation[2] = 0.653; rotation[3] = 2.41; }
    if (angle == 135) { rotation[0] = -0.284; rotation[1] = 0.679; rotation[2] = 0.677; rotation[3] = 2.58; }
    if (angle == 150) { rotation[0] = -0.189; rotation[1] = 0.695; rotation[2] = 0.694; rotation[3] = 2.76; }
    if (angle == 165) { rotation[0] = -0.095; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = 2.95; }
    if (angle == 180) { rotation[0] = -0.00268959; rotation[1] = 0.707126; rotation[2] = 0.707083; rotation[3] = 3.13102; }
    if (angle == 195) { rotation[0] = 0.090; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = -2.97; }
    if (angle == 210) { rotation[0] = 0.183; rotation[1] = 0.695; rotation[2] = 0.695; rotation[3] = -2.78; }
    if (angle == 225) { rotation[0] = 0.278; rotation[1] = 0.679; rotation[2] = 0.680; rotation[3] = -2.6; }
    if (angle == 240) { rotation[0] = 0.375; rotation[1] = 0.655; rotation[2] = 0.656; rotation[3] = -2.43; }
    if (angle == 255) { rotation[0] = 0.473; rotation[1] = 0.622; rotation[2] = 0.624; rotation[3] = -2.26; }
    if (angle == 270) { rotation[0] = 0.574; rotation[1] = 0.578; rotation[2] = 0.580; rotation[3] = -2.10; }
    if (angle == 285) { rotation[0] = 0.674; rotation[1] = 0.521; rotation[2] = 0.524; rotation[3] = -1.96; }
    if (angle == 300) { rotation[0] = 0.771; rotation[1] = 0.449; rotation[2] = 0.452; rotation[3] = -1.83; }
    if (angle == 315) { rotation[0] = 0.860; rotation[1] = 0.360; rotation[2] = 0.363; rotation[3] = -1.72; }
    if (angle == 330) { rotation[0] = 0.933; rotation[1] = 0.254; rotation[2] = 0.257; rotation[3] = -1.64; }
    if (angle == 345) { rotation[0] = 0.982; rotation[1] = 0.133; rotation[2] = 0.137; rotation[3] = -1.59; }
    wb_supervisor_field_set_sf_rotation(rot_field,rotation);
    robot_angle = angle;
}


void go_forward()
{
	base_forwards();
}

void stop()
{
	base_reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ANGLE_UNIT (45) // what angle we should round to for turning commands
#define FOCAL_LENGTH (120) // camera focal length
#define MAX_OBJS (25) // max # of detected objects in FOV

int cameras[3] = {4, 8, 9};

/* TYPEDEFS */
enum types {blu_zombie, aqu_zombie, gre_zombie, pur_zombie,
          red_berry, yel_berry, ora_berry, pin_berry,
          stump, tree, wall, edge};

typedef struct img_observation {
  int type;  // typecode
  int cam;   // cam id
  // bounding box info
  int ymin;
  int ymax;
  int xmin;
  int xmax;
} Obsv;

typedef struct vector {
  float x;
  float y;
} * Vector;

typedef struct position {
	int type;
  float x;
  float y;
} Pos, *Posi;

typedef struct robot_position {
  float x;
  float y;
  // int angle;
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
/////

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

/* converts rgb color codes to hsv */
void rgbToHsv( RgbColor  * rgb, HsvColor * hsv)
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

/* returns true if hsv color is in hue range with sat >= sat_min */
int inHueRange(HsvColor * hsv, int hue1, int hue2)
{
  float sat_min = 0.7;
  return(hsv->h >= hue1 && hsv->h <= hue2 && hsv->s >= sat_min);
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
///////

/* INFO PROCESSING FUNCTIONS */
float estimateVerticalDistance(Obsv * obj)
{
  float y_true = 1.77;
  int y_px = obj->ymax - obj->ymin;

  return((float) y_true * FOCAL_LENGTH / y_px);
}

/* updates Position with info on object position */
void calculateXYpos(Obsv * obj, Pos * pos, float z, int cam_id)
{
  int x_avg = (obj->xmax + obj->xmin) / 2 - 64;
  // cam origin = (x=64, y=32)

  float theta = atan((float) x_avg / FOCAL_LENGTH);

  // pos->x = z * sin(theta);
  // pos->y = z * cos(theta);
	pos->type = obj->type;

	switch (cam_id) {
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

void obsvUpdate(Obsv * obs, int type, int x, int y)
{
  if(obs->type == -1) {
    obs->type = type;
    obs->xmax = x;
    obs->xmin = x;
    obs->ymax = y;
    obs->ymin = y;

  } else {
    if(x > obs->xmax) obs->xmax = x;
    else if (x < obs->xmin) obs->xmin = x;

    if(y > obs->ymax) obs->ymax = y;
    else if (y < obs->ymin) obs->ymin = y;
  }
}

/* main image processing for single camera */
void processSingleImage(int cam_id, Obsv * objs)
{
	int type;
	RgbColor rgb;
	HsvColor hsv;

	const unsigned char * image = wb_camera_get_image(cam_id);

	for (int x = 0; x < 128; x++) {
		for (int y = 0; y < 64; y++) {
      type = -1;

			int r = wb_camera_image_get_red(image, 128, x, y);
			int g = wb_camera_image_get_green(image, 128, x, y);
			int b = wb_camera_image_get_blue(image, 128, x, y);
			// printf("red=%d, green=%d, blue=%d \n", r, g, b);

      RgbColor rgb;
      HsvColor hsv;

      rgb.r = (float) r / 255.0;
      rgb.g = (float) g / 255.0;
      rgb.b = (float) b / 255.0;

      rgbToHsv(&rgb, &hsv);

      // TODO: implement telling the difference between two same-color objects
      if(inHueRange(&hsv, 120, 132))        type = gre_zombie;
      else if (inHueRange(&hsv, 250, 280))  type = pur_zombie;
      else if (inHueRange(&hsv, 165, 185))  type = aqu_zombie;
      else if (inHueRange(&hsv, 200, 225))  type = blu_zombie;

      if(type != -1)
				// printf("color: %d,\n hue, sat (%f, %f)\n xy %d, %d\n",
				// 				type, hsv.h, hsv.s, x, y);
        obsvUpdate(&(objs[type]), type, x, y);
		}
	}
}

/* main image processing for the 3 main cams */
Pos * imageProcess()
/* returns a malloc'd list of positions (terminated by one with type -1) */
{
	Obsv observations[12];
	float d;
	Pos * results;

	results = malloc(MAX_OBJS * sizeof(Pos));

	int len = 0;

	// process each camera image
	for(int j = 0; j < 3; j++) {
		// reset observation space
		for(int i=0; i < 12; i++) observations[i].type = -1;

		// get latest info
		processSingleImage(cameras[j], observations);

		// process info and append to final
		for(int i=0; i < 12; i++) {
			if(observations[i].type != -1) {

				d = estimateVerticalDistance(&(observations[i]));
				calculateXYpos(&(observations[i]), &(results[len++]), d, cameras[j]);
			}
		}
	}
	results[len].type = -1;
	return results;
}

//////

/* BEHAVIOR FUNCTIONS */

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

/* MAIN CONTROL */
void robot_control()
{
  // RobotPos current_pos;
  // current_pos.x = 0;
  // current_pos.y = 0;
	//
  // Pos obs;
  // obs.x = 0;
  // obs.y = 1;
	//
  // Posi zombie[3];
  // zombie[0]->x = -2;
  // zombie[0]->y = -3;
	//
  // zombie[1]->x = 0;
  // zombie[1]->y = -2;
	//
  // zombie[2] = NULL;
	//
  // // Probably only looking for 1 food at a time
  // Pos food;
  // food.x = 2;
  // food.y = 2;
	//
  // arbiter(current_pos, obs, zombie, food);

	Pos * detected = imageProcess();
	Pos * orig = detected;
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

  // display_helper_message();

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
    robot_control();
    go_forward();
    //stop();

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
