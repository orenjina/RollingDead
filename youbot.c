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
#define FOCAL_LENGTH (120)

enum types {blu_zombie, aqu_zombie, gre_zombie, pur_zombie,
          red_berry, yel_berry, ora_berry, pin_berry,
          stump, tree, wall, edge};

typedef struct observation {
  // typecode
  int type;
  // cam id
  int cam;
  // bounding box info
  int ymin;
  int ymax;
  int xmin;
  int xmax;
} Obsv;

// utility functions
float
mmin(float a, float b, float c)
{
  a = fminf(a, b);
  c = fminf(a, c);
  return c;
}
float
mmax(float a, float b, float c)
{
  a = fmaxf(a, b);
  c = fmaxf(a, c);
  return c;
}
//////

// more structs //
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

typedef struct position {
  float x;
  float y;
} Pos, *Posi;
//////

void
rgbToHsv( RgbColor  * rgb, HsvColor * hsv)
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

int
inHueRange(HsvColor * hsv, int hue1, int hue2)
{
  float sat_min = 0.7;
  return(hsv->h >= hue1 && hsv->h <= hue2 && hsv->s >= sat_min);
}

void
obsvUpdate(Obsv * obs, int type, int x, int y)
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

float
estimateVerticalDistance(Obsv * obj)
{
  float y_true = 1.77;
  int y_px = obj->ymax - obj->ymin;

  return((float) y_true * FOCAL_LENGTH / y_px);
}

Pos
calculateXYpos(Obsv * obj, float z)
{
  Pos res;

  int x_avg = (obj->xmax + obj->xmin) / 2 - 64;
  // cam origin = (x=64, y=32)

  float theta = atan((float) x_avg / FOCAL_LENGTH);

  res.x = z * sin(theta);
  res.y = z * cos(theta);

  return res;
}

void
publishObservations(Obsv * objs)
{
  float d;
  Pos est;
	Obsv * obj;
  // objs is a 12-item array
  for(int i = 0; i < 12; i++) {
    if(objs[i].type != -1) {
			obj = &objs[i];
      d = estimateVerticalDistance(&(objs[i]));
      est = calculateXYpos(&(objs[i]), d);
			// printf("y_min %d, y_max %d\n", obj->ymin, obj->ymax);
      printf("Zombie detected: distance %f, est position (%f, %f)\n", d, est.x, est.y);
    }
  }
}

void robot_control()
{
	////////////// TO GET RGB FROM THE CAMERA ///////////////////////////////////////////////////
  Obsv objs[12];
  int type_id;

	RgbColor rgb;
	HsvColor hsv;

	for (int i = 0; i < 12; i++) {
		objs[i].type = -1;
	}

	const unsigned char * image = wb_camera_get_image(4);

	for (int x = 0; x < 128; x++) {

		for (int y = 0; y < 64; y++) {
      type_id = -1;

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

      // TODO: implement it so it can tell the difference between two same-color zombies
      if(inHueRange(&hsv, 120, 132))        type_id = gre_zombie;
      else if (inHueRange(&hsv, 250, 280))  type_id = pur_zombie;
      else if (inHueRange(&hsv, 165, 185))  type_id = aqu_zombie;
      else if (inHueRange(&hsv, 200, 225))  type_id = blu_zombie;

      if(type_id != -1)
				printf("color: %d,\n hue, sat (%f, %f)\n xy %d, %d\n",
								type_id, hsv.h, hsv.s, x, y);
        obsvUpdate(&(objs[type_id]), type_id, x, y);

      // do something with the observations

      publishObservations(objs);
		}
	}


	/////////////////////////////////////////////////////////////////////////////////////////////
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
  wb_gps_enable(2,TIME_STEP);
  // wb_compass_enable(3,TIME_STEP);
  wb_camera_enable(4,TIME_STEP); // front
  // wb_camera_enable(5,TIME_STEP);
  // wb_camera_enable(6,TIME_STEP);
  // wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP); // back
  wb_camera_enable(9,TIME_STEP); // right
  // wb_camera_enable(10,TIME_STEP); // left
  // wb_camera_enable(11,TIME_STEP);
  // wb_gyro_enable(12,TIME_STEP);
  // wb_light_sensor_enable(13,TIME_STEP);
  // wb_receiver_enable(14,TIME_STEP);
  // wb_range_finder_enable(15,TIME_STEP);
  // wb_lidar_enable(16,1);

  //WbDeviceTag lidar = wb_robot_get_device("lidar");
  //wb_lidar_enable_point_cloud(lidar);

  //WbDeviceTag rec = wb_robot_get_device("receiver");

  int i = 0;

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

    if (i < 100)
    {
    	base_forwards();
    }
    if (i == 100)
    {
    	base_reset();
    	base_turn_left();
    }
    if (i == 300)
    {
    	i = 0;
    }
    i++;

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
