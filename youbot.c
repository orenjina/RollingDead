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
#include <webots/compass.h>
#include <webots/gps.h>

#include <youbot_zombie_1.h>

void wb_camera_enable(WbDeviceTag tag, int sampling_period);
void wb_camera_disable(WbDeviceTag tag);
int wb_camera_get_sampling_period(WbDeviceTag tag);

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




void robot_control()
{
	////////////// TO ROTATE THE ROBOT (BETWEEN 0 - 345) WITH 15 DEGREE INTERVALS ///////////////
	rotate_robot(45);
	rotate_robot(255);
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////// TO MOVE ROBOT FORWARD AND TO STOP IT /////////////////////////////////////////
	go_forward();
	stop();
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////// TO GET RGB FROM THE CAMERA ///////////////////////////////////////////////////
	const unsigned char *image = wb_camera_get_image(3);
	for (int x = 0; x < 128; x++)
	{
		for (int y = 0; y < 64; y++) 
		{
			int r = wb_camera_image_get_red(image, 64, x, y);
			int g = wb_camera_image_get_green(image, 64, x, y);
			int b = wb_camera_image_get_blue(image, 64, x, y);
			//printf("red=%d, green=%d, blue=%d", r, g, b);
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
  
  wb_camera_enable(3,1);

  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  get_all_berry_pos();
  
  int robot_not_dead = 1;
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// DECLARE LOCAL VARIABLES HERE ONLY //////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
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
	if (timer % 8 == 0)
	{  
		const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
		check_berry_collision(&robot_info, trans[0], trans[2]);
		check_zombie_collision(&robot_info, trans[0], trans[2]);
	}
    if (timer == 64)
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
    
    
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
