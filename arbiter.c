#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// the ratio of time to turn vs time to travel a meter
#define TURN_FACTOR (3)

typedef struct vector {
  float x;
  float y;
} * Vector;

typedef struct position {
  int x;
  int y;
} Pos, *Posi;

typedef struct robot_position {
  int x;
  int y;
  int angle; // 0 up, 1 right, 2 down, 3 left
} RobotPos;

// random number between lower and upper
int randomN(int lower, int upper)
{
    int num = (rand() % (upper - lower + 1)) + lower;
    return num;
}

// Adds the given array of vectors
Vector vector_sum(Vector vectors, int len)
// must be null-terminated
// also frees the input vectors along the way
{
  Vector sum = malloc(sizeof(struct vector));
  sum->x = 0;
  sum->y = 0;

  for (int i = 0; i < len; i++) {
    sum->x += (vectors)->x;
    sum->y += (vectors)->y;
    vectors++;
  }

  return sum;
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

// // Simple calculation on the angle between the robot direction and the given
// // angle. Assume angle in degrees
// double angleBetween(int dir, double angle) {
// 	double ret = ((int)(360 + dir * 90 - angle)) % 360;
// 	if (ret > 180) {
// 		ret -= 360;
// 	}
// 	return ret;
// }


// Do similar things as zombies but for food
// So same structure but different parameters
int* findFood(RobotPos robot, Posi food, int len)
{
	printf("finding berries\n");
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

  for (int i = 0; i < len; i++) {
		// Temporary function on the amount of influence berries have
		// given the distance
    v->x += (-2.0 / sqrt(robot.x - (food)->x + 1));
    v->y += (-2.0 / sqrt(robot.y - (food)->y + 1));
    food++;
	}

	// printf("v->x: %f\n", v->x);
	// printf("v->y: %f\n", v->y);

	int* get = malloc(sizeof(int)*5);


	// formulas subjected to tweaking
	get[0] = proj(v, robot.angle);
	get[1] = proj(v, reverse(robot.angle));
	// convenient calculation for the sides
	get[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	get[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	get[4] = 0;

	return get;
}

// Compute vector of the given zombies,
// but a multiplying factor is at work as well.
int* avoidZombies(RobotPos robot, Posi zombie, int len)
{
	printf("avoiding zombies\n");
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

  for (int i = 0; i < len; i++) {
		// Temporary function on the amount of influence a zombie has
		// given the distance
    v->x += (2.0 / sqrt(robot.x - (zombie)->x + 1));
    v->y += (2.0 / sqrt(robot.y - (zombie)->y + 1));
    zombie++;
	}

	// printf("v->x: %f\n", v->x);
	// printf("v->y: %f\n", v->y);

	int* avoid = malloc(sizeof(int)*5);


	// formulas subjected to tweaking
	avoid[0] = proj(v, robot.angle);
	avoid[1] = proj(v, reverse(robot.angle));
	// convenient calculation for the sides
	avoid[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	avoid[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	avoid[4] = 0;

	return avoid;
}

int* knockBerryDown(void)
{
	int* knock = malloc(sizeof(int)*5);
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

int* avoidObstacles(RobotPos robot, Posi obs, int len)
{
	printf("obstacle avoiding\n");
  Vector v;
  v = malloc(sizeof(struct vector)); // must be freed

  v->x = 0;
  v->y = 0;

  for (int i = 0; i < len; i++) {
		// Temporary function on the amount of influence obstacles have
		// given the distance
    v->x += (0.8 / sqrt(robot.x - (obs)->x + 1));
    v->y += (0.8 / sqrt(robot.y - (obs)->y + 1));
    obs++;
	}

	// printf("v->x: %f\n", v->x);
	// printf("v->y: %f\n", v->y);

	int* avoid = malloc(sizeof(int)*5);


	// formulas subjected to tweaking
	avoid[0] = proj(v, robot.angle);
	avoid[1] = proj(v, reverse(robot.angle));
	// convenient calculation for the sides
	avoid[2] = proj(v, reverse(robot.angle + 1)) - TURN_FACTOR;
	avoid[3] = proj(v, reverse(robot.angle + 3)) - TURN_FACTOR;
	avoid[4] = 0;

	return avoid;
}


// Arbiter decides which action to take
// For now ties go to the first tie one in array *CAN BE CHANGED*
// [0]: forward, [1]: back, [2]: left, [3]: right, [4]: do nothing
//
// Gets all arguments from sensors and internal map
// Parameter can be further explained here
void arbiter(RobotPos robot, Posi zombie, int zombie_len, Posi food, int food_len, Posi obs, int obs_len)
{
	int* foodVote = findFood(robot, food, food_len);
	int* avoidObstaclesVote = avoidObstacles(robot, obs, obs_len);
	double* exploreVote = explore();
	int* knockBerryVote = knockBerryDown();
	int* avoidZombiesVote = avoidZombies(robot, zombie, zombie_len);

	// for (int i = 0; i < 5; i++) {
	// 	printf("%d\n", avoidZombiesVote[i]);
	// }

	double* finalVotes = malloc(sizeof(double)*5);
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
		// go_forward();
	}
	else if (winningIndex == 1)
	{
		printf("backwards won\n");
		// go_backward();
	}
	else if (winningIndex == 2)
	{
		printf("left won\n");
		// turn_left();
	}
	else if (winningIndex == 3)
	{
		printf("right won\n");
		// turn_right();
	}
	else if (winningIndex == 4)
	{
		printf("do nothing won\n");
		// stop();
	}


}



int main()
{
	// Robot current position. Center and facing front
  RobotPos current_pos;
  current_pos.x = 0;
  current_pos.y = 0;
	current_pos.angle = 0;

	// Zombie testing parameters
	Posi zombie = (Posi) malloc(2 * sizeof(struct position));
  zombie[0].x = -2;
  zombie[0].y = -3;

	// printf("hola, world\n");

  zombie[1].x = -3;
  zombie[1].y = -4;

	// printf("sup, world\n");

	// Food testing parameters
	Posi food = (Posi) malloc(2 * sizeof(struct position));
  food[0].x = 2;
  food[0].y = 1;

  food[1].x = -1;
  food[1].y = 4;

	arbiter(current_pos, zombie, 2, food, 2, zombie, 2);
	// printf("Hello, World!\n");
	return 0;
}
