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

int* findFood(void)
{
	int* food = malloc(sizeof(int)*5);
	food[0] = 0;
	food[1] = 0;
	food[2] = 0;
	food[3] = 0;
	food[4] = 0;

	return food;
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

int* explore(void)
{
	int* explore = malloc(sizeof(int)*5);
	explore[0] = 0;
	explore[1] = 0;
	explore[2] = 0;
	explore[3] = 0;
	explore[4] = 0;

	return explore;
}

int* avoidObstacles(void)
{
	int* obstacles = malloc(sizeof(int)*5);
	obstacles[0] = 0;
	obstacles[1] = 0;
	obstacles[2] = 0;
	obstacles[3] = 0;
	obstacles[4] = 0;

	return obstacles;
}


// Arbiter decides which action to take
// For now ties go to the first tie one in array *CAN BE CHANGED*
// [0]: forward, [1]: back, [2]: left, [3]: right, [4]: do nothing
//
// Gets all arguments from sensors and internal map
// Parameter can be further explained here
void arbiter(RobotPos robot, Posi zombie, int zombie_len)
{
	int* foodVote = findFood();
	int* avoidObstaclesVote = avoidObstacles();
	int* exploreVote = explore();
	int* knockBerryVote = knockBerryDown();
	int* avoidZombiesVote = avoidZombies(robot, zombie, zombie_len);

	// for (int i = 0; i < 5; i++) {
	// 	printf("%d\n", avoidZombiesVote[i]);
	// }

	int* finalVotes = malloc(sizeof(int)*5);
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

  RobotPos current_pos;
  current_pos.x = 0;
  current_pos.y = 0;
	current_pos.angle = 0;

	Posi zombie = (Posi) malloc(2 * sizeof(struct position));
  zombie[0].x = -2;
  zombie[0].y = -3;

	// printf("hola, world\n");

  zombie[1].x = -3;
  zombie[1].y = -4;

	// printf("sup, world\n");

	arbiter(current_pos, zombie, 2);
	printf("Hello, World!\n");
	return 0;
}
