#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

int* findFood(void)
{
	int* food = malloc(sizeof(int)*5);
	food[0] = 1;
	food[1] = 2;
	food[2] = 300;
	food[3] = 4;
	food[4] = 5;

	return food;
}

int* avoidZombies(void)
{
	int* avoid = malloc(sizeof(int)*5);
	avoid[0] = 1;
	avoid[1] = 2;
	avoid[2] = 3;
	avoid[3] = 400;
	avoid[4] = 5;

	return avoid;
}

int* knockBerryDown(void)
{
	int* knock = malloc(sizeof(int)*5);
	knock[0] = 0;
	knock[1] = 0;
	knock[2] = 0;
	knock[3] = 0;
	knock[4] = 10000;

	return knock;
}

int* explore(void)
{
	int* explore = malloc(sizeof(int)*5);
	explore[0] = 1;
	explore[1] = 0;
	explore[2] = 3;
	explore[3] = 4;
	explore[4] = 5;

	return explore;
}

int* avoidObstacles(void)
{
	int* obstacles = malloc(sizeof(int)*5);
	obstacles[0] = 1;
	obstacles[1] = 0;
	obstacles[2] = 3;
	obstacles[3] = 4;
	obstacles[4] = 5;

	return obstacles;
}


// Arbiter decides which action to take
// For now ties go to the first tie one in array *CAN BE CHANGED*
// [0]: forward, [1]: back, [2]: left, [3]: right, [4]: do nothing
void arbiter(void)
{
	int* foodVote = findFood();
	int* avoidObstaclesVote = avoidObstacles();
	int* exploreVote = explore();
	int* knockBerryVote = knockBerryDown();
	int* avoidZombiesVote = avoidZombies();

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
	arbiter();
	printf("Hello, World!\n");
	return 0;
}
