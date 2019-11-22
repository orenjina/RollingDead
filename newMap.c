#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#define MAX(x, y) (((x) > (y)) ? (x) : (y))


struct mapNode
{
	char *object;
	char *placeholderObject;
	int riskFactor;
	bool empty;
};

struct mapNode ***Map;
int centerX, centerY;
int MAP_WIDTH;
int MAP_HEIGHT;
int cameraMaxView = 10;


// TODO: Expand map when near edge here aswell?
void clearSector(char *direction)
{
	// Check for obstacles, end of map, and end of sight: (can reproduce for down/left/right)
	if (!strcmp(direction, "UP"))
	{
		// Loop through each y value: (stop at max range or end of map)
		for (int y = 1; y < cameraMaxView && centerY - y > -1; y++)
		{
			// find the left bound and right bound: (again make sure don't hit edge)
			int leftBound = ceil(((float)y)/-1.83);
			int rightBound = ceil(((float)y)/1.83);

			int leftConverted = MAX(leftBound + centerX, 0);
			int rightConverted = rightBound + centerX;

			// Remove all the objects in this row segment
			for (int k = leftConverted; k < rightConverted + 1 && k < MAP_WIDTH; k++)
			{
				if (Map[centerY - y][k]->object != NULL)
				{
					printf("clearSector function: Object %s at (%i,%i) was removed\n", Map[centerY - y][k]->object, centerY - y, k);
					Map[centerY - y][k]->object = NULL;
					Map[centerY - y][k]->empty = true;
				}
			}
		}
	}
	else if (!strcmp(direction, "DOWN"))
	{
		// Loop through each y value: (stop at max range or end of map)
		for (int y = -1; y > -1*cameraMaxView && centerY - y < MAP_HEIGHT; y--)
		{
			// find the left bound and right bound: (again make sure don't hit edge)
			int leftBound = ceil(((float)y)/1.83);
			int rightBound = ceil(((float)y)/-1.83);

			int leftConverted = MAX(leftBound + centerX, 0);
			int rightConverted = rightBound + centerX;

			// Remove all the objects in this row segment
			for (int k = leftConverted; k < rightConverted + 1 && k < MAP_WIDTH; k++)
			{
				if (Map[centerY - y][k]->object != NULL)
				{
					printf("clearSector function: Object %s at (%i,%i) was removed\n", Map[centerY - y][k]->object, centerY - y, k);
					Map[centerY - y][k]->object = NULL;
					Map[centerY - y][k]->empty = true;
				}
			}
		}
	}
	else if (!strcmp(direction, "LEFT"))
	{
		// Loop through each y value: (stop at max range or end of map)
		for (int x = -1; x > -1*cameraMaxView && centerX + x > -1; x--)
		{
			// find the left bound and right bound: (again make sure don't hit edge)
			int upBound = ceil(((float)x/-1.83));
			int downBound = ceil(((float)x)/1.83);

			int upConverted = upBound + centerX;
			int downConverted = MAX(downBound + centerX, 0);

			// Remove all the objects in this row segment
			for (int k = downConverted; k < upConverted + 1 && k < MAP_HEIGHT; k++)
			{
				if (Map[k][centerX + x]->object != NULL)
				{
					printf("clearSector function: Object %s at (%i,%i) was removed\n", Map[k][centerX + x]->object, centerX + x, k);
					Map[k][centerX + x]->object = NULL;
					Map[k][centerX + x]->empty = true;
				}
			}
		}
	}
	else if (!strcmp(direction, "RIGHT"))
	{
		for (int x = 1; x < cameraMaxView && centerX + x < MAP_WIDTH; x++)
		{
			// find the left bound and right bound: (again make sure don't hit edge)
			int upBound = ceil(((float)x/1.83));
			int downBound = ceil(((float)x)/-1.83);

			int upConverted = upBound + centerX;
			int downConverted = MAX(downBound + centerX, 0);

			// Remove all the objects in this row segment
			for (int k = downConverted; k < upConverted + 1 && k < MAP_HEIGHT; k++)
			{
				if (Map[k][centerX + x]->object != NULL)
				{
					printf("clearSector function: Object %s at (%i,%i) was removed\n", Map[k][centerX + x]->object, centerX + x, k);
					Map[k][centerX + x]->object = NULL;
					Map[k][centerX + x]->empty = true;
				}
			}
		}
	}
}

// When see edge of map need to expand in that direction
void expand(int expansionAmount, char *direction)
{
	if (!strcmp(direction, "UP"))
	{
		// increase rows by expansionAmount
		int oldHeight = MAP_HEIGHT;
		centerY += expansionAmount;
		MAP_HEIGHT += expansionAmount;
		Map = realloc(Map, sizeof(struct mapNode **)*MAP_HEIGHT);

		// alloc storage for all the new slots
		for (int i = oldHeight - 1; i < MAP_HEIGHT; i++)
		{
			Map[i] = malloc(sizeof(struct mapNode *)*MAP_WIDTH);
			for (int j = 0; j < MAP_WIDTH; j++)
			{
				Map[i][j] = malloc(sizeof(struct mapNode));
				Map[i][j]->empty = true;
				Map[i][j]->object = NULL;
				Map[i][j]->placeholderObject = NULL;
			}
		}

		// shift everything down
		for (int i = 0; i < oldHeight; i++)
		{
			for (int j = 0; j < MAP_WIDTH; j++)
			{
				// copy the current object out first
				if (Map[i][j]->object != NULL)
				{
					Map[i + expansionAmount][j]->placeholderObject = Map[i][j]->object;
					Map[i][j]->object = NULL;
					Map[i][j]->empty = true;
				}

				// then move current placeholder to the object spot
				if (Map[i][j]->placeholderObject != NULL)
				{
					Map[i][j]->object = Map[i][j]->placeholderObject;
					Map[i][j]->placeholderObject = NULL;
					Map[i][j]->empty = false;
				}
			}
		}

		// new charted land could have placeholderObjects
		for (int i = oldHeight - 1; i < MAP_HEIGHT; i++)
		{
			for (int j = 0; j < MAP_WIDTH; j++)
			{
				// then move current placeholder to the object spot
				if (Map[i][j]->placeholderObject != NULL)
				{
					Map[i][j]->object = Map[i][j]->placeholderObject;
					Map[i][j]->placeholderObject = NULL;
					Map[i][j]->empty = false;
				}
			}
		}

	}
	else if (!strcmp(direction, "LEFT"))
	{
		// increase rows by expansionAmount
		int oldWdith = MAP_WIDTH;
		centerX += expansionAmount;
		MAP_WIDTH += expansionAmount;

		// alloc storage for all the new slots
		for (int i = 0; i < MAP_HEIGHT; i++)
		{
			Map[i] = realloc(Map[i],sizeof(struct mapNode *)*MAP_WIDTH);
			for (int j = oldWdith - 1; j < MAP_WIDTH; j++)
			{
				Map[i][j] = malloc(sizeof(struct mapNode));
				Map[i][j]->empty = true;
				Map[i][j]->object = NULL;
				Map[i][j]->placeholderObject = NULL;
			}
		}

		// shift everything right
		for (int i = 0; i < MAP_HEIGHT; i++)
		{
			for (int j = 0; j < oldWdith; j++)
			{
				// copy the current object out first
				if (Map[i][j]->object != NULL)
				{
					Map[i][j + expansionAmount]->placeholderObject = Map[i][j]->object;
					Map[i][j]->object = NULL;
					Map[i][j]->empty = true;
				}

				// then move current placeholder to the object spot
				if (Map[i][j]->placeholderObject != NULL)
				{
					Map[i][j]->object = Map[i][j]->placeholderObject;
					Map[i][j]->placeholderObject = NULL;
					Map[i][j]->empty = false;
				}
			}
		}

		// new charted land could have placeholderObjects
		for (int i = 0; i < MAP_HEIGHT; i++)
		{
			for (int j = oldWdith - 1; j < MAP_WIDTH; j++)
			{
				// then move current placeholder to the object spot
				if (Map[i][j]->placeholderObject != NULL)
				{
					Map[i][j]->object = Map[i][j]->placeholderObject;
					Map[i][j]->placeholderObject = NULL;
					Map[i][j]->empty = false;
				}
			}
		}

	}
	else if (!strcmp(direction, "DOWN"))
	{
		int oldHeight = MAP_HEIGHT;
		MAP_HEIGHT += expansionAmount;
		Map = realloc(Map, sizeof(struct mapNode **)*MAP_HEIGHT);

		// alloc storage for all the new slots
		for (int i = oldHeight - 1; i < MAP_HEIGHT; i++)
		{
			Map[i] = malloc(sizeof(struct mapNode *)*MAP_WIDTH);
			for (int j = 0; j < MAP_WIDTH; j++)
			{
				Map[i][j] = malloc(sizeof(struct mapNode));
				Map[i][j]->empty = true;
				Map[i][j]->object = NULL;
				Map[i][j]->placeholderObject = NULL;
			}
		}
	}
	else if (!strcmp(direction, "RIGHT"))
	{
		// increase rows by expansionAmount
		int oldWdith = MAP_WIDTH;
		MAP_WIDTH += expansionAmount;

		// alloc storage for all the new slots
		for (int i = 0; i < MAP_HEIGHT; i++)
		{
			Map[i] = realloc(Map[i],sizeof(struct mapNode *)*MAP_WIDTH);
			for (int j = oldWdith - 1; j < MAP_WIDTH; j++)
			{
				Map[i][j] = malloc(sizeof(struct mapNode));
				Map[i][j]->empty = true;
				Map[i][j]->object = NULL;
				Map[i][j]->placeholderObject = NULL;
			}
		}
	}
	else
	{
		exit(fprintf(stderr, "invalid direction given in expand()\n"));
	}
}

void printMap(void)
{
	for (int i = 0; i < MAP_HEIGHT; i++)
	{
		for (int j = 0; j < MAP_WIDTH; j++)
		{
			int x , y;

			x = (j - centerX); // column = centerX + x;
			y = (centerY - i); // row = centerY - y;

			if (Map[i][j]->empty)
			{
				printf("(%3i,%3i):  ",x,y);
			}
			else if (!strcmp(Map[i][j]->object, "b"))
			{
				printf("(%3i,%3i):",x,y);
				printf("\033[1;31m");
				printf("B ");
				printf("\033[0m");
			}
			else
			{
				printf("(%3i,%3i):",x,y);
				printf("\033[1;32m");
				printf("Z ");
				printf("\033[0m");
			}
		}
		printf("\n");
	}
}



// Input: {object, x, y}
void addToMap(char *object, int x, int y)
{
	int column = centerX + x;
	int row = centerY - y;

	Map[row][column]->empty = false;
	Map[row][column]->object = object;
}

void initialize(int height, int width)
{
	MAP_HEIGHT = height;
	MAP_WIDTH = width;
	centerX = width/2 - 1;
	centerY = height/2 - 1;

	// Initialize map: (create a function for it)
	Map = malloc(sizeof(struct mapNode **)*MAP_HEIGHT);
	for (int i = 0; i < MAP_HEIGHT; i++)
	{
		Map[i] = malloc(sizeof(struct mapNode *)*MAP_WIDTH);
		for (int j = 0; j < MAP_WIDTH; j++)
		{
			Map[i][j] = malloc(sizeof(struct mapNode));
			Map[i][j]->empty = true;
			Map[i][j]->object = NULL;
			Map[i][j]->placeholderObject = NULL;
		}
	}
}



int main(int argc, char **argv)
{
	initialize(10,10);

	// For sake of testing, take in command line arguments, and call function to input those objects in those arrays
	bool zombie = false, berry = false, findX = false, findY = false, findName = true;
	int x, y, zCounter = 0, bCounter = 0;
	char *pEnd;
	for (int i = 1; i < argc; i++)
	{
		if (findName)
		{
			if (!strcmp(argv[i], "b"))
			{
				berry = true;
				bCounter++;
			} else if (!strcmp(argv[i], "z"))
			{
				zombie = true;
				zCounter++;
			} else
			{
				exit(fprintf(stderr, "incorrect command args\n"));
			}
			findName = false;
			findX = true;
		}
		else if (findX)
		{
			findX = false;
			findY = true;
			x = (int)strtol(argv[i],&pEnd, 10);
		}
		else if (findY)
		{
			findY = false;
			findName = true;
			y = (int)strtol(argv[i],&pEnd, 10);

			if (zombie)
			{
				addToMap("z", x, y);
				zombie = false;
			}
			else
			{
				addToMap("b", x, y);
			}
		}
		else
		{
			exit(fprintf(stderr, "incorrect command args 2\n"));
		}
	}

	//printMap();
	//expand(3, "LEFT");
	//expand(3, "DOWN");
	//expand(20, "UP");
	//printf("\n\n");
	printMap();
	printf("\n");
	clearSector("RIGHT");
	printf("\n");
	printMap();

	// Standard 2D array:

	// Case where see two things on top of eachother:
		// Put both in the same box? Or put one one row behind the other box

	return 0;
}
