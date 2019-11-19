#include<stdio.h>
#include<stdlib.h>
// Estimated ratio of turn time versus grid travel time
#define TURN 10

void astar(mapNode*** G,int n, int m, int x1, int y1, int dir, int x2, int y2);

typedef struct LinkedList {
	int x;
	int y;
	int dir; // 0 verticle, 1 horizontal
	double t; // time elapsed
	double h; // heuristic function value
	int ini; // initial action, left, right, or turn
	struct LinkedList* next; // next node
} *node;

int main()
{
	int G[MAX][MAX],i,j,n,u;
	printf("Enter no. of vertices:");
	scanf("%d",&n);
	printf("\nEnter the adjacency matrix:\n");

	for(i=0;i<n;i++)
		for(j=0;j<n;j++)
			scanf("%d",&G[i][j]);

	printf("\nEnter the starting node:");
	scanf("%d",&u);
	astar(G,n,u);

	return 0;
}

// Return whether the spot is open
// Assume given coordinate is legal
int open(mapNode*** G, int x, int y) {
  return G[x][y]->empty;
}

// Return the manhattan distance of the points
// x, y: target x, y
// nx, ny: source x, y
int manhattanDist (double x, double y, double nx, double ny) {
  return abs(nx - x) + abs(ny - y);
}

// Return the angle from one position to another
int angle (double x1, double y1, double x2, double y2) {
	return atan((y1 - y2) / (x1 - x2));
}

// Compute the heuristics of the source and target
int heuristic (double x1, double y1, double x2, double y2) {
	return manhattanDist(x1, y1, x2, y2) + TURN * abs(angle(x1, y2, x2, y2));
}

// Given item matrix, find the adjacent nodes of the given node
// Return the number of successors returned
// G: item matrix
// n, m: dimension of G
// source: the given node
// target: the targetted node
// suc: output parameter for the successor
int successor(mapNode*** G, int n, int m, node source, node target, node* suc, int ini)
{
  // Go through the possible options
	ret = 0;
  int[] data = new int[] {-1, 1};

  for (int i = 0; i < 2; i++) {
		int x = source->x;
		int y = source->y;
		if (source->dir) {
	    y = ny + data[i];
	    if (y < 0 || y >= m) {
	      continue;
	    }
		} else {
    	x = nx + data[i];
	    if (x < 0 || x >= n) {
	      continue;
	    }
		}
    if (open(G, x, y)) {
			node newNode = malloc(sizeof(LinkedList));
			// initialize initial action
			newNode->x = x;
			newNode->y = y;
			newNode->dir = source->dir;
			newNode->t = source->t + 1;
			newNode->h = heuristic(x, y, target->x, target->y);
			newNode->ini = source->ini;
			*suc = newNode;
			suc++;
			ret++;
    }
  }
	// Turn
	node newNode = malloc(sizeof(LinkedList));
	// initialize initial action
	newNode->x = x;
	newNode->y = y;
	newNode->dir = (1 - source->dir);
	newNode->t = source->t + TURN;
	newNode->h = heuristic(source->x, source->y, target->x, target->y);
	newNode->ini = source->ini;
	*suc = newNode;
	ret++;
	return ret;
}

// Insert an element into the linkedlist priority queue
// Assume add prepared so that next is null, and nothing
// is next to it
void insert(node* head, node add) {
	node cur = *head;
	if (cur->t + cur->h >= add->t + add->h) {
		add->next = cur;
		*head = add;
		return;
	}
	while (cur->next != NULL) {
		next = cur->next;
		if ((next->t + next->h) >= (t + h)) {
			add->next = next;
			cur->next = add;
			return;
		}
		cur = next;
	}
	cur->next = add;
}

// add num nodes at add
void insertAll(node* head, node* add, int num) {
	for (int i = 0; i < num; i++) {
		insert(head, *(add + i));
	}
}

// Return the result of how to reach the target
// Return null if we are already there
// Return 1 if going forwards, 2 if going backwards, 3 if turn
// G: item matrix
// n, m: dimension
// source: the coordinate of the starting node
// 	 consists of x1, y1, dir
// target: the coordinate of the desired fruit
//   consists of x2, y2
void astar(mapNode*** G, int n, int m, int x1, int y1, int dir, int x2, int y2) {
	node head;
	head->next = NULL;
	head->x = x1;
	head->y = y1;
	head->dir = dir;
	head->ini = NULL;
	head->t = 0;
	head->h = heuristic(s, t);

	// memory for neighbor listings
	// maximum 3 options, forward, back, turn
	node list[] = (node) malloc(3 * sizeof(LinkedList));

	// fencepost so that we record the initial action correctly
	node cur = head;
	int res;
	res = successor(G, n, m, cur->x, cur->y, cur->dir, list);
	insertAll(&cur, &list, res);

	while (cur != NULL) {
		// Keep removing until we find the target
		if ((cur->x == x2) && (cur->y == y_2)) {
			// Found the target
			return cur->ini;
		}

		cur = head;
		res = successor(G, n, m, cur->x, cur->y, cur->dir, list);
		insertAll(&cur, &list, res);
		// Move the head along, and free the memory before
		cur = cur->next;
		free(head);
		head = cur;
	}

	free(list);
}
