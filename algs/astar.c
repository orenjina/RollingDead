#include<stdio.h>
#include<stdlib.h>
// Estimated ratio of turn time versus grid travel time
#define TURN 10

void astar(mapNode*** G,int n, int m, int x1, int y1, int dir, int x2, int y2);

typedef struct LinkedList {
	int x;
	int y;
	int dir; // 0 north 1 east 2 south 3 west
	double t; // time elapsed
	double h; // heuristic function value
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
// nx, ny: coordinate of the given node
// suc: output parameter for the successor
int successor(mapNode*** G, int n, int m, int nx, int ny, int* suc)
{
  // Go through the 4 possible options
  int[] datax = new int[] {-1, 0, 0, 1};
  int[] datay = new int[] {0, -1, 1, 0};

  for (int i = 0; i < 4; i++) {
    int x = nx + datax[i];
    int y = ny + datay[i];
    if (x < 0 || x >= n) {
      continue;
    }
    if (y < 0 || y >= m) {
      continue;
    }
    if (open(G, x, y)) {

    }
  }
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

// Return the result of how to reach the target
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
	head->t = 0;
	head->h = heuristic(s, t);

	// memory for neighbor listings
	int list[] = (int*) malloc(6 * sizeof(int));

	while (head != NULL) {
		cur = head;
		int res;
		res = successor(G, n, m, cur, list)
		// Keep removing until we find the target
		// Order maintained by the linked list insert

		for (int i = 0; i < res; i++) {
			// If it is the target, we can stop
			// Otherwise add it to the linked list
		}

		// Move the head along, and free the memory before
		// if necessary
	}

	free(list);
}
