#include<stdio.h>
#include<stdlib.h>
#include<math.h>
// Estimated ratio of turn time versus grid travel time
#define TURN 3

typedef struct LinkedList {
	int x;
	int y;
	int dir; // 0 verticle, 1 horizontal
	double t; // time elapsed
	double h; // heuristic function value
	int ini; // initial action, left, right, or turn
	struct LinkedList* next; // next node
} *node;

int astar(int*** G,int n, int m, int x1, int y1, int dir, int x2, int y2);

int main()
{
	int n = 10;
	int m = 10;
	int** G = (int**) malloc(n * sizeof(int*));
  for(int i = 0; i < n; i++)
    G[i] = (int*) malloc (m * sizeof(int));

	for(int i = 0; i < n; i++) {
		for(int j = 0; j < m; j++) {
			G[i][j] = 0;
		}
	}

	int x = 5;
	int y = 5;

	int xp = 1;
	int yp = 1;

	int res = astar((int***) &(G), n, m, x, y, 0, xp, yp);
	printf("%d", res);
	return 0;
}

// Return whether the spot is open
// Assume given coordinate is legal
int open(int*** G, int x, int y) {
  // return G[x][y]->empty;
	return ((*G)[x][y] == 0);
}

// Return the manhattan distance of the points
// x, y: target x, y
// nx, ny: source x, y
int manhattanDist (int x, int y, int nx, int ny) {
  return abs(nx - x) + abs(ny - y);
}

// Return the angle from one position to another
double angle (double x1, double y1, double x2, double y2) {
	return atan((y2 - y1) / (x2 - x1));
}

// Compute the heuristics of the source and target
int heuristic (double x1, double y1, double x2, double y2) {
	return manhattanDist(x1, y1, x2, y2) + TURN * fabs(angle(x1, y2, x2, y2));
}

// inspect linked list elements, debugging use
void inspectList(node* head, int num) {
	printf("-------- begin inspect for list -------\n");
	for (int i = 0; i < num; i++) {
		printf("current node is: %x\n", *(head + i));
	}
}

// Insert an element into the linkedlist priority queue
// Assume add prepared so that next is null, and nothing
// is next to it
void insert(node* head, node add) {
	printf("NOW INSERTING!!!!!!!!!\n");
	node cur = *head;
	if (cur->t + cur->h >= add->t + add->h) {
		add->next = cur;
		*head = add;
		return;
	}
	while (cur->next != NULL) {
		// printf("calling\n");
		node next = cur->next;
		if ((next->t + next->h) >= (add->t + add->h)) {
			add->next = next;
			cur->next = add;
			return;
		}
		cur = next;
	}
	cur->next = add;
}

// inspect linked list elements, debugging use
void inspect(node* head) {
	printf("-------- begin inspect -------\n");
	node cur = *head;
	while (cur!= NULL) {
		printf("current node is: %x\n", cur);
		cur = cur->next;
	}
}

// add num nodes at add
void insertAll(node* head, node* const add, int num) {
	printf("insertall begins, %d entries\n", num);
	for (int i = 0; i < num; i++) {
		printf("inserting entry %x\n", *(add + i));
		inspect(head);
		inspectList(add, 3);
		insert(head, *(add + i));
	}
	printf("insertall finishes\n");
}

// Given item matrix, find the adjacent nodes of the given node
// Return the number of successors returned
// G: item matrix
// n, m: dimension of G
// source: the given node
// target: the targetted node
// suc: output parameter for the successor
// ini: whether to initialize the initial action or not
int successor(int*** G, int n, int m, node source, node target, node* suc, int ini)
{
  // Go through the possible options
	int ret = 0;
  int data[2] = {-1, 1};

  for (int i = 0; i < 2; i++) {
		int x = source->x;
		int y = source->y;
		if (source->dir) {
	    y = y + data[i];
	    if (y < 0 || y >= m) {
	      continue;
	    }
		} else {
    	x = x + data[i];
	    if (x < 0 || x >= n) {
	      continue;
	    }
		}
    if (open(G, x, y)) {
			struct LinkedList ne;
			node newNode = &ne;
			// initialize initial action
			newNode->x = x;
			newNode->y = y;
			newNode->dir = source->dir;
			newNode->t = source->t + 1;
			newNode->h = heuristic(x, y, target->x, target->y);
			if (ini) {
				newNode->ini = i;
			} else {
				newNode->ini = source->ini;
			}
			newNode->next = NULL;
			**(suc + ret) = ne;
			ret++;
    }
  }
	printf("Inspecting source\n");
	inspect(&source);
	// Turn
	struct LinkedList ne;
	node newNode = &ne;
	// initialize initial action
	int x = source->x;
	int y = source->y;
	newNode->x = x;
	newNode->y = y;
	newNode->dir = (1 - source->dir);
	newNode->t = source->t + TURN;
	newNode->h = heuristic(source->x, source->y, target->x, target->y);
	if (ini) {
		newNode->ini = 2;
	} else {
		newNode->ini = source->ini;
	}
	newNode->next = NULL;
	**(suc + ret) = ne;
	ret++;
	inspectList(suc, 3);
	return ret;
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
int astar(int*** G, int n, int m, int x1, int y1, int dir, int x2, int y2) {
	node head = malloc(sizeof(struct LinkedList));
	head->next = NULL;
	head->x = x1;
	head->y = y1;
	head->dir = dir;
	head->ini = -1;
	head->t = 0;
	head->h = heuristic(x1, y1, x2, y2);

	node target = malloc(sizeof(struct LinkedList));
	target->x = x2;
	target->y = y2;

	// memory for neighbor listings
	// maximum 3 options, forward, back, turn

	node list = (node) malloc(3 * sizeof(struct LinkedList));

	printf("Head is given the pointer %x\n", head);
	printf("List is given the pointer %x\n", list);
	printf("List[2] is given the pointer %x\n", &(list[2]));
	inspectList(&list, 3);

	// fencepost so that we record the initial action correctly
	node cur = head;
	inspect(&head);
	printf("head was: %x\n", head);
	int res;
	res = successor(G, n, m, cur, target, &list, 1);
	printf("res: %d\n", res);
	printf("now head is: %x\n", head);
	head = cur->next;
	printf("now head is: %x\n", head);
	inspect(&head);
	insertAll(&head, &list, res);

	free(cur);

	while (head != NULL) {
		printf("loop call\n");
		// Keep removing until we find the target
		if ((cur->x == x2) && (cur->y == y2)) {
			// Found the target
			return cur->ini;
		}

		list = (node) malloc(3 * sizeof(struct LinkedList));

		cur = head;
		inspect(&head);

		printf("IFJLJKDLFJSKLFJLDKJFLSKJF\n");
		head = cur->next;

		inspect(&head);
		res = successor(G, n, m, cur, target, &list, 0);
		printf("successor called\n");

		inspectList(&list, res);

		insertAll(&head, &list, res);
		printf("insert called\n");
		// Move the head along, and free the memory before
		free(cur);
	}

	return -1;
}
