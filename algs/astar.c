#include<stdio.h>
#include<stdlib.h>
#define INFINITY 9999
#define MAX 10

void astar(int G[MAX][MAX],int n,int startnode);

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
int open(int G[MAX][MAX], x, y) {
  return G[x][y] < 2;
}

// Return the manhattan distance of the points
// x, y: target x, y
// nx, ny: source x, y
int manhattanDist (int x, int y, int nx, int ny) {
  return abs(nx - x) + abs(ny - y);
}

// Given item matrix, find the adjacent nodes of the given node
// G: item matrix
// n, m: dimension of G
// nx, ny: coordinate of the given node
// suc: output parameter for the successor
void successor(int G[MAX][MAX], int n, int m, int nx, int ny, int* suc)
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

// Return the result of how to reach the target
// G: item matrix
// n, m: dimension
// start: the coordinate of the starting node
// target: the coordinate of the desired fruit
void astar(int G[MAX][MAX],int n,int startnode)
{
	int cost[MAX][MAX],distance[MAX],pred[MAX];
	int visited[MAX],count,mindistance,nextnode,i,j;

	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//create the cost matrix
	for (i = 0;i < n; i++)
		for (j = 0; j < n; j++)
			if (G[i][j] == 0)
				cost[i][j] = INFINITY;
			else
				cost[i][j] = G[i][j];

	//initialize pred[],distance[] and visited[]
	for(i=0;i<n;i++)
	{
		distance[i] = cost[startnode][i];
		pred[i] = startnode;
		visited[i] = 0;
	}

	distance[startnode]=0;
	visited[startnode]=1;
	count=1;

	while(count<n-1)
	{
		mindistance=INFINITY;

		//nextnode gives the node at minimum distance
		for(i=0;i<n;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}

			//check if a better path exists through nextnode
			visited[nextnode]=1;
			for(i=0;i<n;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	}

	//print the path and distance of each node
	for(i=0;i<n;i++)
		if(i!=startnode)
		{
			printf("\nDistance of node%d=%d",i,distance[i]);
			printf("\nPath=%d",i);

			j=i;
			do
			{
				j=pred[j];
				printf("<-%d",j);
			}while(j!=startnode);
	}
}