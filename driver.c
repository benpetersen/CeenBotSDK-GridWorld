/*	DOCUMENTATION:
	==============
	4/4/12:
		In order to map the fastest route back to the starting node after finding the red node,
			the robot must be aware of what it DOESN'T KNOW, as well as what it knows. This is
			because we don't want the robot to try to follow a route that could potentially
			contain blocks in the way.
		To resolve this issue, I have added the IDK value to the Segment enum.
		The issue with this is that if we mark the corner nodes as VISITED without actually having
			been there, the robot won't sense the surrounding segments once it gets there. This is
			because currently there is an optimization so that the robot only senses surrounding
			segments if it hasn't been to that block before.
		There are two possible solutions to this problem:
			1) sense every segment upon every move (even when we already know if there are blocks
			   on those segments or not)
			2) initialize the corner nodes to UNVISITED -- this is also not very "smart" because
			   we know that the red node won't be in a corner based on the competition rules
		At this point, solution #2 is implemented, so the corner nodes are checked.
*/

#include <stdio.h>
#include <stdlib.h>

/* Global Constants: */
/* ================= */
#define NUM_ROWS 7		/* number of rows in grid-world */
#define NUM_COLS 6      /* number of columns in grid-world */
#define NUM_NODES 42	/* 7 * 6 = 42 */
/* #define MOVE_TIME 1000 */	/* amount of time to move to first node ahead -- in milliseconds */
/* #define TURN_TIME 1000 */	/* amount of time to turn 90 degrees left or right -- in milliseconds */

/* ADT's: */
/* ====== */
typedef enum {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, FRONT = 0, BACK = 1} Direction;
typedef enum {FALSE, TRUE} Bool;			/* C does not have a primitive Bool type */
typedef enum {UNVISITED, VISITED} Node;		/* indicates whether or not a node has been visited */
typedef enum {BLOCKED, UNBLOCKED, IDK} Segment;	/* indicates knowledge of presense of blocks */

/* Storage variables: */
/* ================== */
/* UPDATE AFTER TURNING: */
Direction direction;			/* stores current direction */

/* UPDATE AFTER MOVING: */
/*======================*/
int currentNode[2];				/* [0]: current row, [1]: current column */
/* UPDATE ONLY AFTER SENSING THE RED VINYL */
Bool foundRedNode;
/* UPDATE ONLY AFTER ENTERING AN UNVISITED NODE */
Node grid[NUM_ROWS][NUM_COLS];				/* UNVISITED or VISITED */
Segment horizSeg[NUM_ROWS + 1][NUM_COLS];	/* horizontal segments, BLOCKED or UNBLOCKED */
Segment vertSeg[NUM_ROWS][NUM_COLS + 1];	/* vertical segments, BLOCKED or UNBLOCKED */
int **route;		/* 
						Should be "int route [NUM_NODES][2];", but in order
						for the followPath function to work, a pointer declaration
						is needed.

						Ex:	route[0][0] indicates row # of first node in path
							route[0][1] indicates col # of first node in path
					*/
/* Sensors: */
/* ======== */
// TO SENSE VINYL: TRUE -- vinyl is sensed underneath robot */
Bool color_s(void);		/* for red square underneath robot */
Bool bottomL_s(void);	/* for vinyl on the front-left */
Bool bottomR_s(void);	/* for vinyl on the front-right */
/* TO SENSE BLOCKS: TRUE -- block is sensed in adjacent segment */
Bool left_s(void);		/* for blocks to the left */
Bool right_s(void);		/* for blocks to the right */
Bool frontL_s(void);	/* for blocks to the front-left */
Bool frontR_s(void);	/* for blocks to the front-right */

/* Actuators: */
/* ========== */
void evenToOddRow(void);
void oddToEvenRow(void);
void changeColumn(void);
void turnLeft(void);		/* turns robot 90 degrees to relative LEFT */
void turnRight(void);		/* turns robot 90 degrees to relative RIGHT */
void turn180(void);			/* turns robot 180 degrees in relative orientation */
void sing(void);

/* function declarations */
void initializeGlobalVariables(void);
void deallocateMemory(void);
void initZigZagRoute(void);
void checkForBlocks(void);
Segment segAbs(Direction);	/* returns status of adjacent segment, absolute Direction */
Segment segRel(Direction);	/* returns status of adjacent segment, relative Direction */
void turnAbs(Direction);	
void moveForward(void);
void moveBackward(void);
void findLowestNode(int *, int *);
void followPath(int **, int);
void thinkAndMove(void);
Bool takeQuickestPath(int, int);

/* THESE FUNCTIONS NEED TO BE WRITTEN AFTER FINISHED TESTING */
void avoid(int []);					/* avoids an obstacle by moving around it */
void maneuverRight(int []);			/* move around the RIGHT side of an obstacle */
void maneuverLeft(int []);			/* move around the LEFT side of an obstacle */
void traverseGrid(int **, int);

/* !!!TEST ONLY!!! Remove from uploaded code, or perhaps use a #pragma so that these
   functions and variables are only defined on a specific compiler, like gcc */
void initializeTestVariables(void);	/* initializes variables used only in virtual grid */
void displayGrid(void);				/* virtual grid */
void displayRoute(void);			/* displays route to screen */
void pauseAndClear(void);			/* used in displayGrid and displayRoute function */
void flushInputBuffer(void);		/* JUST IN CASE... */
/* the functions indicated by TEST are not altered logically -- they just support console I/O */
void moveForwardTEST(void);
void checkForBlocksTEST(void);
void traverseGridTEST(int **, int);
void avoidTEST(int []);				
void maneuverRightTEST(int []);
void maneuverLeftTEST(int []);
void singTEST(void);
Bool color_sTEST(void);		/* for red square underneath robot */
Bool bottomL_sTEST(void);	/* for vinyl on the front-left */
Bool bottomR_sTEST(void);	/* for vinyl on the front-right */
Bool left_sTEST(void);		/* for blocks to the left */
Bool right_sTEST(void);		/* for blocks to the right */
Bool frontL_sTEST(void);	/* for blocks to the front-left */
Bool frontR_sTEST(void);	/* for blocks to the front-right */
int redNode[2];				/* location of red floor tile in grid */
/* 
	The following 2 arrays are representations of locations of physical obstacles,
	which may or may not have been sensed by the robot. Up to 13 segments will be blocked.
*/
Segment blockedHorizSeg[NUM_ROWS + 1][NUM_COLS];
Segment blockedVertSeg[NUM_ROWS][NUM_COLS + 1];
Segment blockedSegAbs(Direction);	/* indicates presence of virtual grid block, absolute Direction */
Segment blockedSegRel(Direction);	/* indicates presence of virtual grid block, relative Direction */

/* DEMONSTRATION ONLY!!! */
void easyReturnTEST(void);
void easyReturn(void); /* assumes no obstacles on grid -- moves robot back to start */

/* function main -- VERSION 1 */
//int main(void)
//{
//	initializeGlobalVariables();
//
//	/* check for blocks surrounding start space */
//	checkForBlocks();
//
//	/* find colored goal space */
//	while (color_s() == FALSE)
//	{
//		thinkAndMove();
//	}
//
//	/* sing a song and go back to original space */
//	sing();
//	takeQuickestPath(0, 0); /* take quickest path to start node */
//  
//  deallocateMemory();
//	return 0;
//}

/* function main -- TEST VERSION */
int main(void)
{
	initializeGlobalVariables();
	initializeTestVariables();

	///* MISC: test to see that the route has been initialized correctly */
	//displayRoute();

	///* MISC: test the singTEST function (alarm escape sequence '\a') */
	//singTEST();
	
	system("clear");
	displayGrid();
	checkForBlocksTEST(); /* check for blocks surrounding start space */

	///* TEST 1: move robot to edge of grid (no obstacles in way) */
	//while(currentNode[0] < NUM_ROWS - 1)
	//{
	//	moveForwardTEST();
	//	displayGrid();
	//}
	//
	///* turn around */
	//turnAbs(LEFT);		displayGrid();
	//turnAbs(UP);			displayGrid();
	//turnAbs(RIGHT);		displayGrid();
	//turnAbs(DOWN);		displayGrid();

	/* TEST 2: move robot until it reaches a blocked segment (it will sense each segment as it moves) */
	//while (segRel(FRONT) == UNBLOCKED)
	//{
	//	moveForwardTEST();
	//	displayGrid();
	//}

	/* IMPORTANT TEST!!! (this will be used in actual physical implementation) */
	traverseGridTEST(route, NUM_NODES);
	easyReturnTEST();

	deallocateMemory();
	return 0;
}

void initializeGlobalVariables(void)
{
	int i, j;	/* loop control variables */
	
	/* haven't found finish space yet */
	foundRedNode = FALSE;

	/* current orientation and position of robot */
	direction = DOWN;
	currentNode[0] = 0; currentNode[1] = 0;

	/* initialize grid segments that have been “visited”
	(or are not going to contain the red space -- i.e. the corner spaces) */
	//grid[0][0] = VISITED;							/* top-left */
	//grid[0][NUM_COLS - 1] = VISITED;				/* top-right */
	//grid[NUM_ROWS - 1][0] = VISITED;				/* bottom-left */
	//grid[NUM_ROWS - 1][NUM_COLS - 1] = VISITED;		/* bottom-right */

	/* initialize horizontal segments */
	for (i = 0; i < NUM_ROWS + 1; ++i)	/* set all horizontal segments to IDK */
	{
		for (j = 0; j < NUM_COLS; ++j)
			horizSeg[i][j] = IDK;
	}
	for (i = 0; i < NUM_COLS; ++i)		/* set border horizontal segments to BLOCKED */
	{
		horizSeg[0][i] = BLOCKED;
		horizSeg[NUM_ROWS][i] = BLOCKED;
	}

	/* initialize vertical segments */
	for (i = 0; i < NUM_ROWS; ++i)		/* set all vertical segments to IDK */
	{
		for (j = 0; j < NUM_COLS + 1; ++j)
			vertSeg[i][j] = IDK;
	}
	for (i = 0; i < NUM_ROWS; ++i)		/* set border vertical segments to BLOCKED */
	{
		vertSeg[i][0] = BLOCKED;
		vertSeg[i][NUM_ROWS - 1] = BLOCKED;
	}

	/* initialize route (i.e. sequence of nodes to check) */
	route = (int**)malloc(NUM_NODES * sizeof(int*)); 
	for (i = 0; i < NUM_NODES; ++i)
	{
		route[i] = (int*)malloc(2 * sizeof(int));
	}
	initZigZagRoute();

	return;
}

/* deallocates dynamically allocated memory for global variables */
void deallocateMemory(void)
{
    int i; 
 
	/* int **route */
    for (i = 0; i < NUM_NODES; ++i) 
    {
		free(route[i]);
    }
    free(route);

	return;
}

/* initializes route with pure "zig-zag" pattern */
void initZigZagRoute(void)
{
	int i, j,	/* loop control variables */
		n = 0;	/* index for route */

	for (j = 0; j < NUM_COLS; ++j)
	{
		if (j % 2 == 0) /* column number = 0, 2, ... */
		{
			for (i = 0; i < NUM_ROWS; ++i)
			{
				route[n][0] = i;
				route[n][1] = j;
				++n;
			}
		}
		else /* column number = 1, 3, ... */
		{
			for (i = NUM_ROWS - 1; i >= 0; --i)
			{
				route[n][0] = i;
				route[n][1] = j;
				++n;
			}
		}
	}

	return;
}

/* this function checks for blocks occupying adjacent segments in each new node visited */
void checkForBlocks(void)
{
	/* define aliases for adjacent segments */
	Segment *top, *btm, *lft, *rgt;
	top = &(horizSeg[currentNode[0]][currentNode[1]]);
	btm = &(horizSeg[currentNode[0] + 1][currentNode[1]]);
	lft = &(vertSeg[currentNode[0]][currentNode[1]]);
	rgt = &(vertSeg[currentNode[0]][currentNode[1] + 1]);

	if (direction == UP)
	{
		if (frontL_s() == TRUE || frontR_s() == TRUE)	*top = BLOCKED;		else *top = UNBLOCKED;
		if (left_s() == TRUE)							*lft = BLOCKED;		else *lft = UNBLOCKED;
		if (right_s() == TRUE)							*rgt = BLOCKED;		else *rgt = UNBLOCKED;
	}
	else if (direction == DOWN)
	{
		if (frontL_s() == TRUE || frontR_s() == TRUE)   *btm = BLOCKED;		else *btm = UNBLOCKED;
		if (left_s() == TRUE)							*rgt = BLOCKED;		else *rgt = UNBLOCKED;
		if (right_s() == TRUE)							*lft = BLOCKED;		else *lft = UNBLOCKED;
	}
	else if (direction == LEFT)
	{
		if (frontL_s() == TRUE || frontR_s() == TRUE)   *lft = BLOCKED;		else *lft = UNBLOCKED;
		if (left_s() == TRUE)							*btm = BLOCKED;		else *btm = UNBLOCKED;
		if (right_s() == TRUE)							*top = BLOCKED;		else *top = UNBLOCKED;
	}
	else /* direction == RIGHT */
	{
		if (frontL_s() == TRUE || frontR_s() == TRUE)	*rgt = BLOCKED;		else *rgt = UNBLOCKED;
		if (left_s() == TRUE)							*top = BLOCKED;		else *top = UNBLOCKED;
		if (right_s() == TRUE)							*btm = BLOCKED;		else *btm = UNBLOCKED;
	}
	return;
}

/* returns status of adjacent segment, specified by absolute Direction */
Segment segAbs(Direction absDir)
{
	if (absDir == UP)
		return horizSeg[currentNode[0]][currentNode[1]];
	else if (absDir == DOWN)
		return horizSeg[currentNode[0] + 1][currentNode[1]];
	else if (absDir == LEFT)
		return vertSeg[currentNode[0]][currentNode[1]];
	else /* absDir == RIGHT */
		return vertSeg[currentNode[0]][currentNode[1] + 1];
}

/* returns status of adjacent segment, specified by relative Direction */
Segment segRel(Direction relSeg)
{
	Segment returnValue;

	/* currently facing absolute UP */
	if (direction == UP)
	{
		returnValue = segAbs(relSeg); /* same as segAbs function */
	}
	/* currently facing absolute DOWN */
	else if (direction == DOWN)
	{
		if (relSeg == FRONT) /* absolute DOWN */
			returnValue = segAbs(DOWN);
		else if (relSeg == BACK) /* absolute UP */
			returnValue = segAbs(UP);
		else if (relSeg == LEFT) /* absolute RIGHT */
			returnValue = segAbs(RIGHT);
		else /* relSeg == RIGHT ---> absolute LEFT */
			returnValue = segAbs(LEFT);
	}
	/* currently facing absolute LEFT */
	else if (direction == LEFT)
	{
		if (relSeg == FRONT) /* absolute LEFT */
			returnValue = segAbs(LEFT);
		else if (relSeg == BACK) /* absolute RIGHT */
			returnValue = segAbs(RIGHT);
		else if (relSeg == LEFT) /* absolute DOWN */
			returnValue = segAbs(DOWN);
		else /* relSeg == RIGHT ---> absolute UP */
			returnValue = segAbs(UP);
	}
	/* currently facing absolute RIGHT */
	else /* direction == RIGHT */
	{
		if (relSeg == FRONT) /* absolute RIGHT */
			returnValue = segAbs(RIGHT);
		else if (relSeg == BACK) /* absolute LEFT */
			returnValue = segAbs(LEFT);
		else if (relSeg == LEFT) /* absolute UP */
			returnValue = segAbs(UP);
		else /* relSeg == RIGHT ---> absolute DOWN */
			returnValue = segAbs(DOWN);
	}

	return returnValue;
}

/* turn robot to face specified absolute direction */
void turnAbs(Direction dirToFace)
{
	/* want to face up */
	if (dirToFace == UP)
	{
		if (direction == DOWN)
			turn180();
		else if (direction == LEFT)
			turnRight();
		else if (direction == RIGHT)
			turnLeft();
		/* else (direction == UP) ---> no need to turn */
	}
	/* want to face down */
	else if (dirToFace == DOWN)
	{
		if (direction == UP)
			turn180();
		else if (direction == LEFT)
			turnLeft();
		else if (direction == RIGHT)
			turnRight();
		/* else (direction == DOWN) ---> no need to turn */
	}
	/* want to face left */
	else if (dirToFace == LEFT)
	{
		if (direction == UP)
			turnLeft();
		else if (direction == DOWN)
			turnRight();
		else if (direction == RIGHT)
			turn180();
		/* else (direction == LEFT) ---> no need to turn */
	}
	/* want to face right */
	else /* dirToFace == RIGHT */
	{
		if (direction == UP)
			turnRight();
		else if (direction == DOWN)
			turnLeft();
		else if (direction == LEFT)
			turn180();
		/* else (direction == RIGHT) ---> no need to turn */
	}
 
	return;
}

/* move from the current space to the next one the robot is facing */
void moveForward(void)
{
	/* check to make sure the robot can make the movement */
	if (segRel(FRONT) == UNBLOCKED)
    {
        /***code for bottomL_s and bottomR_s sensors COULD MAYBE go here***/
		if (direction == UP || direction == DOWN)
		{
			if (currentNode[0] % 2 == 0) /* currently in even-numbered row */
				evenToOddRow();
			else /* currently in odd-numbered row */
				oddToEvenRow();
		}
		else /* direction == LEFT || direction == RIGHT) */
		{
			changeColumn();
		}

		/* update currentNode array */
		if (direction == UP)
			currentNode[0] -= 1; /* decrement row */
		else if (direction == DOWN)
			currentNode[0] += 1; /* increment row */
		else if (direction == LEFT)
			currentNode[1] -= 1; /* decrement column */
		else /* direction == RIGHT */
			currentNode[1] += 1; /* increment column */
                
		/* if the node has not been visited before, scan for blocks and set node in grid to VISITED */
		if (grid[currentNode[0]][currentNode[1]] == UNVISITED)
		{
			checkForBlocks();
			grid[currentNode[0]][currentNode[1]] = VISITED;
		}

		/* if red vinyl is detected, sing and set global variable foundRedNode to TRUE */
		if (color_s() == TRUE)
		{
			sing();
			foundRedNode = TRUE;
		}
    }

    return;
}

/* move from the current space to the one behind the robot -- MAY BE AN UNNECESSARY FUNCTION */
void moveBackward(void)
{
    /* code for stepper motor controls goes here */

	/* update currentNode array */
	if (direction == UP)
		currentNode[0] += 1; /* increment row */
	else if (direction == DOWN)
		currentNode[0] -= 1; /* decrement row */
	else if (direction == LEFT)
		currentNode[1] += 1; /* increment column */
	else /* direction == RIGHT */
		currentNode[1] -= 1; /* decrement column */

    return;
}

/* finds the upper-left-most node -- does NOT account for obstacles!!! */
/* Comment (3/11, Sam): The way this is implemented, each node is only checked once */
/* Comment (3/28, Sam): This function is being phased out with the "route" implementation */
void findLowestNode(int *row, int *col)
{
	int i, j, n;			/* loop control variables */
	Bool isFound = FALSE;	/* set to TRUE once an unvisited node has been found */
        
	/* Checks upper-left-most (n + 1) X (n + 1) group of nodes, from n = 0
	   through n = NUM_COLS - 1 (NUM_COLS = 6 < 7 = NUM_ROWS). */
	for (n = 0; (n < NUM_COLS) && !isFound; ++n)
	{
		/* check nodes in lower row of square */
		for (j = 0; (j <= n) && !isFound; ++j)
		{
			if (grid[n][j] == UNVISITED)
			{
				*row = n;
				*col = j;
				isFound = TRUE;
			}
		}

		for (i = 0; (i < n) && !isFound; ++i)
		{
			if (grid[i][n] == UNVISITED)
			{
				*row = i;
				*col = n;
				isFound = TRUE;
			}
		}
	}

	/* if no nodes could be found in the upper 6x6 group of nodes,
		check the bottom row of nodes from left to right */
	if (!isFound)
	{
		for (j = 0; (j < NUM_COLS) && !isFound; ++j)
		{
			if (grid[NUM_ROWS - 1][j] == UNVISITED)
			{
				*row = NUM_ROWS - 1;
				*col = j;
				isFound = TRUE;
			}
		}
	}

	return;
}

/*  path[index][0: row /// 1: col]
	index:		number of nodes; 0 = first node (current node), 1 = second node, size - 1 = last node
	row/col:	0 = row #, 1 = column #
	size:		length of path array
*/
void followPath(int **path, int size)
{
	int i; /* loop control variable */

	/* if current node is first node in path, execute body of function
	  (if current node is NOT the first node in path, followPath will NOT execute)*/
	if ((currentNode[0] == path[0][0]) && (currentNode[1] == path[0][1]))
	{         
		for (i = 1; i < size; ++i)
		{
			/* TURN TO FACE APPROPRIATE DIRECTION */
			/* next node is UP */
			if ((currentNode[0] == path[i][0] + 1) && (currentNode[1] == path[i][1]))
				turnAbs(UP);
			/* next node is DOWN */
			else if ((currentNode[0] == path[i][0] - 1) && (currentNode[1] == path[i][1]))
				turnAbs(DOWN);
			/* next node is LEFT */
			else if ((currentNode[0]==path[i][0]) && (currentNode[1]==path[i][1] + 1))
				turnAbs(LEFT);
			/* next node is RIGHT */
			else if ((currentNode[0]==path[i][0]) && (currentNode[1]==path[i][1] - 1))
				turnAbs(RIGHT);
			else {}
				/* no error-handling now -- note that moveForward always executes */
                
			/* move to next node and scan for surrounding blocks if node has not been visited */
			moveForward();

		} /* for */
	} /* if */

	return;
}

/* FLOOD CONTROL ALGORITHM
   For now, the robot just moves along the fastest path to the 
   upper-left-most unvisited node of the grid based on what it
   knows about segments being blocked or not */
void thinkAndMove(void)
{
	int minRow, minCol; /* row and column of upper-left-most node */
	
	do
	{
		findLowestNode(&minRow, &minCol);
	} while (takeQuickestPath(minRow, minCol) == FALSE); // FIXME: function returns void

	return;
}

/* There are two parts to the takeQuickestPath function:
        1) calculate fastest path from current node to node[row][col]
                A) use horizSeg and vertSeg arrays to determine
                   all known unblocked paths to target node
                B) use MOVE_TIME and (TURN_TIME along with shape of paths) to
                   determine the amount of time needed to traverse paths
                C) find the path with the minimum travel time
           If there is no known path to the destination node,
           guess what the shortest path will be. If it is found that the
           node is inaccessible (which would happen if all the segments
           surrounding that node had blocks on them), mark that node as
           TRUE(visited) and abandon trying to gain access to that node
        2) MOVE the robot along that path
           These are your 3 basic options for movement:
                A) turn( UP, DOWN, LEFT, RIGHT );
                B) moveForward();
                C) moveBackward();
           If it is found that the node is inaccessible (which would happen if
           all the segments surrounding that node had blocks on them), mark that
           node as TRUE(visited) and abandon trying to gain access to that node
*/
Bool takeQuickestPath(int row, int col)
{
	Bool pathExists;
	int **path;		/* pointer to dynamic array of node arrays (2 ints per node) -- not sure if syntax will work */
	int size;		/* number of nodes in path */
	int i;

	/* find the fastest route between current node and node at [row][col]: */
	{
		/* HEAVY-DUTY ALGORITHM GOES HERE 
		1) determine if pathExists based on what we know
		2) determine the size of the path
		3) dynamically allocate a path (array of row/col values)
		*/
		pathExists = FALSE; /* DUMMY VALUE!!! */
		size = 10;			/* DUMMY VALUE!!! */
	}

	if (pathExists == TRUE)
	{
		/* dynamically allocate array of nodes (array/col values) */
		path = (int**)malloc(size * sizeof(int*)); 
		for (i = 0; i < size; ++i)
		{
			path[i] = (int*)malloc(2 * sizeof(int));
		}

		followPath(path, size);
	}

	return pathExists;
}

/* SENSOR FUNCTIONS: */
/* TO SENSE VINYL: TRUE -- vinyl is sensed underneath robot */
Bool color_s(void) /* for red square underneath robot */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
Bool bottomL_s(void)	/* for vinyl on the front-left */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
Bool bottomR_s(void)	/* for vinyl on the front-right */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
/* TO SENSE BLOCKS: TRUE -- block is sensed in adjacent segment */
Bool left_s(void)		/* for blocks to the left */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
Bool right_s(void)		/* for blocks to the right */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
Bool frontL_s(void)		/* for blocks to the front-left */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}
Bool frontR_s(void)		/* for blocks to the front-right */
{
	/***sensor code goes here***/

	return FALSE; /* dummy return value */
}

/* ACTUATOR FUNCTIONS: */
/* sings a song -- this function should only be called once the goal space has been found */
void sing(void)
{
    /* code for speaker controls goes here */
    return;
}
/* move from an even-numbered row to an odd-numbered row 
   REMEMBER: row numbers range from 0 to 6 (7 row) */
void evenToOddRow(void)
{
	/***stepper motor code goes here***/

	return;
}
/* move from an odd-numbered row to an even-numbered row 
   REMEMBER: row numbers range from 0 to 6 (7 row) */
void oddToEvenRow(void)
{
	/***stepper motor code goes here***/
	
	return;
}
/* move from one column to the next */
void changeColumn(void)
{
	/***stepper motor code goes here***/
	
	return;
}
/* turns robot 90 degrees to relative LEFT */
void turnLeft(void)
{
	/* stepper motor code for turning robot 90 degrees to the left goes here */

	if (direction == UP)
		direction = LEFT;
	else if (direction == DOWN)
		direction = RIGHT;
	else if (direction == LEFT)
		direction = DOWN;
	else /* direction == RIGHT */
		direction = UP;

	return;
}
/* turns robot 90 degrees to relative RIGHT */
void turnRight(void)
{
	/* stepper motor code for turning robot 90 degrees to the right goes here */

	if (direction == UP)
		direction = RIGHT;
	else if (direction == DOWN)
		direction = LEFT;
	else if (direction == LEFT)
		direction = UP;
	else /* direction == RIGHT */
		direction = DOWN;

	return;
}
/* turns robot 180 degrees from it's current relative position */
void turn180(void)
{
	/* stepper motor code for turning robot 180 degrees goes here */

	if (direction == UP)
		direction = DOWN;
	else if (direction == DOWN)
		direction = UP;
	else if (direction == LEFT)
		direction = RIGHT;
	else /* direction == RIGHT */
		direction = LEFT;

	return;
}

/* THE FOLLOWING FUNCTIONS ARE TEST ONLY!!! */

/* possible orientations of blocks:
	BLOCK 1: straight, short
	
		BBB+BBB
	
		BBB+BBB+BBB
		
		B		B
		+		+
		B		+
		+		B
				+
				B
			
	BLOCK 2: straight, long
	(yes, it is the same as BLOCK 1)
	
		BBB+BBB
	
		BBB+BBB+BBB
	
		B		B
		+		+
		B		+
		+		B
				+
				B
	
	BLOCK 3: L-shaped
	
		B+BBB     BBB+B
		+			  +
		B			  B
	
	
		B			  B
		+			  +
		B+BBB	  BBB+B
		
	BLOCK 4: T-shaped
	
		BBB+BBB		B
		   B		+BBB+BBB
		   +		B
		   B
		   
			   B	BBB+BBB
		BBB+BBB+	   B
			   B       +
			           B
*/
void initializeTestVariables(void)
{
	int i, j; /* LCV's */

	/* location of destination floor tile */
	redNode[0] = 4;
	redNode[1] = 5;

	/* initialize all PHYSICAL (independent of robot) segments to UNBLOCKED */
	for (i = 0; i < NUM_ROWS + 1; ++i)
	{
		for (j = 0; j < NUM_COLS; ++j)
			blockedHorizSeg[i][j] = UNBLOCKED;
	}
	for (i = 0; i < NUM_ROWS; ++i)
	{
		for (j = 0; j < NUM_COLS + 1; ++j)
			blockedVertSeg[i][j] = UNBLOCKED;
	}

	/*
		set locations of blocks:
		========================
		Segment blockedHorizSeg[NUM_ROWS + 1][NUM_COLS];
		Segment blockedVertSeg[NUM_ROWS][NUM_COLS + 1];
	*/
	/* set BLOCK 1 */
	/*blockedHorizSeg[4][0] = BLOCKED;
	blockedHorizSeg[4][1] = BLOCKED;*/

	/* set BLOCK 2 */
	/*blockedVertSeg[0][1] = BLOCKED;
	blockedVertSeg[1][1] = BLOCKED;*/

	/* set BLOCK 3 */
	/* <none> */

	/* set BLOCK 4 */
	/* <none> */

	return;
}

/* FUNCTION displayGrid
 Grid:
   0   1   2   3   4   5
 +###+###+###+###+###+###+
0#   |   |   |   |   |   #
 +---+---+---+---+---+---+
1#   |   |   |   |   |   #
 +---+---+---+---+---+---+
2#   |   |   |   |   |   #
 +---+---+---+---+---+---+
3#   |   |   |   |   |   #
 +---+---+---+---+---+---+
4#   |   |   |   |   |   #
 +---+---+---+---+---+---+
5#   |   |   |   |   |   #
 +---+---+---+---+---+---+
6#   |   |   |   |   |   #
 +###+###+###+###+###+###+
 
 Direction indicators:
   ^
 <   >
   v
  
  Block indicators:
  ? / ???	unsensed	unblocked
  | / ---	sensed		unblocked
  B	/ BBB	unsensed	blocked	
  # / ###	sensed		blocked
  
  Other indicators:
  .		VISITED node
  R		red node
*/
void displayGrid(void)
{
	int i, j; /* LCV's */
 
	/* display header */
	printf("    0   1   2   3   4   5\n  "); 
	
	/* display bulk of grid */
	for (i = 0; i < NUM_ROWS; ++i)
	{
		/* horizontal segments */
		for (j = 0; j < NUM_COLS; ++j)
		{
			printf("+");
			
			if (horizSeg[i][j] == IDK)
			{
				if (blockedHorizSeg[i][j] == BLOCKED)
					printf("BBB");
				else
					printf("???");
			}
			else
			{
				if (horizSeg[i][j] == UNBLOCKED)
					printf("---");
				else /* BLOCKED */
					printf("###");
			}
		}
		printf("+\n%d ", i); /* new line and row number */
		
		/* vertical segments and node information */
		for (j = 0; j < NUM_COLS; ++j)
		{
			/* segments */
			if (vertSeg[i][j] == IDK)
			{
				if (blockedVertSeg[i][j] == BLOCKED)
					printf("B ");
				else
					printf("? ");
			}
			else
			{
				if (vertSeg[i][j] == UNBLOCKED)
					printf("| ");
				else /* BLOCKED */
					printf("# ");
			}

			/* node information */
			if (i == currentNode[0] && j == currentNode[1]) /* display robot */
			{
				if (direction == UP)
					printf("^ ");
				else if (direction == DOWN)
					printf("v ");
				else if (direction == LEFT)
					printf("< ");
				else /* direction == RIGHT */
					printf("> ");
			}
			else if (i == redNode[0] && j == redNode[1]) /* display red indicator */
			{
				printf("R ");
			}
			else if (grid[i][j] == VISITED) /* display VISITED indicator */
			{
				printf(". ");
			}
			else /* display blank -- UNVISITED indicator */
			{
				printf("  ");
			}
		} /* for */
		
		/* last column of vertical segments */
		if (vertSeg[i][NUM_COLS] == IDK) /* this should never evaluate to TRUE */
		{
			if (blockedVertSeg[i][NUM_COLS] == BLOCKED)
				printf("B\n  ");
			else
				printf("?\n  ");
		}
		else
		{
			if (vertSeg[i][NUM_COLS] == UNBLOCKED)
				printf("|\n  ");
			else /* BLOCKED */
				printf("#\n  ");
		}
	}
	
	/* last row of horizontal segments */
	for (j = 0; j < NUM_COLS; ++j)
	{
		printf("+");
		
		if (horizSeg[NUM_ROWS][j] == IDK) /* this should never evaluate to TRUE */
		{
			if (blockedHorizSeg[NUM_ROWS][j] == BLOCKED)
				printf("BBB");
			else
				printf("???");
		}
		else
		{
			if (horizSeg[NUM_ROWS][j] == UNBLOCKED)
				printf("---");
			else /* BLOCKED */
				printf("###");
		}
	}
	printf("+\n  ");

	/* wait for user input */
	pauseAndClear();

	return;
}

/* test to see that the route has been initialized correctly */
void displayRoute(void)
{
	int i; /* LCV */

	printf("    R C\n");
	printf("   =====\n");
	for (i = 0; i < NUM_NODES; ++i)
	{
		printf("%02d: %d %d\n", i, route[i][0], route[i][1]);
	}

	return;
}

/****************************************************************************
*****************************************************************************
   Function:  pauseAndClear
*****************************************************************************
Description:  Waits for the user to press the {ENTER} key, and then clears the
              screen.
 Input args:  none
Output args:  none
In/Out args:  none
     Return:  void
*****************************************************************************
****************************************************************************/
void pauseAndClear(void)
{
	char enter;
	enter = 0;

	while (enter != '\n' && enter != EOF)
		enter = getchar();

	system("clear");

	return;
}

/****************************************************************************
*****************************************************************************
   Function:  flushInputBuffer
*****************************************************************************
Description:  Flushes the console input buffer. C has no standard mechanism
              for this.
 Input args:  none
Output args:  none
In/Out args:  none
     Return:  void
*****************************************************************************
****************************************************************************/
void flushInputBuffer(void)
{
	char ch;
	ch = 0;

	/* flush the input buffer */
	while (ch != '\n' && ch != EOF)
		ch = getchar();

	return;
}

/* same as moveForward, except it calls checkForBlocksTEST instead of checkForBlocks */
void moveForwardTEST(void)
{
	/* check to make sure the robot can make the movement */
	if (segRel(FRONT) == UNBLOCKED)
    {
        /***code for bottomL_s and bottomR_s sensors COULD MAYBE go here***/
		if (direction == UP || direction == DOWN)
		{
			if (currentNode[0] % 2 == 0) /* currently in even-numbered row */
				evenToOddRow();
			else /* currently in odd-numbered row */
				oddToEvenRow();
		}
		else /* direction == LEFT || direction == RIGHT) */
		{
			changeColumn();
		}

		/* update currentNode array */
		if (direction == UP)
			currentNode[0] -= 1; /* decrement row */
		else if (direction == DOWN)
			currentNode[0] += 1; /* increment row */
		else if (direction  == LEFT)
			currentNode[1] -= 1; /* decrement column */
		else /* direction == RIGHT */
			currentNode[1] += 1; /* increment column */
                
		/* if the node has not been visited before, scan for blocks and set node in grid to VISITED */
		if (grid[currentNode[0]][currentNode[1]] == UNVISITED)
		{
			checkForBlocksTEST();
			grid[currentNode[0]][currentNode[1]] = VISITED;
		}

		/* if red vinyl is detected, sing and set global variable foundRedNode to TRUE */
		if (color_sTEST() == TRUE)
		{
			singTEST();
			foundRedNode = TRUE;
		}
    }

    return;
}

/* same as checkForBlocks, but it calls TEST versions of sensor functions */
void checkForBlocksTEST(void)
{
	/* define aliases for adjacent segments */
	Segment *top, *btm, *lft, *rgt;
	top = &(horizSeg[currentNode[0]][currentNode[1]]);
	btm = &(horizSeg[currentNode[0] + 1][currentNode[1]]);
	lft = &(vertSeg[currentNode[0]][currentNode[1]]);
	rgt = &(vertSeg[currentNode[0]][currentNode[1] + 1]);

	if (direction == UP)
	{
		if (*top != BLOCKED)
		{
			if (frontL_sTEST() == TRUE || frontR_sTEST() == TRUE)
				*top = BLOCKED;
			else
				*top = UNBLOCKED;
		}
		if (*lft != BLOCKED)
		{
			if (left_sTEST() == TRUE)
				*lft = BLOCKED;
			else
				*lft = UNBLOCKED;
		}
		if (*rgt != BLOCKED)
		{
			if (right_sTEST() == TRUE)
				*rgt = BLOCKED;
			else
				*rgt = UNBLOCKED;
		}
	}
	else if (direction == DOWN)
	{
		if (*btm != BLOCKED)
		{
			if (frontL_sTEST() == TRUE || frontR_sTEST() == TRUE)
				*btm = BLOCKED;
			else
				*btm = UNBLOCKED;
		}
		if (*rgt != BLOCKED)
		{
			if (left_sTEST() == TRUE)
				*rgt = BLOCKED;
			else
				*rgt = UNBLOCKED;
		}
		if (*lft != BLOCKED)
		{
			if (right_sTEST() == TRUE)
				*lft = BLOCKED;
			else
				*lft = UNBLOCKED;
		}
	}
	else if (direction == LEFT)
	{
		if (*lft != BLOCKED)
		{
			if (frontL_sTEST() == TRUE || frontR_sTEST() == TRUE)
				*lft = BLOCKED;
			else
				*lft = UNBLOCKED;
		}
		if (*btm != BLOCKED)
		{
			if (left_sTEST() == TRUE)
				*btm = BLOCKED;
			else
				*btm = UNBLOCKED;
		}
		if (*top != BLOCKED)
		{
			if (right_sTEST() == TRUE)
				*top = BLOCKED;
			else
				*top = UNBLOCKED;
		}
	}
	else /* direction == RIGHT */
	{
		if (*rgt != BLOCKED)
		{
			if (frontL_sTEST() == TRUE || frontR_sTEST() == TRUE)
				*rgt = BLOCKED;
			else
				*rgt = UNBLOCKED;
		}
		if (*top != BLOCKED)
		{
			if (left_sTEST() == TRUE)
				*top = BLOCKED;
			else
				*top = UNBLOCKED;
		}
		if (*btm != BLOCKED)
		{
			if (right_sTEST() == TRUE)
				*btm = BLOCKED;
			else
				*btm = UNBLOCKED;
		}
	}
	return;
}

/* this is the exact same function as traverseGrid except with the
   displayGrid function integrated for testing purposes
   BIG BUGS HERE!!! This is where the bulk of the work remains
*/
void traverseGridTEST(int **path, int size)
{
	int i; /* loop control variable */

	/* if current node is first node in path, execute body of function
	  (if current node is NOT the first node in path, followPath will NOT execute)*/
	if ((currentNode[0] == path[0][0]) && (currentNode[1] == path[0][1]))
	{   
		checkForBlocksTEST();
		for (i = 1; i < size && foundRedNode == FALSE; ++i)
		{
			/* next node is UP */
			if ((currentNode[0] == path[i][0] + 1) && (currentNode[1] == path[i][1]))
			{
				if (direction != UP)	/* turn to face UP */
				{
					turnAbs(UP);
					displayGrid();
				}
				
				/* move */
				if (horizSeg[currentNode[0]][currentNode[1]] == UNBLOCKED) /* clear path */
				{
					moveForwardTEST();
					displayGrid();
				}
				else /* blocked path */
				{
					avoidTEST(path[i]);
				}
			}
			/* next node is DOWN */
			else if ((currentNode[0] == path[i][0] - 1) && (currentNode[1] == path[i][1]))
			{
				if (direction != DOWN)	/* turn to face DOWN */
				{
					turnAbs(DOWN);
					displayGrid();
				}

				/* move */
				if (horizSeg[currentNode[0] + 1][currentNode[1]] == UNBLOCKED) /* clear path */
				{
					moveForwardTEST();
					displayGrid();
				}
				else /* blocked path */
				{
					avoidTEST(path[i]);
				}
			}
			/* next node is LEFT */
			else if ((currentNode[0]==path[i][0]) && (currentNode[1]==path[i][1] + 1))
			{
				if (direction != LEFT)	/* turn to face LEFT */
				{
					turnAbs(LEFT);
					displayGrid();
				}
				
				/* move */
				if (vertSeg[currentNode[0]][currentNode[1]] == UNBLOCKED) /* clear path */
				{
					moveForwardTEST();
					displayGrid();
				}
				else /* blocked path */
				{
					avoidTEST(path[i]);
				}
			}
			/* next node is RIGHT */
			else if ((currentNode[0]==path[i][0]) && (currentNode[1]==path[i][1] - 1))
			{
				if (direction != RIGHT)	/* turn to face RIGHT */
				{
					turnAbs(RIGHT);
					displayGrid();
				}

				/* move */
				if (vertSeg[currentNode[0]][currentNode[1] + 1] == UNBLOCKED) /* clear path */
				{
					moveForwardTEST();
					displayGrid();
				}
				else /* blocked path */
				{
					avoidTEST(path[i]);
				}
			}
			else {}
				/* no error-handling now */
                
		} /* for */
	} /* if */

	return;
}

/* navigates around the block *in front* of the robot if it can */
void avoidTEST(int destination[])
{
	if (currentNode[1] < NUM_COLS - 1) /* not in right-most column */
	{
		/* attempt to move around the right side of obstacle */
		maneuverRightTEST(destination);
	}
	else /* in right-most column */
	{
		/* attempt to move around the left side of obstacle */
		maneuverLeftTEST(destination);
	}

	return;
}

void maneuverRightTEST(int destination[])
{
	/* these variables are used for potential back-tracking */
	int i, j = 0; /* LCV's */
	int boundary;
	Bool failedAttempt = FALSE;
	int pathSize = 1;
	int tempPath[NUM_NODES][2];
	int **backtrackPath = (int**)malloc(NUM_NODES * sizeof(int*));
	for (i = 0; i < NUM_NODES; ++i)
		backtrackPath[i] = (int*)malloc(2 * sizeof(int));
	tempPath[0][0] = currentNode[0];
	tempPath[0][1] = currentNode[1];

	if (direction == DOWN) /* heading from UP to DOWN */
		boundary = 0; /* row 0 */
	else /* heading from DOWN to UP */
		boundary = NUM_ROWS - 1; /* row NUM_ROWS - 1 */

	turnAbs(RIGHT);
	do
	{
		if (segRel(LEFT) == UNBLOCKED)
		{
			turnLeft();
			moveForwardTEST();

			tempPath[pathSize][0] = currentNode[0];
			tempPath[pathSize][1] = currentNode[1];
			++pathSize;
		}
		else if (segRel(FRONT) == BLOCKED)
		{
			turnRight();
		}
		else
		{
			moveForwardTEST();

			tempPath[pathSize][0] = currentNode[0];
			tempPath[pathSize][1] = currentNode[1];
			++pathSize;
		}

		/* if maneuver cannot be completed */
		if (currentNode[0] == boundary && segRel(RIGHT) == BLOCKED)
		{
			failedAttempt = TRUE;
		}

	} while ((currentNode[0] != destination[0] || currentNode[1] != destination[1])
				&& failedAttempt == FALSE);

	/* backtrack if necessary */
	if (failedAttempt == TRUE)
	{
		/* copy reverse order of tempPath (path taken by robot to attempt to maneuver
		   around obstacle) into backtrackPath */
		for (i = pathSize - 1; i >= 0; --i)
		{
			backtrackPath[j][0] = tempPath[i][0];
			backtrackPath[j][1] = tempPath[i][1];
			++j;
		}

		followPath(backtrackPath, pathSize);
	}

	return;
}

void maneuverLeftTEST(int destination[])
{
	return;
}

/* returns status of adjacent segment, specified by absolute Direction
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegAbs(Direction absDir)
{
	if (absDir == UP)
		return blockedHorizSeg[currentNode[0]][currentNode[1]];
	else if (absDir == DOWN)
		return blockedHorizSeg[currentNode[0] + 1][currentNode[1]];
	else if (absDir == LEFT)
		return blockedVertSeg[currentNode[0]][currentNode[1]];
	else /* absDir == RIGHT */
		return blockedVertSeg[currentNode[0]][currentNode[1] + 1];
}

/* returns status of adjacent segment, specified by relative Direction
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegRel(Direction relSeg)
{
	Segment returnValue;

	/* currently facing absolute UP */
	if (direction == UP)
	{
		returnValue = blockedSegAbs(relSeg); /* same as blockedSegAbs function */
	}
	/* currently facing absolute DOWN */
	else if (direction == DOWN)
	{
		if (relSeg == FRONT) /* absolute DOWN */
			returnValue = blockedSegAbs(DOWN);
		else if (relSeg == BACK) /* absolute UP */
			returnValue = blockedSegAbs(UP);
		else if (relSeg == LEFT) /* absolute RIGHT */
			returnValue = blockedSegAbs(RIGHT);
		else /* relSeg == RIGHT ---> absolute LEFT */
			returnValue = blockedSegAbs(LEFT);
	}
	/* currently facing absolute LEFT */
	else if (direction == LEFT)
	{
		if (relSeg == FRONT) /* absolute LEFT */
			returnValue = blockedSegAbs(LEFT);
		else if (relSeg == BACK) /* absolute RIGHT */
			returnValue = blockedSegAbs(RIGHT);
		else if (relSeg == LEFT) /* absolute DOWN */
			returnValue = blockedSegAbs(DOWN);
		else /* relSeg == RIGHT ---> absolute UP */
			returnValue = blockedSegAbs(UP);
	}
	/* currently facing absolute RIGHT */
	else /* direction == RIGHT */
	{
		if (relSeg == FRONT) /* absolute RIGHT */
			returnValue = blockedSegAbs(RIGHT);
		else if (relSeg == BACK) /* absolute LEFT */
			returnValue = blockedSegAbs(LEFT);
		else if (relSeg == LEFT) /* absolute UP */
			returnValue = blockedSegAbs(UP);
		else /* relSeg == RIGHT ---> absolute DOWN */
			returnValue = blockedSegAbs(DOWN);
	}

	return returnValue;
}

void singTEST(void)
{
	printf("\a"); /* alarm character code */
	return;
}

/* TO SENSE VINYL: TRUE -- vinyl is sensed underneath robot */
Bool color_sTEST(void) /* for red square underneath robot */
{
	if (currentNode[0] != redNode[0] || currentNode[1] != redNode[1])
		return FALSE;
	else
		return TRUE;
}
/* To test the functionality of bottomL_s and bottomR_s in the virtual grid,
   perhaps a random number generator could be utilized to tilt the robot
   off-center at random times. SOMETHING TO DO!!! */
Bool bottomL_sTEST(void)	/* for vinyl on the front-left */
{
	/* this function is not fully developed */
	return FALSE; /* dummy value */
}
Bool bottomR_sTEST(void)	/* for vinyl on the front-right */
{
	/* this function is not fully developed */
	return FALSE; /* dummy value */
}
/* TO SENSE BLOCKS: TRUE -- block is sensed in adjacent segment */
Bool left_sTEST(void)		/* for blocks to the left */
{
	if (blockedSegRel(LEFT) == UNBLOCKED) /* representation of sensor code */
		return FALSE;
	else
		return TRUE;
}
Bool right_sTEST(void)		/* for blocks to the right */
{
	if (blockedSegRel(RIGHT) == UNBLOCKED) /* representation of sensor code */
		return FALSE;
	else
		return TRUE;
}
Bool frontL_sTEST(void)		/* for blocks to the front-left */
{
	if (blockedSegRel(FRONT) == UNBLOCKED) /* representation of sensor code */
		return FALSE;
	else
		return TRUE;
}
Bool frontR_sTEST(void)		/* for blocks to the front-right */
{
	return frontL_sTEST(); /* same virtual representation */
}

/* FOR DEMONSTRATION PURPOSES ONLY!!! */
void easyReturnTEST(void) /* assuming no obstacles, moves robot back to starting node */
{
	if (currentNode[0] != 0) /* get to row 0 */
	{
		turnAbs(UP);
		displayGrid();
		do
		{
			moveForwardTEST();
			displayGrid();
		} while (currentNode[0] != 0);
	}

	if (currentNode[1] != 0) /* get to column 0 */
	{
		turnAbs(LEFT);
		displayGrid();
		do
		{
			moveForwardTEST();
			displayGrid();
		} while (currentNode[1] != 0);
	}

	return;
}
void easyReturn(void) /* assuming no obstacles, moves robot back to starting node */
{
	if (currentNode[0] != 0) /* get to row 0 */
	{
		turnAbs(UP);
		do
		{
			moveForward();
		} while (currentNode[0] != 0);
	}

	if (currentNode[1] != 0) /* get to column 0 */
	{
		turnAbs(LEFT);
		do
		{
			moveForward();
		} while (currentNode[1] != 0);
	}

	return;
}

// TEST FUNCTIONS NEED TO BE WRITTEN AFTER FINISHED TESTING:
void avoid(int destination[])
{
	return;
}
void maneuverRight(int destination[])
{
	return;
}
void maneuverLeft(int destination[])
{
	return;
}
void traverseGrid(int **path, int size)
{
	return;
}
