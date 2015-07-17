#include <stdio.h>	/* printf, scanf -- SAM ONLY */
#include <stdlib.h> /* system("pause") -- SAM ONLY */

/*------------------------------------------------------GLOBAL VARIABLE DECLARATIONS-----------------------------------------------------------*/
/* Constants: */
/* ========== */
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

/* Position and Orientation of Robot: */
/* ================================== */
int current[2];					/* [0]: current row, [1]: current column -- updates in moveForward() */
Direction direction;			/* stores current direction -- updates in turn functions */

/* Environmental Information: */
/* ========================== */
Bool foundRedNode;							/* FALSE until red vinyl is sensed, then set to TRUE */
Node grid[NUM_ROWS][NUM_COLS];				/* 2D array comprised of every Node in grid; either UNVISITED or VISITED -- updates in moveForward() */
Segment horizSeg[NUM_ROWS + 1][NUM_COLS];	/* 2D array comprised of all horizontal Segments; BLOCKED, UNBLOCKED, or IDK -- updates in checkForBlocks() */
Segment vertSeg[NUM_ROWS][NUM_COLS + 1];	/* 2D array comprised of all vertical Segments; BLOCKED, UNBLOCKED, or IDK -- updates in checkForBlocks() */
Bool adjacencyMatrix[NUM_ROWS][NUM_COLS][NUM_ROWS][NUM_COLS]; /* used for very inefficient implementation of Dijkstra's algorithm */

/* Data Structure(s) used in Instruction Routines: */
/* =============================================== */
int **route;	/*	Array of Nodes to be visited sequentially by robot.
					
					Should be "int route [NUM_NODES][2];", but in order
					for the followPath function to work, a pointer declaration
					is needed.

					Ex:	route[0][0] indicates row # of first node in path
						route[0][1] indicates col # of first node in path	*/

/* Virtual Objects -- VIRTUAL GRID ONLY */
/* ==================================== */
/* The following 2 arrays are representations of locations of physical obstacles, which may or may not
   have been sensed by the robot. Up to 13 segments will be blocked based on competition specification of blocks. */
Segment blockedHorizSeg[NUM_ROWS + 1][NUM_COLS];
Segment blockedVertSeg[NUM_ROWS][NUM_COLS + 1];
int redNode[2];		/* location of red floor tile in grid */

/*---------------------------------------------------------FUNCTIONS DECLARATIONS-----------------------------------------------------------*/
/* Initialization and Deallocation: */
/* ================================ */
void initializeGlobalVariables(void);
void initializeTestVariables(void);	/* initializes variables used only in virtual grid */
void initZigZagRoute(void);
void deallocateMemory(void);

/* Actuators: */
/* ========== */
void turnAbsTEST(Direction);		/* turns robot in absolute Direction specified by input argument */
void turnLeftTEST(void);			/* turns robot 90 degrees to relative LEFT */
void turnRightTEST(void);			/* turns robot 90 degrees to relative RIGHT */
void turn180TEST(void);				/* turns robot 180 degrees in relative orientation */
void moveForwardTEST(void);			/* checks to see if segment in relative FRONT is UNBLOCKED, and then moves to that node */
void moveBackwardTEST(void);		/* simply moves backward (no sensor checks) */
void followPathTEST(int **, int);	/* moves */
void easyReturnTEST(void);			/* DEMONSTRATION ONLY: makes robot move from current Node back to Node [0][0], assuming no blocks on grid */
void singTEST(void);				/* makes virtual robot send a beep to the console */

/* Sensors: */
/* ======== */
void checkForBlocksTEST(void);			
Bool color_sTEST(void);		/* for red square underneath robot */
Bool bottomL_sTEST(void);	/* for vinyl on the front-left */
Bool bottomR_sTEST(void);	/* for vinyl on the front-right */
Bool left_sTEST(void);		/* for blocks to the left */
Bool right_sTEST(void);		/* for blocks to the right */
Bool frontL_sTEST(void);	/* for blocks to the front-left */
Bool frontR_sTEST(void);	/* for blocks to the front-right */

/* Robot Memory Check Test Functions */
/* ================================= */
Segment segAbs(Direction);	/* returns status of adjacent segment, absolute Direction */
Segment segRel(Direction);	/* returns status of adjacent segment, relative Direction */

/* Console Output Test Functions */
/* ============================= */
void displayGrid(void);			/* virtual grid */
void displayRoute(void);		/* displays route to screen */
void pauseAndClear(void);		/* used in displayGrid and displayRoute function */
void flushInputBuffer(void);

/* Virtual Object Presence Functions */
/* ================================= */
Segment blockedSegAbs(Direction);	/* indicates presence of virtual grid block on segment adjacent to current Node; absolute Direction */
Segment blockedSegRel(Direction);	/* indicates presence of virtual grid block on segment adjacent to current Node; absolute Direction */

/*---------------------------------------------------------MAIN ROUTINE-----------------------------------------------------------------*/
int main(void)
{
	/* TEST DECLARATIONS -- at top of main function */
	/* no declarations right now */

	/* initialize: */
	initializeGlobalVariables();

	/* initialize: */
	initializeTestVariables();

	/* these functions should always go first when testing with virtual grid */
	system("clear");
	displayGrid();
	checkForBlocksTEST(); /* check for blocks surrounding start space -- MANDATORY */

	/* this always comes before the return statement */
	deallocateMemory();

	return 0;
}

/*---------------------------------------------------------INITIALIZATION AND DEALLOCATION FUNCTIONS-----------------------------------------------------------*/

/* FUNCTION: initializeGlobalVariables
   initializes all global variables common to both the actual robot and virtual grid:

	Variable:		Meaning:
	=========		========
    foundRedNode	FALSE until the robot finds the red vinyl, in which case it is set to TRUE in moveForward()
	direction		Direction that robot is currently facing -- updated in turn functions
	current[0]		Number of row currently occupied by robot -- updated in moveForward()
	current[1]		Number of column currently occupied by robot -- updated in moveForward()
	grid[][]		2D array of Nodes, either VISITED or UNVISITED
	horizSeg[][]	2D arrays of Segments, either UNBLOCKED, BLOCKED, or IDK -- updated in checkForBlocks,
	verSeg[][]			which is called in moveForward()
	route			Planned array of Nodes to be traveled sequentially by robot -- route still needs to be implemented somewhere
*/
void initializeGlobalVariables(void)
{
	int i, j;	/* loop control variables */
	
	/* haven't found finish space yet */
	foundRedNode = FALSE;

	/* current orientation and position of robot */
	direction = DOWN;
	current[0] = 0; current[1] = 0;

	/* initialize all grid Nodes to UNVISITED */
	for (i = 0; i < NUM_ROWS; ++i)
	{
		for (j = 0; j < NUM_COLS; ++j)
			grid[i][j] = UNVISITED;
	}

	/* initialize grid Nodes that have been “visited”
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

/* FUNCTION: initializeTestVariables
   initializes all global variables used only in virtual grid:

   	Variable:			Meaning:
	=========			========
	redNode[0]			virtual representation of physical row occupied by floor tile with red vinyl
	redNode[0]			 "       "              "  "       column " " " " " " "
	blockedHorizSeg[][]	2D arrays containing virtual representations of either the absense or presense of physical blocks	
	blockedVertSeg[][]      on each Segment in the grid

possible orientations of blocks:
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

/*-------------------------------------------------------------------SENSOR TEST FUNCTIONS------------------------------------------------------------------*/

/* if adjacent Segments are not BLOCKED (either IDK or UNBLOCKED), sensor functions are called to sense surrounding segments (UNBLOCKED Segments are tested
   to increase out chances of not missing a block) */
void checkForBlocksTEST(void)
{
	/* define aliases for adjacent segments */
	Segment *top, *btm, *lft, *rgt;
	top = &(horizSeg[current[0]][current[1]]);
	btm = &(horizSeg[current[0] + 1][current[1]]);
	lft = &(vertSeg[current[0]][current[1]]);
	rgt = &(vertSeg[current[0]][current[1] + 1]);

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

/* TO SENSE VINYL: TRUE -- vinyl is sensed underneath robot */
Bool color_sTEST(void) /* for red square underneath robot */
{
	if (current[0] != redNode[0] || current[1] != redNode[1])
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

/*--------------------------------------------------------------ROBOT MEMORY CHECK TEST FUNCTIONS-----------------------------------------------------------*/
/*-------------------------------------------(checks global variables representing grid objects: Nodes, Segments, etc)--------------------------------------*/

/* returns status of adjacent segment, specified by absolute Direction */
Segment segAbs(Direction absDir)
{
	if (absDir == UP)
		return horizSeg[current[0]][current[1]];
	else if (absDir == DOWN)
		return horizSeg[current[0] + 1][current[1]];
	else if (absDir == LEFT)
		return vertSeg[current[0]][current[1]];
	else /* absDir == RIGHT */
		return vertSeg[current[0]][current[1] + 1];
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

/*------------------------------------------------------------------ACTUATOR TEST FUNCTIONS-----------------------------------------------------------------*/

/* turn robot to face specified absolute Direction */
void turnAbsTEST(Direction dirToFace)
{
	/* want to face up */
	if (dirToFace == UP)
	{
		if (direction == DOWN)
			turn180TEST();
		else if (direction == LEFT)
			turnRightTEST();
		else if (direction == RIGHT)
			turnLeftTEST();
		/* else (direction == UP) ---> no need to turn */
	}
	/* want to face down */
	else if (dirToFace == DOWN)
	{
		if (direction == UP)
			turn180TEST();
		else if (direction == LEFT)
			turnLeftTEST();
		else if (direction == RIGHT)
			turnRightTEST();
		/* else (direction == DOWN) ---> no need to turn */
	}
	/* want to face left */
	else if (dirToFace == LEFT)
	{
		if (direction == UP)
			turnLeftTEST();
		else if (direction == DOWN)
			turnRightTEST();
		else if (direction == RIGHT)
			turn180TEST();
		/* else (direction == LEFT) ---> no need to turn */
	}
	/* want to face right */
	else /* dirToFace == RIGHT */
	{
		if (direction == UP)
			turnRightTEST();
		else if (direction == DOWN)
			turnLeftTEST();
		else if (direction == LEFT)
			turn180TEST();
		/* else (direction == RIGHT) ---> no need to turn */
	}
 
	return;
}

/* turns robot 90 degrees to relative LEFT */
void turnLeftTEST(void)
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
void turnRightTEST(void)
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
void turn180TEST(void)
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

/* if the relative FRONT Segment is UNBLOCKED (no blocks and not on edge of grid), move forward one Node; then if the current Node has not been VISITED
   before, check for surrounding blocks and update the grid and Segment arrays; then check for the red vinyl--if found, update the global foundRedNode
   variable and sing */
void moveForwardTEST(void)
{
	/* check to make sure the robot can make the movement */
	if (segRel(FRONT) == UNBLOCKED)
    {
		/* update current array */
		if (direction == UP)
			current[0] -= 1; /* decrement row */
		else if (direction == DOWN)
			current[0] += 1; /* increment row */
		else if (direction  == LEFT)
			current[1] -= 1; /* decrement column */
		else /* direction == RIGHT */
			current[1] += 1; /* increment column */
                
		/* if the node has not been visited before, scan for blocks and set node in grid to VISITED */
		if (grid[current[0]][current[1]] == UNVISITED)
		{
			checkForBlocksTEST();
			grid[current[0]][current[1]] = VISITED;
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

/* move from the current space to the one behind the robot -- MAY BE AN UNNECESSARY FUNCTION */
void moveBackwardTEST(void)
{
	/* update current array */
	if (direction == UP)
		current[0] += 1; /* increment row */
	else if (direction == DOWN)
		current[0] -= 1; /* decrement row */
	else if (direction == LEFT)
		current[1] += 1; /* increment column */
	else /* direction == RIGHT */
		current[1] -= 1; /* decrement column */

    return;
}



/*  path[index][0: row /// 1: col]
	index:		number of nodes; 0 = first node (current node), 1 = second node, size - 1 = last node
	row/col:	0 = row #, 1 = column #
	size:		length of path array
*/
void followPathTEST(int **path, int size)
{
	int i; /* loop control variable */

	/* if current node is first node in path, execute body of function
	  (if current node is NOT the first node in path, followPath will NOT execute)*/
	if ((current[0] == path[0][0]) && (current[1] == path[0][1]))
	{         
		for (i = 1; i < size; ++i)
		{
			/* TURN TO FACE APPROPRIATE DIRECTION */
			/* next node is UP */
			if ((current[0] == path[i][0] + 1) && (current[1] == path[i][1]))
			{
				turnAbsTEST(UP);
				displayGrid();
			}
			/* next node is DOWN */
			else if ((current[0] == path[i][0] - 1) && (current[1] == path[i][1]))
			{
				turnAbsTEST(DOWN);
				displayGrid();
			}
			/* next node is LEFT */
			else if ((current[0]==path[i][0]) && (current[1]==path[i][1] + 1))
			{
				turnAbsTEST(LEFT);
				displayGrid();
			}
			/* next node is RIGHT */
			else if ((current[0]==path[i][0]) && (current[1]==path[i][1] - 1))
			{
				turnAbsTEST(RIGHT);
				displayGrid();
			}
			else {}
				/* no error-handling now -- note that moveForward always executes */
                
			/* move to next node and scan for surrounding blocks if node has not been visited */
			moveForwardTEST();
			displayGrid();
		} /* for */
	} /* if */

	return;
}

/* FOR DEMONSTRATION PURPOSES ONLY -- assuming no obstacles, moves robot back to starting node */
void easyReturnTEST(void)
{
	if (current[0] != 0) /* get to row 0 */
	{
		turnAbsTEST(UP);
		displayGrid();
		do
		{
			moveForwardTEST();
			displayGrid();
		} while (current[0] != 0);
	}

	if (current[1] != 0) /* get to column 0 */
	{
		turnAbsTEST(LEFT);
		displayGrid();
		do
		{
			moveForwardTEST();
			displayGrid();
		} while (current[1] != 0);
	}

	return;
}

/* emulates singing by sounding a console alarm -- works in Linux at least */
void singTEST(void)
{
	printf("\a"); /* alarm character code */
	return;
}

/*---------------------------------------------------------------CONSOLE OUTPUT TEST FUNCTIONS--------------------------------------------------------------*/

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
			if (i == current[0] && j == current[1]) /* display robot */
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

/*------------------------------------------------------------VIRTUAL OBJECT PRESENCE FUNCTIONS-------------------------------------------------------------*/

/* returns status of adjacent segment, specified by absolute Direction (think of UP, RIGHT, DOWN, and LEFT and North, 
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegAbs(Direction absDir)
{
	if (absDir == UP)
		return blockedHorizSeg[current[0]][current[1]];
	else if (absDir == DOWN)
		return blockedHorizSeg[current[0] + 1][current[1]];
	else if (absDir == LEFT)
		return blockedVertSeg[current[0]][current[1]];
	else /* absDir == RIGHT */
		return blockedVertSeg[current[0]][current[1] + 1];
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
