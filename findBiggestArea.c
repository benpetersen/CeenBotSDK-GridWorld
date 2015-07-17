

/*
This will calculate the highest number of adjacent cells, just pseodocode for right now. I'm almost positive this won't get all possibilities
It might not do well when we are treversing the route and find a wall, might need to get around it, or choose a different area to go in

ROW MAJOR: calculates the total, by going through each position, row by row, col by col (descending, since the zig-zag pattern *should lead us to the top right

*/

int row = currentNode[0];
int col = currentNode[1];

int left[];
int right[];
int total[]; //store each of the rows' totals
int area[];

for( row--) //go backwards from our current row position, in order to get the values we can go to.
{
	for(col++)
	{
		if(grid[row][col] == true || weHitEdge == true) // found a spot or *edge we could start our calculations on
		{
			//store left and right values in that row
			if(right[row] == null) //right value for this row is null
				right[row] = col; //store column position into right
			else if(left[row] == null)
				left[row] = col;
			
		}
	}
	if(right[row] == null) //we weren't able to traverse the row by the first movement through the grid
	{
		//store right as the far right edge
		right[row] = 8;
	}
	if(left[row] == null)
	{
		left[row] = 0; 
	}
	//important check to make sure left isn't next to right, we'll need to stop it here because that's when the area *stops
	
	if(left[row]+1 >= right[row])
	{
		//stop it here, calculate this groups area
		area[i] = calculateTotalForThisArea(total[]) //add up the totals' for each row to get this area.
	}
	else
	{
		total[row] = calculateTotalForThisRow(left[], right[]); //right - left to get the total for this row
		//iterate again, since the area is still in one piece.
	}
}

//Then we'll need to do this Column Major wise, in order to get the values to the bottom 