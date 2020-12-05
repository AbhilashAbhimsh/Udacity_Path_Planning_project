#### Algorithm:

The FLow can be divided into 2 Sets
1> Behavior planning
2> Trajectory Generation

## Behavior Planning Takes place in following steps:

1 > Checking if there is car_ahead [main.cpp Line:116-137]

2> If there is any Car ahead what to do next?
	
    2a> Check for any neighbouring lanes in the Drive side to perform Lane 					change[main.cpp Line:143]
	
	2b> Determine if we can make the Lane change to that future lanes by checking if 		there is any car in the front of that Lane if yes Dont perform Lane Change to 		 that Lane and also Check if there is any car behind in that Future lane which 		   is too close, that might collide when our car while performing Lane Change 		   [main.cpp Line:145]
	
	2c> If we have 2 Lanes that we can perform lane change then we can check which 			lane has less traffic ahead and select a lane which has less traffic in case 		 we have 2 lanes which are safe to perform Lane Change [main.cpp Line:147]
	
    2d> If Lane Change is not feasible then we need to progeressively reduce speed to   	  maintain Safe distance from Car Ahead of our Lane.[main.cpp Line:177-182]
	
## Trajectory Generation:

Based on the Future Lane Check Generate a Trajectory by using following Steps:
1> First determine 5 points that we help in Fitting a smooth Polynomial to help generate trajectory[main.cpp Line:196-204]

2> Determine the distance Gap between the Trajectory Points based on the current speed and time.[main.cpp Line:215-217]

3> Include the left over Trajectory points that are yet to be traversed for continuity and smooth transition between      	successive transition of Trajectory Generation.[main.cpp Line:223-227]

4 > Based on the spacing between the points generate the rest of the trajectories and add to the list for the simulator to execute.[main.cpp Line:230-258]


#### key Observations:
-> If we reduce the number of values to next_x_vals and next_y_vals we can always make sudden Manuevers. Like If the car infront of our car that just changed lane to our lane. [Optimum Value is between 15 and 30]