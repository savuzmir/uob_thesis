# UoB Thesis 

Note: For now, all files are created as independent c++ files. Once the basic functionality will work, I will begin integrating/merging the relevant parts to make them Golem compatible and will remove the independent temporary implementations (i.e. the trajectory creator) that will not be needed in the final system. These will also be then integrated with Golem (i.e. as additional functionality within packages/Plugin/Data)

## TODO:
- Test and debug the complete cycle. 
- Finish the plotting function and make the required animation
- Make the code more efficient and less wasteful. Most likely, a lot more memory than needed is being used in parts at the moment due to me not being familiar with best practices
- Use SFLM Library through which user input should be obtained 
- Implement the function that checks for matrix sizes 
- Integrate with Golem 

## Done:
- LQR class that correctly computes LQR and saves it in a map of trajectories. Example:
FeedbackMatrixMap["A5"] would return the corresponding K matrix for the 5th waypoint of trajectory A. The names are completely arbitrary. 
- Trajectory creator (CreateTrajectory) will create multiple trajectories saved to a file. These are then read 
- Utility function that checks correct size 
- Compared my implementation of the SolveDARE with github.com/RobotLocomotion/drake and it gives nearly identical results, used theirs due to additional checks that are inside. 
- Made typedefs for all continously used data structures and added iterator for loops
- included header guards to the relevant files
- Made the nearest neighbour computation handle more than one nearest neighbour. In that case it will conservatively pick the     trajectory that has been prefered so far by the user.
- Substantially overhauled the nearest neighbour (i.e. the way it parses trajectories and picks the current trajectory) 
- A complete cycle of 1. import trajectories (this will be replaced with Golem when I will start integrating it), 2. compute Kmatrix for all trajectories, waypoints, and name it correspondingly, 3. wait for user input and obtain system state that are then used, 4. find nearest waypoint of a trajectory, make prediction of the next state and compute the optimal input for the desired trajectory and feed that back. 

