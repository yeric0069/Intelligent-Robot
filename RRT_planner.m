function [pthObj, solnInfo] = RRT_planner(start, goal, map)

% set 2 demensions vector for map
ss = stateSpaceSE2;

% generate a validator for this map
sv = validatorOccupancyMap(ss); 

% load map
sv.Map = map; 

% set smallest safe distance between vehicle and occupied 
sv.ValidationDistance = 0.1; 

% limit state boundary
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

% genetare a RRT planner
myplannerRRT = plannerRRT(ss,sv); 

% set maximum connection distance of planner
myplannerRRT.MaxConnectionDistance = 0.3;
myplannerRRT.MaxIterations = 5000;
myplannerRRT.ContinueAfterGoalReached = true;   

% return the path object and solution information
[pthObj,solnInfo] = plan(myplannerRRT,start,goal);