#Path planning project

[//]: # (Image References)
[example]: ./example2.png

This is an overview about my solution of the path-planning project in Udacity Car-ND. The goal of the project is to guide a car along a highway with three lanes. On the highway there are a bunch of other cars driving. The path-planning should allow to drive along the highway with about 50 mph. If possible, traffic should be passed so that the car can follow the highway as fast as possible.

The solution is structured into three different parts. First i'm describing in short the values we're getting from the environment. Then i'll explain how the car decides which of the lanes it takes and why. Furthermore i'm explaining how the resulting speed on the lane is identified. IN the last section i'm explaining how our decision is resulting in a trajectory the car is following. 

## Data-Input
The project is using the Udacity car simulator. The simulator is providing a bunch of information which are stored in different structs. There is a waypoint list which describing the track - the waypoints are used to describe the path the car should take. The waypoint list is static and is not changing over time. I'm storing this information in the `SMapWaypoint` structure.

The simulator itself provides a bunch of dynamic data. On the one hand, there is the current car position described in global map coordinates as well as in frenet s-d coordinates. This information is received every update cycle and is stored in the `SCarPos` struct. Then there is the sensor fusion data. This structure contains all the information about other traffic - we're using this information later to identify the successor and predecessor vehicle next to our car. In a very early step, we're filtering the list of sensor fusion data (a total of 12 vehicles) to a smaller number (in an area of about 1000 meter around our current car position).

##Path planning
Our goal is to find a fast path through the traffic. Therefore we're planning our car-position to a car-position in future - a future of 1 second. To do so, we're using the so called `SForecastState`. The `SForecast` state contains the information about our car in a 1-second horizont. It contains a member offset, which represents the number of projection points of our previous path (we're aiming to have a total of 50 projection points within a horizont of 1 second). Furthermore it contains the information of the future car s and d frenet coordinates. Last but not least, it has a member which represents the middle of the lane we'll be in future. The forecast state is resetted with every update cycle.
In order to avoid the collission with other cars, we first analyze the given traffic and place the vehicles in our surrounding to the corresponding lanes. This is done during the `prepareLanes` method. The method is very simple - it takes all the filtered sensor-fusion data, identifies for each vehicle the lane it is currently located on and stored the velocity as well as the distance to our "future" car-position. This information is stored in the so called `SLane` structures. 
The decision which lane to chose is done by iterating through our prepared lanes and identiy for each of the lanes the "preference value". This value is more or less the sum of the possible speed on a lane over a timing horizont of 3 seconds. The preference values is calculated by using the `speedvalueDevelopmentOnLane` function. It calculates the position of my car within a horizont of 3 seconds as well as the position of surrounding cars on the corresponding lane. The velocity allowed on the lane for each of the calculated position (a total 9) is identified by a very simple rule (as shown in the Q&A of the project). 

If the distance to the predecessor car is larger than a so called *securityBelt* and the velocity is below the max velocity of 50 mph, we add 0.224 mph to the velocity. If we don't meet the first criteria only, we're staying with the given velocity. If none of the two criterias is true, we're reducing the velocity by 0.224 mph to avoid a collosion on that lane.
Summing up these velocities for a total of 9 points provides us different preference values for the lanes. The preference value will tell us which lane we should try to take. Imagine a lane with a car which is slow and close to us with a lane that contains no other car or a car in the far distance. In general, we would prefer the later lane - and that's exactly the preference value provides to us.

One remark to the securityBelt: I've followed more or less the ruleset we have in germany - taking half of the speedometer in meter distance. In my case i take a third of the speedometer.

The `speedvalueDevelopmentOnLane` provides beneath the preference value the allowed velocity for the current projection in future. Remember, we're always calculating the car-position for the future 1 second later.

##Decision making
We're calculating the speed and preference value for each lane. Futhermore we check if a lane is allowed for us to drive. This is done in the function `laneChangeAllowed`. The function takes the projected car position and speed and compares the new car-position with the car-positions of the other vehicles on the lane. If we're not violating the securityBelt distance, a lane is safe for us and we're allow to change on it. An exception is the current lane which is always valid.

So which lane are we chosing: we take the lane which has the highest preference value and is valid for us to drive. In case that we have to pass two lanes at onec (imagine changing from lane 0 to lane 2), we mark the goal lane as `intended` lane and chose the lane in between as `current` lane. As soon as we can, we then change to the next lane. In order to avoid jerk (switching back and forth between lanes), a counter is used, the `laneChangeBuffer` to guarantee that after every lane change we stay at least for 400ms on that lane.

With the velocity and the planned lane we're calculating our trajectory.

##Trajectory calculation
The trajectory calculation is following the suggestion using splines described in the Q&A section of the project intro. Is is done during the `calcNextXY` method. In essence we're taking the path we've calculated so far in previous cycle and enrich the number of waypoints in the path to 50 in total. The additional points are calculated by using the projected car position and identify the car's position in 30,60, 90 meters following a certain lane (our chosen lane) with respect to the waypoints. This set of points is used to calculate a spline (by the  [spline.h library](http://kluge.in-chemnitz.de/opensource/spline/)). We're identifying 50 points on that spline with a total distance of 30 meter. The previous trajectory path is filled with the just calculated points to a total number of 50 points.
This spline trajectory calculation guarantees a smooth transition from one lane to the other.


The resulting trajectory if forwarded back to the simulator.


![Pathplanner in action][example]

##Summary
The solution describes a simple path planner which allows a car to drive a long a highway. It considers the vehicles in its closer surrounding to find a safe and fast path along the highway.
The path planner is just a simple approach. It doesn't consider the possible behaviour of other vehicles on the highway - e.g. sudden lanechange or similar. Furthermore the acceleration model is very straight and simple - it doesn't allow any emergency handling (full stopp). 


* * *

##Review and rework
The first submission failed the project criteria. The reviewer mentioned two findings:
* max velocity of 50mph was exceeded
* during the testrun it came to a crash with another vehicle.

So is spended more time, revieweing my solution and find the weak spots. Concerning the max velocity - i didn't find any obvious misbehaviour in my sources. All velocity calculation was always based on the restriction, that the max-velocity (set to 50mph) must not be violated. I never faced the problem during my testruns. So the only countermeasurement I did therefore was, to reduce the max velocity to 49.5 mph. This give a little velocity buffer and should avoid any further max-speed violation.

Concerning the crash with other vehicles: unfortunetely the situation isn't completely clear for me. From the screenshots given in the review, it looks like our vehicle runs into the red-forein vehicle without facing any certain situation. In the sources, the `laneChangeAllowed` function has exactly the function, to avoid such situation: this function always takes the projected car position into consideration and compares the position with the projected car positions of the closest vehicles. There are two possible clashes in my assumption:
first of all i'm only considering the closest neighboors - nobody else. Which might lead to a situation in which i'm assuming, the lane is free - because I've almost passed the car, but there is another car directly next to the one.
I've taken care of that by now reflecting all vehicles in my surroundings. That means instead of checking the distances to one predecessor and one successor, i'm comparing the carposition with each successor/ predecessor of the lane i would lake to move to. This affects all the functions `prepareLanes`, `laneChangeAllowed` and `speedvalueDevelopmentOnLane`.

The second weak point is the distance to the car i'm planning to pass. Due to the fact that I'm reducing the security belt in case that I plan to pass a car to about 7 meter, I might run into the situation that I'm hitting the obstacle while moving from one lane to the other. So I've disabled this feature and keept the rule of thumbs: distance to other cars must be half of speedometer in meter.

