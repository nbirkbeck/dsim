# DSim - a simple traffic simulator

Dependencies:

 * Bazel
 * GLog
 * Proto
 * https://github.com/nbirkbeck/nimage


## Simulator Overview

The simulator has a simple level representation and cars that randomly drive in the
map.
 

### Level representation

The *level* consists of roadways, parking lots, and traffic controls (e.g., 
stop signs or lights). Cars originate in one of the parking lots, and drive to a random 
location.

#### Roads

* Road's are one-way and consist of a bunch of Road Segments. Each Road Segment
is an ordered array of 2D points and has a speed limit that the Cars will try to obey.

#### Parking lots

* A parking lot is simply a rectangular region with a bunch of parking spots (points)
within it. Cars will start at a random parking spot and choose to drive to another
random parking spot in a parking lot.

#### Traffic controls

There 2 types of traffic controls:

1. Stop signs: stop signs are annotated with a direction and are only enforced in that direction
1. Traffic lights: affect all incoming road segments. There is a timer on the light. The timer
will advance through each incoming road segment so that it is only green for "timer" 
seconds for each incoming segment sequentially.

### Navigation / Car Control

The car control is pretty simple. Each time the car reaches a destination, it charts
a new path to a new destination (using A*; code in [src/plan/plan.cc](src/plan/plan.cc)). 
The simulator estimates the speed along the road segments, so if a segment is moving
 slowly, the car should choose another path. Once the path is found, the car simply 
follows the path (it doesn't  replan) until it reaches its destination.

#### Collision avoidance

Cars try to accelerate until they reach either their max speed or the max speed of 
the roadway. The car will decelerate in one of the following conditions:

* If a hard corner is coming up. Cars have a maximum speed they can go through a
right-angle corner. They will start to decelerate to allow enough time to slow down.
* If there is an upcoming intersection that is not green
* If there are some cars ahead

There is no real collision detection: each car simply tries to decelerate to meet 
the above conditions. Sometimes it doesn't work out and cars will overlap.

#### Path following

To achieve some sort of realism in the paths that are not straight lines, and to avoid 
having to use a finely tesselated curve for the roads, all roads are interpretted
as being smooth (if they are long enough). This is done using an implied
 quadratic bezier curve.

The path following interpolation adjusts the road segment points to apply the bezier 
curve. Distance of the smoothed curve is approximated using a simple integral.


## Level editing

The levels can currently be edited in blender. See the examples in the [levels](levels) directory

![blender-level](media/blender-level.png)

Parking lots are simple 2D rectangular planes (centered on the origin). "Empty"s parented 
to the planes will be interpreted as parking spots.

Roads are "path" objects in blender. They can be annotated with a custom "speed" entry
to control the speed limit.

Traffic controls are also "empty" objects without a parent. The name of the empty, if
prefixed with either "Stop" or "Lights" will be interpretted as the respective traffic control. 
Stop signs are aligned with the y-axis of the empty. Traffic lights have a custom 
property "timer" that dictates the frequency of the update.

A rendering of the above level (in the simulator) looks like the following:

![](media/full_level_lights.png)

where the annotations on the ground indicate the approximate speed of the segments.





