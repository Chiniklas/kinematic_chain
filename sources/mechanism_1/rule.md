You are helping me build a 2D kinematic topology model of a single-DOF finger exoskeleton linkage using Python and matplotlib.

The mechanism is planar. The power transmission direction is from right to left. The rigid bodies are ordered as:
Base, link 1, link 2, link 3.

Use only simple polygons, bars, circular revolute joints, labels, and arrows. Do not create a 3D CAD model.

Topology:
1. Main serial chain:
   Base -- R01 -- link 1 -- R12 -- link 2 -- R23 -- link 3

2. Passive rod for the first closed loop:
   P0 -- B2
   - P0 is a fixed revolute joint on Base.
   - B2 is a revolute joint fixed on link 2.
   - The rod P0-B2 is an independent passive coupler link with constant length.
   - This loop transfers the motion from link 1 to link 2.

3. Passive rod for the second closed loop:
   A1 -- C3
   - A1 is a revolute joint fixed on link 1.
   - C3 is a revolute joint fixed on link 3.
   - The rod A1-C3 is an independent passive coupler link with constant length.
   - This rod is behind link 2 in the CAD, but it must not be connected to link 2.
   - This loop transfers the motion from link 1/link 2 to link 3.

Rigid-point memberships:
- Base contains P0 and R01.
- Link 1 contains R01, R12, and A1.
- Link 2 contains R12, R23, and B2.
- Link 3 contains R23 and C3.

Revolute joints:
- R01 connects Base and link 1.
- R12 connects link 1 and link 2.
- R23 connects link 2 and link 3.
- P0 connects Base and rod P0-B2.
- B2 connects link 2 and rod P0-B2.
- A1 connects link 1 and rod A1-C3.
- C3 connects link 3 and rod A1-C3.

Mobility:
There are 6 links including ground:
Base, link 1, link 2, link 3, rod P0-B2, rod A1-C3.
There are 7 revolute joints.
The planar mobility is:
M = 3 * (6 - 1) - 2 * 7 = 1.
So the mechanism should be modeled as one degree of freedom.

Task:
Write a Python script using matplotlib that:
1. Defines nominal 2D coordinates for all joints:
   P0, R01, R12, R23, B2, A1, C3, and the distal tip T3.
2. Draws Base as a fixed polygon on the right.
3. Draws link 1, link 2, and link 3 as colored polygons.
4. Draws P0-B2 and A1-C3 as passive rods.
5. Draws all revolute joints as circular markers.
6. Labels every joint and every rigid body.
7. Draws dashed internal rigid membership lines:
   A1-R12 belongs to link 1.
   B2-R12 belongs to link 2.
   C3-R23 belongs to link 3.
8. Draws an arrow showing transmission direction:
   Base -> 1 -> 2 -> 3, right to left.
9. Optionally implements a simple one-DOF animation:
   - input angle q drives link 1 about R01;
   - solve link 2 angle using the constraint |P0-B2| = constant;
   - solve link 3 angle using the constraint |A1-C3| = constant;
   - update the plot for q over a small range.

Use clean code structure:
- define a function transform_point(origin, theta, local_point)
- define a function draw_link_polygon(...)
- define a function circle_intersection(...) for solving four-bar closure
- define a function draw_mechanism(q)
- use matplotlib.animation.FuncAnimation for the optional animation.

Important:
A1-C3 must not be connected to link 2. It is an independent floating passive rod behind link 2.
P0-B2 and A1-C3 are passive coupler links, not motors.
Only R01 is the active input.