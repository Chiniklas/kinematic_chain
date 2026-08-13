# Task-space reachability objective

This is the only active optimization objective. It asks whether mechanism TCP `H`,
connected by optimizable rod `L_tip_rod`, can reach the lower-surface distal slot in
the horizontal and maximum-curled hand poses for every long finger.

## Coordinate frame and hand slot

The origin is mechanism mount `D`. Positive x is distal along horizontal `A–D`, and
positive y is dorsal. Hand joint `J1` is at `(0, -c)`, where `c` is the fixed upstream
`dorsal_clearance_mm` parameter.

Let phalanx lengths be `l1`, `l2`, and `l3`, distal width be `w3`, and MCP/PIP/DIP
flexion be `a1`, `a2`, and `a3`. Define the absolute link headings

```text
theta1 = -a1
theta2 = -(a1 + a2)
theta3 = -(a1 + a2 + a3).
```

The DIP and tip locations relative to J1 are

```text
P_DIP = l1 u(theta1) + l2 u(theta2)
P_tip = P_DIP + l3 u(theta3),
u(theta) = [cos(theta), sin(theta)].
```

The distal dorsal normal is `n3 = [-sin(theta3), cos(theta3)]`. The pin-in-slot path on
the lower/palmar surface, expressed in the D frame, is

```text
S(t) = P_DIP - w3 n3 + t u(theta3) + [0, -c],
0 <= t <= l3.
```

Thus the translational range always equals the corresponding finger's distal length.
The pin may rotate freely at every valid `t`. The horizontal slot uses zero flexion;
the curled slot uses the maximum MCP/PIP/DIP angles in each finger objective YAML.

## Mechanism, rod, and slot residual

For candidate design vector `x`, the analytic circle-intersection solver tracks the
nominal assembly branch and returns TCP position `H(q; x)`. With rod length `lr`, the
closure error at a mechanism pose is

```text
r(q, S; x) = min_(0 <= t <= l3) abs(||H(q; x) - S(t)||2 - lr).
```

This error is zero whenever the circle centered at H with radius `lr` intersects the
finite lower-surface slot. Computationally, distances from H to the slot form an
interval `[d_min, d_max]`, so

```text
r = d_min - lr,  if lr < d_min
r = 0,           if d_min <= lr <= d_max
r = lr - d_max,  if lr > d_max.
```

The normalized squared error is

```text
z(q, S; x) = (r(q, S; x) / 5 mm)^2.
```

Horizontal reachability is assigned to `q = 0°`. Curled reachability uses a smooth
minimum over the sampled `0–90°` actuator range:

```text
L_horizontal = z(0°, S_horizontal; x)

L_curled = -tau log(mean_q(exp(-z(q, S_curled; x) / tau))),
tau = 0.02.
```

Per finger and across all fingers,

```text
L_finger = 0.5 (L_horizontal + L_curled)
L_total = 1/4 sum_f L_finger.
```

If the linkage cannot assemble at the start of its branch, a large reachability loss
is returned. No separate feasibility penalty is active.

## Deliberate limitations

The objective checks reachability of the complete distal slot, not one fixed distal
joint. The measured MCP/PIP/DIP angles define the slot pose, but a reachable pin
location still does not guarantee that the passive human joints follow those angles
under load. Energy, torque, path tracking, collision, singularity, joint limits,
smoothness, compactness, and regularization remain disabled.

`dorsal_clearance_mm` and distal width are fixed upstream design inputs. The slot
translation and rod length participate in reachability; only rod length is currently
an Adam design variable.
