# Ideal fixed-contact objective mathematics

There are four independent optimization problems, one for each long finger. Every
problem starts from the same nominal Mechanism 2 dimensions but owns its own design
vector and Adam state.

## Ideal mechanism–hand interface

The current interface deliberately ignores cuff compliance, skin motion, attachment
pressure, backlash, and out-of-plane motion. The distal phalanx is a rigid body. `R4`
is an exact revolute joint fixed at the longitudinal midpoint of its upper surface,
and `H–R4` is an ideal rigid two-force rod.

The crank angle `q` is the sole prescribed motion coordinate. The ideal compliant
finger has one passive curl coordinate `s in [0,1]`, constrained to the measured
horizontal-to-maximum-curl path:

```text
a_fj(s) = s a_fj,max
theta_f1(s) = -a_f1(s)
theta_f2(s) = -(a_f1(s) + a_f2(s))
theta_f3(s) = -(a_f1(s) + a_f2(s) + a_f3(s)).
```

With `u(theta)=[cos(theta),sin(theta)]` and fixed dorsal mount clearance `c`,

```text
P_DIP,f(s) = l_f1 u(theta_f1) + l_f2 u(theta_f2) + [0,-c]
P_tip,f(s) = P_DIP,f(s) + l_f3 u(theta_f3)
R4_f(s) = 0.5 (P_DIP,f(s) + P_tip,f(s)).
```

There is no slider coordinate and no contact-point travel objective.

This first idealization has zero hand stiffness and only one finger synergy DOF; it
does not yet predict force. For every increasing crank sample, closure selects the
nearest nondecreasing passive root `s_k`. The hard admissible-motion rule is

```text
s_0 = 0,   0 <= s_k <= 1,   s_(k+1) >= s_k,
s_(K-1) >= 1 - 0.01.
```

Because all measured flexion maxima are nonnegative and every joint angle is scaled
by the same monotone `s`, no joint can bend dorsally or reverse midway through curl.
There is no prescribed `s(q)` law and no hand/crank ratio.

Each design vector contains 11 mechanism lengths and `L_tip_rod`. The horizontal
ground baseline `L_ad=54 mm`, finger geometry, palm geometry, and `D–J1` clearance are
fixed inputs.

## Hard constraint 1: intended-workspace rod closure

For mechanism TCP `H(q;x)`, fixed-contact closure residual is

```text
r_f(q,s;x) = abs(||H(q;x)-R4_f(s)|| - L_tip_rod).
```

A sampled static search selects a terminal crank angle where maximum curl can close:

```text
q_f,* = first forward root of r_f(q,1;x)=0,
```

or uses the minimum-residual sample when no root exists. The commanded crank samples
are `K=31` increasing values from `0` to `q_f,*`. At each fixed `q_k`, the passive
coordinate is the nearest nondecreasing root of

```text
r_f(q_k,s_k;x)=0.
```

Fallback residuals remain visible to the hard closure test; the solver cannot make an
incompatible design feasible by silently moving the contact or reversing the finger.

Adam receives differentiable feasibility guidance

```text
J_closure-guide,f = (1/K) sum_k (r_f,k / 5 mm)^2,
```

but candidate acceptance uses the hard rule

```text
max_k r_f,k <= 0.1 mm.
```

Both endpoints are included, so a low mean cannot hide a disconnected pose.

## Hard constraint 2: hand–mechanism non-collision

Mechanism edges are represented as 2 mm-radius capsules, the output rod as a
1.5 mm-radius capsule, and rigid ternary bodies as expanded polygons. Palm and
phalanges use their rounded-rectangle geometry. Only the intentional `D–J1` mount and
the 4 mm neighborhood around fixed `R4` are excluded.

For signed primitive clearance `d_f,k,p`, positive means separated and negative means
penetration. The hard sampled rule is

```text
min_(k,p) d_f,k,p >= 0.
```

Adam is guided toward the configured `c_safe=2 mm` clearance through a smooth hinge:

```text
h_sigma(z) = sigma log(1+exp(z/sigma)), sigma=0.5 mm
J_clearance-guide,f = SmoothMax_(k,p)
    (h_sigma(c_safe-d_f,k,p)/c_safe)^2.
```

Collision is evaluated at all 31 poses but is not yet continuously certified between
samples.

## Sole design objective: output-rod perpendicularity

Let

```text
v_f,k = (H(q_k;x)-R4_f(s_k)) / ||H(q_k;x)-R4_f(s_k)||
t_f,k = u(theta_f3(s_k)).
```

The ideal rod is perpendicular to the distal surface when its tangential component is
zero:

```text
e_perpendicular,f,k = (v_f,k dot t_f,k)^2
J_perpendicular,f = SmoothMax_k e_perpendicular,f,k.
```

The loss is orientation-independent, equals zero for a normal rod, and equals one for
a rod parallel to the distal surface. Reports also show

```text
delta_f,k = asin(clamp(abs(v_f,k dot t_f,k),0,1)),
```

the angular deviation from the surface normal.

## Adam training loss and acceptance

Perpendicularity is the only design-quality objective. Closure, hand-motion
admissibility, and non-collision are hard constraints. Closure and collision retain
smooth guidance terms because a Boolean rejection has no useful finite-difference
gradient:

```text
L_Adam,f = (1 J_closure-guide,f
            + 5 J_clearance-guide,f
            + 1 J_perpendicular,f) / 7.
```

These coefficients do not turn feasibility terms into product objectives. Candidate
ranking first counts passed hard constraints; only equally feasible candidates are
compared by training loss. A promotable candidate must curl monotonically downward to
the measured maximum, close the fixed `H–R4` rod, and avoid unintended hand penetration
at every sampled pose.

Force-transmission Jacobians, interface pressure, compliance, actuator limits,
singularity margins, and a multi-DOF stiffness/equilibrium hand model are intentionally
deferred.
