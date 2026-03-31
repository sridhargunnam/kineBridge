# Module 1: The Basics — 2D Planar Arms

## 1.1 What is Forward Kinematics?

Forward Kinematics (FK) answers the question:

> **Given the joint angles of a robot arm, where is the end-effector in Cartesian space?**

This is the *forward* direction: **joint space → task space** (also called Cartesian space or operational space).

For a planar (2D) robot, "task space" is simply the `(x, y)` position of the tip of the arm.


## 1.2 The 1-Link Planar Arm

The simplest possible robot: a single rigid link of length `L`, attached to a fixed base by a revolute (hinge) joint.

```
         (end-effector)
        *
       /
      / L
     /
    /  θ
   *-------→ x-axis
 (base)
```

The joint angle `θ` (theta) is measured counter-clockwise from the positive x-axis.

### FK Derivation

The end-effector position is simply a point on a circle of radius `L`:

```
x = L · cos(θ)
y = L · sin(θ)
```

This comes directly from the **unit circle** definition of sine and cosine.

### Key Insight

The FK of a 1-link arm is a **mapping** from a single scalar `θ` to a 2D point `(x, y)`. The set of all reachable points forms a circle of radius `L` centered at the base — this is the **workspace** of the 1-link arm.


## 1.3 The 2-Link Planar Arm

Now chain two links together: link 1 has length `L1`, link 2 has length `L2`.

```
              (end-effector)
             *
            /
           / L2
          /
  (joint 1)
        *
       /
      / L1
     /
    /  θ1
   *-------→ x-axis
 (base)
```

### FK Derivation

The key concept is **cumulative angles**. Joint 2's angle `θ2` is measured *relative to link 1*, but to find the absolute orientation of link 2 in the world frame, we need:

```
absolute angle of link 2 = θ1 + θ2
```

Now we can compute positions step by step:

**Joint 1 position (elbow):**
```
x1 = L1 · cos(θ1)
y1 = L1 · sin(θ1)
```

**End-effector position:**
```
x2 = x1 + L2 · cos(θ1 + θ2)
y2 = y1 + L2 · sin(θ1 + θ2)
```

Or expanded:
```
x_ee = L1 · cos(θ1) + L2 · cos(θ1 + θ2)
y_ee = L1 · sin(θ1) + L2 · sin(θ1 + θ2)
```

### Why "cumulative"?

Each joint rotates *everything downstream*. When joint 1 rotates by `θ1`, it carries link 2 (and joint 2, and the end-effector) along with it. This is the essence of a **kinematic chain** — and it's exactly how MuJoCo's nested `<body>` tags work (more on that in Section 1.6).

### The Workspace

The 2-link arm's workspace is an **annulus** (ring):
- **Outer radius** = `L1 + L2` (arm fully extended)
- **Inner radius** = `|L1 - L2|` (arm fully folded)
- If `L1 == L2`, the inner radius is 0 and the arm can reach the base.


## 1.4 Inverse Kinematics for the 2-Link Arm

Inverse Kinematics (IK) is the reverse problem:

> **Given a desired end-effector position `(x, y)`, what joint angles `(θ1, θ2)` achieve it?**

For a 2-link planar arm, there is a **closed-form (analytic) solution** using the law of cosines.

### Step 1: Solve for θ2 using the Law of Cosines

The distance from base to target is:
```
d² = x² + y²
```

By the law of cosines applied to the triangle formed by L1, L2, and d:
```
d² = L1² + L2² - 2·L1·L2·cos(π - θ2)
```

Since `cos(π - θ2) = -cos(θ2)`:
```
d² = L1² + L2² + 2·L1·L2·cos(θ2)
```

Solving for `cos(θ2)`:
```
cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)
```

Then:
```
θ2 = ±arccos(cos_θ2)
```

The **two solutions** correspond to:
- `+arccos` → **elbow down** configuration
- `-arccos` → **elbow up** configuration

### Step 2: Solve for θ1 using atan2

Once we know `θ2`, we find `θ1`:
```
θ1 = atan2(y, x) - atan2(k2, k1)
```

where:
```
k1 = L1 + L2·cos(θ2)
k2 = L2·sin(θ2)
```

### Reachability Check

Before computing, verify the target is reachable:
```
|L1 - L2| ≤ sqrt(x² + y²) ≤ L1 + L2
```

If not, the target is outside the workspace.

### Why Two Solutions?

This is a fundamental concept in robotics: IK is generally **not unique**. Even this simple 2-link arm has two valid solutions for most targets. As we add more DOF (degrees of freedom), the number of solutions can become infinite — this is called **redundancy** and is a major topic in Module 3.


## 1.5 Coordinate Frames and the World Frame

Before we model anything in MuJoCo, we need to establish coordinate conventions:

- The **world frame** is the fixed, global reference frame. In MuJoCo, this is defined by `<worldbody>`.
- Each body in the kinematic tree has its own **local frame**. Positions and orientations specified in a `<body>` tag are *relative to its parent*.
- Joint axes are defined in the body's local frame.

For our 2D arms, we work in the **xy-plane** with z pointing up. Hinge joints rotate about the **z-axis**.


## 1.6 MJCF Modeling Basics

MuJoCo uses the **MJCF** (MuJoCo Format) XML format to define robots.

### The Kinematic Tree

MJCF represents the robot as a tree of nested `<body>` elements:

```xml
<worldbody>
    <body name="link1" pos="0 0 0">         <!-- attached to world -->
        <joint .../>                         <!-- joint connecting to parent -->
        <geom .../>                          <!-- visual/collision geometry -->
        <body name="link2" pos="L1 0 0">    <!-- child: attached to link1 -->
            <joint .../>
            <geom .../>
        </body>
    </body>
</worldbody>
```

### Key Tags

| Tag | Purpose | Key Attributes |
|-----|---------|----------------|
| `<body>` | A rigid body in the kinematic tree | `name`, `pos` (position relative to parent) |
| `<joint>` | Connects a body to its parent with a DOF | `name`, `type` (hinge, slide, ball), `axis` |
| `<geom>` | Shape for visualization and/or collision | `type` (capsule, box, sphere), `size`, `fromto` |
| `<site>` | A point marker attached to a body (no physics) | `name`, `pos` — useful for reading end-effector position |
| `<actuator>` | Defines how joints are controlled | `<motor joint="..."/>` |

### The `pos` Attribute: Relative Positioning

This is critical: `pos` in a `<body>` is **relative to the parent body's frame**. So if link1 starts at the origin and has length `L1 = 1.0`:

```xml
<body name="link1" pos="0 0 0">
    ...
    <body name="link2" pos="1.0 0 0">   <!-- offset by L1 along x -->
        ...
    </body>
</body>
```

This nesting mirrors the mathematical FK chain: each body inherits its parent's transformations, just like how `θ1 + θ2` accumulates rotations.

### Hinge Joints for 2D Arms

For planar (2D) arms in the xy-plane, we use hinge joints rotating about the z-axis:

```xml
<joint name="joint1" type="hinge" axis="0 0 1"/>
```

The `axis="0 0 1"` means the joint rotates about z, which keeps motion in the xy-plane.

### Geom as Capsules

A convenient way to visualize links:

```xml
<geom type="capsule" fromto="0 0 0  1.0 0 0" size="0.04"/>
```

This draws a capsule (cylinder with rounded ends) from the body origin to the point `(1.0, 0, 0)` with radius `0.04`. The `fromto` attribute defines the two endpoints.

### Sites for Tracking Positions

To read the end-effector position in Python, attach a `<site>` to the last body:

```xml
<site name="end_effector" pos="1.0 0 0" size="0.02"/>
```

In Python: `data.site_xpos[model.site('end_effector').id]` gives the world-frame position.

### Actuators

To control joints from Python, define actuators:

```xml
<actuator>
    <motor joint="joint1" ctrllimited="true" ctrlrange="-3.14 3.14"/>
</actuator>
```

Then in Python: `data.ctrl[0] = desired_torque` or use position control with `<position joint="..."/>`.


## 1.7 Connecting FK Math to MuJoCo

The entire point of this module is to verify that **your hand-written FK math matches MuJoCo's simulation**:

1. Write FK functions in pure numpy.
2. Build the same arm in MJCF XML.
3. Set `data.qpos` to specific joint angles.
4. Call `mujoco.mj_forward(model, data)` to compute MuJoCo's FK.
5. Read `data.site_xpos` and compare against your numpy FK.

They should match to floating-point precision. If they don't, either your math or your XML is wrong — and debugging that mismatch is one of the best ways to deeply learn both FK and MJCF.


## 1.8 Summary of Equations

### 1-Link FK
```
x = L · cos(θ)
y = L · sin(θ)
```

### 2-Link FK
```
x_elbow = L1 · cos(θ1)
y_elbow = L1 · sin(θ1)
x_ee    = L1 · cos(θ1) + L2 · cos(θ1 + θ2)
y_ee    = L1 · sin(θ1) + L2 · sin(θ1 + θ2)
```

### 2-Link Analytic IK
```
cos_θ2 = (x² + y² - L1² - L2²) / (2·L1·L2)
θ2     = ±arccos(cos_θ2)
k1     = L1 + L2·cos(θ2)
k2     = L2·sin(θ2)
θ1     = atan2(y, x) - atan2(k2, k1)
```
