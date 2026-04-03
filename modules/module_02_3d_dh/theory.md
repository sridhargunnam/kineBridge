# Module 2: Expanding to 3D & Joint Types

## 2.1 Why 3D Changes Everything

In Module 1, our planar arms lived in the xy-plane. A single angle fully described a joint's rotation because there was only one possible rotation axis (z). In 3D:

- A body's **pose** requires 6 numbers: 3 for position `(x, y, z)` and 3 for orientation (roll, pitch, yaw — or a rotation matrix).
- Joints can rotate about **any axis**, not just z.
- We need a systematic way to describe how coordinate frames are attached to each link and how they relate to their neighbors.

This is where **homogeneous transformation matrices** and the **Denavit-Hartenberg (DH) convention** come in.


## 2.2 Homogeneous Transformation Matrices (4x4)

A 3D rigid-body transformation combines rotation and translation. We encode both in a single 4x4 matrix:

```
T = | R   p |     R = 3x3 rotation matrix
    | 0   1 |     p = 3x1 translation vector
```

Explicitly:

```
T = | r11  r12  r13  px |
    | r21  r22  r23  py |
    | r31  r32  r33  pz |
    |   0    0    0   1 |
```

### Why 4x4?

A 3x3 rotation matrix can only rotate — it cannot translate. By embedding rotation and translation into a 4x4 matrix operating on **homogeneous coordinates** `[x, y, z, 1]^T`, we can do both in one matrix multiplication:

```
[x']     [x]
[y'] = T [y]
[z']     [z]
[ 1]     [1]
```

### Chaining Transforms

The power of homogeneous matrices is that **chaining** them is just matrix multiplication:

```
T_0^n = T_0^1 · T_1^2 · T_2^3 · ... · T_{n-1}^n
```

`T_0^n` gives the position and orientation of frame `n` expressed in frame `0` (the world/base frame). This is exactly FK for a serial manipulator.

### Elementary Rotation Matrices

Rotation about the **x-axis** by angle θ:
```
Rx(θ) = | 1    0       0   |
        | 0   cos(θ) -sin(θ)|
        | 0   sin(θ)  cos(θ)|
```

Rotation about the **y-axis** by angle θ:
```
Ry(θ) = |  cos(θ)  0  sin(θ)|
        |    0     1    0   |
        | -sin(θ)  0  cos(θ)|
```

Rotation about the **z-axis** by angle θ:
```
Rz(θ) = | cos(θ) -sin(θ)  0 |
        | sin(θ)  cos(θ)  0 |
        |   0       0     1 |
```

### Building a Transform

To create a homogeneous transform that rotates about z by θ and then translates by `[a, 0, 0]`:

```
T = | cos(θ) -sin(θ)  0  a·cos(θ) |
    | sin(θ)  cos(θ)  0  a·sin(θ) |
    |   0       0     1      0     |
    |   0       0     0      1     |
```

This is the core building block of DH transforms.


## 2.3 The Denavit-Hartenberg (DH) Convention

DH is a **systematic convention** for placing coordinate frames on each link of a serial manipulator. It reduces the description of each link-joint pair to exactly **4 parameters**, regardless of the robot's complexity.

### The 4 DH Parameters

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Joint angle | θ (theta) | Rotation about the z-axis of frame i-1 |
| Link offset | d | Translation along the z-axis of frame i-1 |
| Link length | a | Translation along the x-axis of frame i |
| Link twist | α (alpha) | Rotation about the x-axis of frame i |

### Which parameter is the joint variable?

- For a **revolute** (hinge) joint: **θ is the variable**, and d, a, α are constants.
- For a **prismatic** (slide) joint: **d is the variable**, and θ, a, α are constants.

### The DH Transformation Matrix

Each link's transform from frame i-1 to frame i is computed as:

```
T_{i-1}^{i} = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
```

Multiplied out, this gives:

```
T = | cos(θ)  -sin(θ)·cos(α)   sin(θ)·sin(α)   a·cos(θ) |
    | sin(θ)   cos(θ)·cos(α)  -cos(θ)·sin(α)   a·sin(θ) |
    |   0         sin(α)          cos(α)            d      |
    |   0           0               0               1      |
```

This is the **standard DH** convention (Hartenberg & Denavit, 1955). There is also a "modified DH" (Craig) convention with a slightly different ordering — we use the standard convention in this module.

### The DH Table

For any serial robot, you fill out a table:

| Link i | θ_i | d_i | a_i | α_i |
|--------|-----|-----|-----|-----|
| 1 | θ1 | 0 | L1 | 0 |
| 2 | θ2 | 0 | L2 | 0 |
| 3 | θ3 | 0 | L3 | 0 |

Then FK is simply: `T_0^3 = T_0^1(θ1) · T_1^2(θ2) · T_2^3(θ3)`

The end-effector position is the translation part of the final product: `T_0^3[:3, 3]`.

### Example: 2-Link Planar Arm in DH

Our Module 1 arm revisited with DH:

| Link | θ | d | a | α |
|------|---|---|---|---|
| 1 | θ1 | 0 | 1.0 | 0 |
| 2 | θ2 | 0 | 1.0 | 0 |

All α = 0 (no twist) and all d = 0 (no offset) because both joints rotate about z and all links extend along x in the same plane. This produces the same FK equations we derived with trig in Module 1 — DH is a generalization.


## 2.4 Joint Types in 3D

### Revolute (Hinge) Joint

Allows rotation about a single axis. 1 DOF.
- In DH: **θ is the variable**
- In MJCF: `<joint type="hinge" axis="0 0 1"/>`
- Axis can be any unit vector, not just z

### Prismatic (Slide) Joint

Allows linear translation along a single axis. 1 DOF.
- In DH: **d is the variable**
- In MJCF: `<joint type="slide" axis="0 0 1"/>`
- Think of a telescoping link or a linear actuator

### Ball (Spherical) Joint

Allows rotation about all three axes simultaneously. 3 DOF.
- Cannot be directly represented by a single DH row
- Typically modeled as 3 consecutive revolute joints with zero link length
- In MJCF: `<joint type="ball"/>` — a single tag gives 3 rotational DOF
- Quaternion representation: `data.qpos` uses 4 numbers (w, x, y, z) for a ball joint

### Spherical Wrist

A common configuration at the end of industrial arms: 3 revolute joints whose axes intersect at a single point. This creates a "wrist" that can achieve any orientation independently of the arm's position — a property called **kinematic decoupling** (crucial for analytical IK in Module 3+).

In DH, a spherical wrist is three consecutive revolute joints with `a = 0` (zero link length), alternating α values of `±π/2`:

| Link | θ | d | a | α |
|------|---|---|---|---|
| 4 | θ4 | 0 | 0 | -π/2 |
| 5 | θ5 | 0 | 0 | π/2 |
| 6 | θ6 | 0 | 0 | 0 |


## 2.5 Joint Limits

Real robots have physical limits on how far joints can move. In DH/FK, these are constraints on the variable parameter:

- Revolute: `θ_min ≤ θ ≤ θ_max` (e.g., -π to π, or -2.96 to 2.96 rad)
- Prismatic: `d_min ≤ d ≤ d_max` (e.g., 0 to 0.5 meters)

In MJCF, use the `range` attribute:

```xml
<joint name="j1" type="hinge" axis="0 0 1" range="-170 170"/>
```

The `range` is in **degrees** for hinge joints and **meters** for slide joints. MuJoCo also needs `limited="true"` or a global `<compiler angle="radian"/>` setting.


## 2.6 MJCF Modeling in 3D

### Moving Beyond the XY-Plane

In Module 1, all joints had `axis="0 0 1"` (z-axis), keeping motion in the xy-plane. In 3D:

- Joints can rotate about **any axis**: `axis="1 0 0"` (x), `axis="0 1 0"` (y), or arbitrary `axis="0.707 0.707 0"`.
- Body positions use all three components: `pos="0 0 0.5"` offsets along z.
- The kinematic tree (nested `<body>` tags) is the same concept — child frames are positioned relative to parents.

### 3D Geom Types

Beyond capsules:
- `<geom type="box" size="0.1 0.05 0.05"/>` — half-sizes along x, y, z
- `<geom type="cylinder" size="0.04 0.25"/>` — radius and half-height
- `<geom type="sphere" size="0.05"/>` — radius

### Euler Angles for Body Orientation

To orient a body's local frame relative to its parent, use `euler`:

```xml
<body name="link2" pos="0 0 0.5" euler="0 90 0">
```

This rotates the body frame 90° about y before positioning it. MuJoCo defaults to XYZ intrinsic Euler angles (in degrees, unless `<compiler angle="radian"/>` is set).

### The `<compiler>` Tag

Useful settings for 3D arms:

```xml
<compiler angle="degree"/>  <!-- or "radian" -->
```

This controls how `range`, `euler`, and other angle values are interpreted.


## 2.7 From 2D FK to 3D FK via DH

The workflow for computing FK of a 3D arm:

1. **Assign DH frames** to each link following the convention
2. **Fill the DH table** with θ, d, a, α for each link
3. **Compute each T_{i-1}^{i}** using the DH formula
4. **Multiply the chain**: `T_0^n = T_0^1 · T_1^2 · ... · T_{n-1}^n`
5. **Extract the result**: position = `T_0^n[:3, 3]`, orientation = `T_0^n[:3, :3]`

Verify against MuJoCo by setting `data.qpos` and comparing `data.site_xpos`.


## 2.8 Summary of Key Equations

### Homogeneous Transform (from R and p)
```
T = | R  p |     where R is 3x3, p is 3x1
    | 0  1 |
```

### DH Transform (standard convention)
```
T(θ, d, a, α) = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) |
                 | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) |
                 |   0        sin(α)          cos(α)           d     |
                 |   0          0               0              1     |
```

### FK via DH Chain
```
T_0^n = T_0^1 · T_1^2 · ... · T_{n-1}^n
position    = T_0^n[:3, 3]
orientation = T_0^n[:3, :3]
```
