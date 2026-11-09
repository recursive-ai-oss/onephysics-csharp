using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>A shape's frame in world space: the origin the shape's local coordinates are relative to, and its rotation. A planar world uses z = 0 and a rotation about z.</summary>
internal struct FluidFrame
{
    public double X, Y, Z;
    public float Qx, Qy, Qz, Qw;
}

/// <summary>A rigid body as the fluid step sees it: the pose and velocity it starts the step with and the virtual pose it has been given since by the fluid's own pushes and by gravity. Planar bodies are embedded with z = 0 and a spin about z.</summary>
internal struct FluidBody
{
    public double X, Y, Z;          // centre of mass
    public float Qx, Qy, Qz, Qw;
    public float Vx, Vy, Vz, Wx, Wy, Wz;
    public float InvMass;
    public float Iw0, Iw1, Iw2, Iw3, Iw4, Iw5;   // world inverse inertia (xx xy xz yy yz zz)
    public float Gx, Gy, Gz;        // gravity acceleration the body will feel
    public bool Dynamic;            // takes impulses and moves by itself (awake, finite mass)
    public bool Moves;              // kinematic or dynamic: its pose advances during the step
    public bool Asleep;             // dynamic but asleep: acts as a static surface unless pushed hard
    // Accumulated by the fluid step (serial): the change of momentum the fluid has given the body.
    public float Jx, Jy, Jz, Lx, Ly, Lz;          // this substep
    public float Tx, Ty, Tz, Ux, Uy, Uz;          // the whole step (momentum and angular momentum)
    public float WetArea;                          // contacts of the step, times the area each stands for
    public int Body;                               // slot in the world's body store
    public double StartX, StartY, StartZ;          // the pose the step found the body in
    public float StartQx, StartQy, StartQz, StartQw;
}

/// <summary>What a sphere against a shape found: the normal (shape to sphere), the separation of the surfaces (negative when overlapping) and the point on the shape.</summary>
internal struct FluidHit
{
    public float Nx, Ny, Nz;
    public float Separation;
    public double Px, Py, Pz;
    public float Friction;
}

/// <summary>What the fluid store needs from an engine. The planar engine presents everything with z = 0.</summary>
internal interface IFluidHost
{
    WorldCore Core { get; }
    bool Planar { get; }
    int EngineSectionId { get; }
    FluidStore ParticleFluids { get; }
    Vector3F Gravity { get; }
    ReplayMutationScope FluidMutation(string operation);

    /// <summary>The live shapes whose broadphase boxes meet the box, ascending by shape slot. Includes sensors; the filter is <see cref="ShapeCollides"/>.</summary>
    void GatherShapes(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, List<int> shapes);

    /// <summary>The same rule as shape pairs: sensors never collide, then the layer matrix, then group, then category and mask.</summary>
    bool ShapeCollides(int shape, int layer, int group, uint category, uint mask);
    int OwnerOf(int shape);

    /// <summary>Fills the body's pose, velocity, mass and gravity. False when the body cannot take part (dead).</summary>
    void LoadBody(int body, out FluidBody b);

    /// <summary>The frame of the shape when its body is at the given pose (centre of mass and rotation).</summary>
    void ShapeFrame(int shape, in FluidBody pose, out FluidFrame frame);

    /// <summary>The shape's box at the frame, in world coordinates.</summary>
    void ShapeBox(int shape, in FluidFrame frame, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ);

    /// <summary>A sphere of the radius centred at the world point against the shape at its frame. False when farther than <paramref name="speculative"/> apart. Safe to call from several threads.</summary>
    bool CollideSphere(int shape, in FluidFrame frame, double cx, double cy, double cz, float radius, float speculative, out FluidHit hit);

    /// <summary>The first contact of a sphere moving by (mx, my, mz) from the point against the shape at its frame, with the sphere's centre at the contact in <paramref name="hit"/>.Px.. . False when there is none or it starts overlapping. Safe to call from several threads.</summary>
    bool SweepSphere(int shape, in FluidFrame frame, double cx, double cy, double cz, float mx, float my, float mz, float radius, out FluidHit hit);

    /// <summary>Adds the fluid's change of momentum to the body's velocity (waking it if it is asleep and the change exceeds the wake speed; smaller changes of a sleeping body are dropped).</summary>
    void ApplyBodyImpulse(int body, float dvx, float dvy, float dvz, float dwx, float dwy, float dwz, float wakeSpeed);

    /// <summary>Whether the body currently exists with the generation.</summary>
    bool BodyAlive(int body, int generation);

    /// <summary>Sample points that estimate submersion: points of a lattice inside the body's shapes, in the body frame relative to the centre of mass, each with its depth (how far it is from the nearest surface of its shape). Each point stands for an equal share of the body's volume. Returns the count.</summary>
    int SamplePoints(int body, Span<Vector3F> points, Span<float> depths);

    /// <summary>Rigid bodies the broadphase knows, as an upper bound for sizing.</summary>
    int BodyHighWater { get; }

    /// <summary>Soft-body and rod particles near the box that particles may touch, appended in a fixed order.</summary>
    void GatherDeformables(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, FluidDeformables into);

    /// <summary>Pushes the accumulated velocity changes back to the deformables (soft particles and rod particles).</summary>
    void ApplyDeformables(FluidDeformables from);
}
