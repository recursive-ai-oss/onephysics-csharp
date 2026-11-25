using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The geometry a soft body, cloth or rod has now, read back as plain arrays so a host can bake it into a model (a membrane's sag, a hanging chain, a drape). A snapshot of the moment it was read; it shares nothing with the world.
/// Meant for use after <see cref="PhysicsWorld2D.Settle"/>, when the shape is at rest, but valid at any time.
/// </summary>
public sealed class RestShape2D
{
    internal RestShape2D() { }

    /// <summary>Particle positions in world coordinates, metres, in particle order.</summary>
    public Vector2D[] Positions { get; internal set; } = [];
    /// <summary>Triangles as index triples into <see cref="Positions"/>: the triangles of a soft body. Empty for a rod and for a strip or body with no triangles.</summary>
    public int[] Triangles { get; internal set; } = [];
    /// <summary>Edges as index pairs into <see cref="Positions"/>: the constraint edges of a soft body, the segments of a rod.</summary>
    public int[] Edges { get; internal set; } = [];
    /// <summary>Segment angles of a rod in radians, counter-clockwise from the +X axis (the tangent direction). Empty for a soft body or cloth.</summary>
    public float[] Frames { get; internal set; } = [];
    /// <summary>Smallest corner of the axis-aligned bounds of the particles.</summary>
    public Vector2D BoundsMin { get; internal set; }
    /// <summary>Largest corner of the axis-aligned bounds of the particles.</summary>
    public Vector2D BoundsMax { get; internal set; }
    /// <summary>The unit up direction, opposite to the world's gravity; zero when the world has no gravity.</summary>
    public Vector2F Up { get; internal set; }
    /// <summary>How far the lowest particle hangs below the reference height, metres. The reference is the lowest pinned particle, or the highest particle when none is pinned. A chain pinned at both ends gives its sag; a cloth pinned at its corners gives its droop.</summary>
    public double Sag { get; internal set; }
    /// <summary>Sum of the segment lengths of a rod in metres; zero for a soft body or cloth.</summary>
    public double ArcLength { get; internal set; }
    /// <summary>The fastest particle speed at the time of reading, m/s: how far from rest the shape still is.</summary>
    public double MaxSpeed { get; internal set; }

    internal static RestShape2D FromSoft(PhysicsWorld2D world, SoftBody body)
    {
        int n = body.ParticleCount;
        var r = new RestShape2D { Positions = new Vector2D[n], Triangles = body.RawTriangles.ToArray() };
        body.CopyPositions(r.Positions);
        var ea = body.RawEdgeA; var eb = body.RawEdgeB;
        r.Edges = new int[2 * ea.Length];
        for (int i = 0; i < ea.Length; i++) { r.Edges[2 * i] = ea[i]; r.Edges[2 * i + 1] = eb[i]; }
        double top = double.NegativeInfinity, pinned = double.PositiveInfinity, fastest = 0;
        var up = UpOf(world);
        for (int i = 0; i < n; i++)
        {
            double h = Height(r.Positions[i], up);
            if (h > top) top = h;
            if (body.IsPinned(i) && h < pinned) pinned = h;
            var v = body.GetVelocity(i); double s = (double)v.X * v.X + (double)v.Y * v.Y; if (s > fastest) fastest = s;
        }
        r.Finish(up, double.IsPositiveInfinity(pinned) ? top : pinned, Math.Sqrt(fastest));
        return r;
    }

    internal static RestShape2D FromRod(PhysicsWorld2D world, Rod rod)
    {
        var data = rod.Particles;
        int n = rod.ParticleCount;
        var r = new RestShape2D { Positions = new Vector2D[n], Frames = new float[n - 1], Edges = new int[2 * (n - 1)] };
        for (int i = 0; i < n; i++) { var p = data.Positions[i]; r.Positions[i] = new Vector2D(p.X, p.Y); }
        for (int i = 0; i < n - 1; i++) { var q = data.Frames[i]; r.Frames[i] = (float)(2.0 * Math64.Atan2(q.Z, q.W)); }
        double top = double.NegativeInfinity, pinned = double.PositiveInfinity, fastest = 0, arc = 0;
        var up = UpOf(world);
        var velocities = data.Velocities;
        for (int i = 0; i < n; i++)
        {
            double h = Height(r.Positions[i], up);
            if (h > top) top = h;
            if (data.Pinned[i] && h < pinned) pinned = h;
            var v = velocities[i]; double s = (double)v.X * v.X + (double)v.Y * v.Y; if (s > fastest) fastest = s;
            if (i < n - 1)
            {
                r.Edges[2 * i] = i; r.Edges[2 * i + 1] = i + 1;
                var a = r.Positions[i]; var b = r.Positions[i + 1];
                double dx = b.X - a.X, dy = b.Y - a.Y; arc += Math.Sqrt(dx * dx + dy * dy);
            }
        }
        r.ArcLength = arc;
        r.Finish(up, double.IsPositiveInfinity(pinned) ? top : pinned, Math.Sqrt(fastest));
        return r;
    }

    private static (double X, double Y) UpOf(PhysicsWorld2D world)
    {
        var g = world.Options.Gravity; double l = Math.Sqrt((double)g.X * g.X + (double)g.Y * g.Y);
        return l > 0 ? (-g.X / l, -g.Y / l) : (0, 0);
    }

    private static double Height(Vector2D p, (double X, double Y) up) => p.X * up.X + p.Y * up.Y;

    private void Finish((double X, double Y) up, double reference, double speed)
    {
        Up = new Vector2F((float)up.X, (float)up.Y); MaxSpeed = speed;
        double lowest = double.PositiveInfinity;
        double x0 = double.PositiveInfinity, y0 = x0, x1 = double.NegativeInfinity, y1 = x1;
        foreach (var p in Positions)
        {
            double h = Height(p, up); if (h < lowest) lowest = h;
            x0 = Math.Min(x0, p.X); y0 = Math.Min(y0, p.Y); x1 = Math.Max(x1, p.X); y1 = Math.Max(y1, p.Y);
        }
        BoundsMin = new Vector2D(x0, y0); BoundsMax = new Vector2D(x1, y1);
        Sag = Math.Max(0, reference - lowest);
    }
}

public sealed partial class PhysicsWorld2D
{
    /// <summary>Reads back the current geometry of a soft body or cloth (positions, surface triangles, edges, bounds, sag) as plain arrays. Read-only; allocates the arrays.</summary>
    /// <exception cref="InvalidHandleException">The body is the default value or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public RestShape2D GetRestShape(SoftBody body)
    {
        _core.FlushPostStep();
        if (!ReferenceEquals(body.OwnerWorld, this)) throw new InvalidHandleException("The soft body is null or belongs to a different world.");
        return RestShape2D.FromSoft(this, body);
    }

    /// <summary>Reads back the current geometry of a rod (positions, segment frames, bounds, sag, arc length) as plain arrays. Read-only; allocates the arrays.</summary>
    /// <exception cref="InvalidHandleException">The rod is the default value or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">The rod was destroyed.</exception>
    public RestShape2D GetRestShape(Rod rod)
    {
        _core.FlushPostStep();
        if (!ReferenceEquals(rod.OwnerStore, _engine.Rods)) throw new InvalidHandleException("The rod is the default value or belongs to a different world.");
        if (!rod.IsValid) throw new StaleHandleException("The rod was destroyed.");
        return RestShape2D.FromRod(this, rod);
    }
}
