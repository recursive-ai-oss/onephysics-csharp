using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzArticulations
{
    // Hostile-byte reader: floats come from the input (NaN, infinities and huge values included) or fall back to a plain value.
    private sealed class Reader
    {
        private readonly byte[] _b; private int _at;
        public Reader(byte[] b, int start) { _b = b; _at = start; }
        public byte Byte(byte fallback) => _at < _b.Length ? _b[_at++] : fallback;
        public float Float(float fallback) { if (_at + 4 > _b.Length) { _at = _b.Length; return fallback; } var v = BitConverter.ToSingle(_b, _at); _at += 4; return v; }
        // Mostly sane values with the occasional hostile one, so deep paths are reached.
        public float Mix(float sane, int hostileEvery = 5) { byte k = Byte(1); return k % hostileEvery == 0 ? Float(sane) : sane * (0.25f + (k % 16) * 0.25f); }
    }

    // An articulation built from hostile bytes: joint kinds, axes, origins, masses, inertias, limits and drives. Creation either throws an ArgumentException or produces an articulation that steps,
    // answers its inverse-dynamics and task-space calls and snapshots and restores to the same bytes.
    internal static void Definition(byte[] input)
    {
        if (input.Length < 4) return;
        bool planar = (input[0] & 1) == 0;
        var r = new Reader(input, 2);
        int n = 1 + input[1] % 10;
        try
        {
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new() { WorldId = 51, SleepEnabled = false });
                w.AddBox(new(20f, 1f), new(0, -0.5), type: BodyType.Static);
                var def = new P2.ArticulationDef { FixedBase = (input[0] & 2) != 0, Substeps = (input[0] & 4) != 0 ? input[1] % 70 : 0, GravityCompensation = (input[0] & 8) != 0 };
                for (int i = 0; i < n; i++)
                {
                    var l = new P2.ArticulationLinkDef
                    {
                        Parent = i == 0 ? -1 : (r.Byte(0) % 4 == 0 ? r.Byte(0) % (i + 1) - 0 : i - 1), Joint = (ArticulationJointKind)(r.Byte(1) % 5), Axis = new Vector2F(r.Mix(1f), r.Mix(0f)),
                        Origin = new Vector2F(r.Mix(0.3f), r.Mix(0f)), Mass = r.Mix(1f), CenterOfMass = new Vector2F(r.Mix(0.1f, 9), 0), Inertia = r.Mix(0.05f),
                        HasLimits = r.Byte(0) % 3 == 0, LowerLimit = r.Mix(-1f), UpperLimit = r.Mix(1f), Friction = r.Mix(0.1f, 9), Damping = r.Mix(0.1f, 9), Armature = r.Mix(0.01f, 9), MaxVelocity = r.Mix(0f, 9),
                        Drive = new ArticulationDrive { Stiffness = r.Mix(20f), Damping = r.Mix(2f), MaxForce = r.Mix(0f, 9), TargetPosition = r.Mix(0f) },
                    };
                    if (r.Byte(0) % 2 == 0) l.AddShape(Shape2D.CreateCircle(new Vec2(r.Mix(0f, 9), 0f), r.Mix(0.1f)));
                    def.AddLink(l);
                }
                var art = w.AddArticulation(def, new Vector2D(r.Mix(0f), r.Mix(1f)));
                Exercise2D(w, art, r);
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new() { WorldId = 51, SleepEnabled = false });
                w.AddBox(new(20f, 1f, 20f), at: new(0, -0.5, 0), type: BodyType.Static);
                var def = new P3.ArticulationDef { FixedBase = (input[0] & 2) != 0, Substeps = (input[0] & 4) != 0 ? input[1] % 70 : 0, GravityCompensation = (input[0] & 8) != 0 };
                for (int i = 0; i < n; i++)
                {
                    var l = new P3.ArticulationLinkDef
                    {
                        Parent = i == 0 ? -1 : (r.Byte(0) % 4 == 0 ? r.Byte(0) % (i + 1) - 0 : i - 1), Joint = (ArticulationJointKind)(r.Byte(1) % 5), Axis = new Vector3F(r.Mix(1f), r.Mix(0f), r.Mix(0f)),
                        Origin = new Vector3F(r.Mix(0.3f), r.Mix(0f), r.Mix(0f)), OriginRotation = new QuaternionF(r.Mix(0f, 7), r.Mix(0f, 7), r.Mix(0f, 7), r.Mix(1f, 7)),
                        Mass = r.Mix(1f), CenterOfMass = new Vector3F(r.Mix(0.1f, 9), 0, 0), InertiaTensor = new Matrix3x3F(r.Mix(0.05f), r.Mix(0f, 9), r.Mix(0f, 9), r.Mix(0f, 9), r.Mix(0.05f), r.Mix(0f, 9), r.Mix(0f, 9), r.Mix(0f, 9), r.Mix(0.05f)),
                        HasLimits = r.Byte(0) % 3 == 0, LowerLimit = r.Mix(-1f), UpperLimit = r.Mix(1f), SwingLimit = r.Mix(1f, 9), Friction = r.Mix(0.1f, 9), Damping = r.Mix(0.1f, 9), Armature = r.Mix(0.01f, 9), MaxVelocity = r.Mix(0f, 9),
                        Drive = new ArticulationDrive { Stiffness = r.Mix(20f), Damping = r.Mix(2f), MaxForce = r.Mix(0f, 9), TargetPosition = r.Mix(0f) },
                    };
                    if (r.Byte(0) % 2 == 0) l.AddShape(P3.ShapeSpec3D.Sphere(r.Mix(0.1f)));
                    def.AddLink(l);
                }
                var art = w.AddArticulation(def, new Vector3D(r.Mix(0f), r.Mix(1f), r.Mix(0f)));
                Exercise3D(w, art, r);
            }
        }
        catch (ArgumentException) { }
        catch (PhysicsException) { }
    }

    private static void Exercise3D(P3.PhysicsWorld3D w, P3.Articulation art, Reader r)
    {
        var buf = new double[art.VelocityCount];
        var qdd = new double[art.VelocityCount];
        for (int i = 0; i < 6; i++)
        {
            if (i == 1) { try { art.SetTaskTarget(art.LinkCount - 1, new Vector3F(r.Mix(0f), r.Mix(0f), r.Mix(0f)), new Vector3D(r.Mix(0.3f), r.Mix(0.5f), r.Mix(0.3f)), r.Mix(1f), r.Mix(0.1f), r.Mix(0.05f)); } catch (Exception e) when (e is ArgumentException or InvalidOperationException) { } }
            if (i == 2) { try { art.ApplyForce(art.LinkCount - 1, new Vector3F(r.Mix(10f), r.Mix(0f), r.Mix(0f)), art.GetLink(0).Position); } catch (Exception e) when (e is ArgumentException or InvalidOperationException) { } }
            if (i == 3) { try { art.SetJointVelocity(Math.Min(1, art.LinkCount - 1), r.Mix(1f)); } catch (Exception e) when (e is ArgumentException or InvalidOperationException) { } }
            w.Step(1f / 60);
        }

        art.ComputeGravityCompensation(buf); art.ComputeInverseDynamics(qdd, buf); art.ComputeMassMatrix(new double[art.VelocityCount * art.VelocityCount]);
        var bytes = w.Snapshot(); w.Restore(bytes);
        if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Articulation round trip changed bytes.");
    }

    private static void Exercise2D(P2.PhysicsWorld2D w, P2.Articulation art, Reader r)
    {
        var buf = new double[art.VelocityCount];
        var qdd = new double[art.VelocityCount];
        for (int i = 0; i < 6; i++)
        {
            if (i == 1) { try { art.SetTaskTarget(art.LinkCount - 1, new Vector2F(r.Mix(0f), r.Mix(0f)), new Vector2D(r.Mix(0.3f), r.Mix(0.5f)), r.Mix(1f), r.Mix(0.1f), r.Mix(0.05f)); } catch (Exception e) when (e is ArgumentException or InvalidOperationException) { } }
            if (i == 3) { try { art.SetJointVelocity(Math.Min(1, art.LinkCount - 1), r.Mix(1f)); } catch (Exception e) when (e is ArgumentException or InvalidOperationException) { } }
            w.Step(1f / 60);
        }

        art.ComputeGravityCompensation(buf); art.ComputeInverseDynamics(qdd, buf); art.ComputeMassMatrix(new double[art.VelocityCount * art.VelocityCount]);
        var bytes = w.Snapshot(); w.Restore(bytes);
        if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Articulation round trip changed bytes.");
    }

    private static byte[]? s_planar, s_spatial;
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new() { WorldId = 51, SleepEnabled = false });
            w.AddBox(new(20f, 1f), new(0, -0.5), type: BodyType.Static);
            w.AddArticulation(P2.ArticulationDef.Arm(0.9f, 3, 4f), new Vector2D(0, 0)).SetTaskTarget(2, new Vector2F(0, 0.3f), new Vector2D(0.4, 0.5));
            w.AddArticulation(P2.ArticulationDef.Chain(6, 0.25f, 1f, 5f), new Vector2D(3, 3));
            for (int i = 0; i < 4; i++) w.Step(1f / 60);
            return s_planar = w.Snapshot();
        }
        if (s_spatial != null) return s_spatial;
        var world = new P3.PhysicsWorld3D(new() { WorldId = 51, SleepEnabled = false });
        world.AddBox(new(20f, 1f, 20f), at: new(0, -0.5, 0), type: BodyType.Static);
        world.AddArticulation(P3.ArticulationDef.Quadruped(), new Vector3D(0, 0.62, 0));
        world.AddArticulation(P3.ArticulationDef.RobotArm(0.9f, 4, 4f), new Vector3D(3, 0, 0)).SetTaskTarget(3, new Vector3F(0, 0.1f, 0), new Vector3D(3.2, 0.5, 0.2));
        for (int i = 0; i < 4; i++) world.Step(1f / 60);
        return s_spatial = world.Snapshot();
    }

    // Damaged snapshots with articulation state: restore either rejects them with a SnapshotFormatException leaving the world unchanged, or accepts them and writes the same bytes back (and the world steps or throws a PhysicsException).
    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(4000, bytes.Length)); bytes[offset] ^= input[i + 2];
        }
        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 51 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected articulation snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted articulation snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 51 }); var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected articulation snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted articulation snapshot changed bytes.");
            for (int i = 0; i < 2; i++) { try { world.Step(1f / 60); } catch (PhysicsException) { return; } }
        }
    }
}
