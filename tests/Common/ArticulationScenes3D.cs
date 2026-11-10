#pragma warning disable OPHYS0002
#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// Articulation scenes in 3D: a seven-joint arm that pushes a box to a goal with a task-space target, and quadrupeds standing on a floor and on a height field. The hash-matrix and smoke scenes and the stability scenes
// (articulation-arm-3d, quadruped-3d) in one place, driven by scripts that read only world state, so a restored or forked copy continues bit for bit.
internal static class ArticulationScenes3D
{
    internal const float Dt = 1f / 60f;
    internal static readonly string[] Names = { "articulation-arm-3d", "quadruped-3d" };

    internal sealed class Scene
    {
        internal string Name = "";
        internal PhysicsWorld3D World = null!;
        internal int Steps = 600;
        internal Action<int> Script = _ => { };
        internal BodyHandle[] Boxes = Array.Empty<BodyHandle>();
        // The arm's goal for its box (z), when the scene has one.
        internal double Goal = double.NaN;
        internal readonly SortedDictionary<string, double> M = new();

        /// <summary>Points the scene at a restored or forked copy of its world.</summary>
        internal void Rebind(PhysicsWorld3D w) { World = w; }

        internal void Measure(int step)
        {
            var w = World;
            double maxSpeed = M.GetValueOrDefault("maxSpeed"), nan = M.GetValueOrDefault("nan"), esc = M.GetValueOrDefault("escapes"), pen = M.GetValueOrDefault("maxPenetration"), joint = M.GetValueOrDefault("jointError");
            double minTorso = M.GetValueOrDefault("minTorsoHeight", double.MaxValue), tunnelled = M.GetValueOrDefault("tunnelled");
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) { continue; }
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type == BodyType.Static) { continue; }
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z));
                if (Math.Abs(p.X) > 100 || Math.Abs(p.Z) > 100 || p.Y < -1 || p.Y > 50) { esc++; }
                if (body.Type == BodyType.Dynamic)
                {
                    foreach (var c in w.GetContacts(body)) { for (int i = 0; i < c.PointCount; i++) { pen = Math.Max(pen, -(double)c.Separation(i)); } }
                    if (p.Y < -0.2) { tunnelled++; }
                }
            }

            foreach (var art in w.GetArticulations())
            {
                pen = Math.Max(pen, art.DeepestPenetration);
                if (step >= 90 && !art.FixedBase) { minTorso = Math.Min(minTorso, art.GetLink(0).Position.Y); }
            }

            joint = Math.Max(joint, JointError(w));
            M["maxSpeed"] = maxSpeed; M["nan"] = nan; M["escapes"] = esc; M["maxPenetration"] = pen; M["tunnelled"] = tunnelled; M["jointError"] = joint;
            if (minTorso != double.MaxValue) { M["minTorsoHeight"] = minTorso; }
            if (step == Steps - 1)
            {
                if (!double.IsNaN(Goal)) { M["boxError"] = Math.Abs(w.GetBody(Boxes[0]).Position.Z - Goal); }
                int fallen = 0;
                foreach (var art in w.GetArticulations())
                {
                    if (art.FixedBase) { continue; }
                    var up = art.GetLink(0).Rotation.Rotate(new Vector3F(0, 1, 0));
                    if (up.Y < 0.7f || art.GetLink(0).Position.Y < 0.3) { fallen++; }
                }

                M["fallen"] = fallen;
            }
        }
    }

    private static PhysicsWorld3D NewWorld(int threads, int id) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    internal static Scene Build(string name, int threads) => name switch
    {
        "articulation-arm-3d" => ArmScene(threads),
        "quadruped-3d" => QuadrupedScene(threads),
        _ => throw new ArgumentOutOfRangeException(nameof(name), name, "unknown scene"),
    };

    // The distance between each link's frame and the place on its parent that the joint puts it at, worst over the articulations, measured from the stored body poses (zero but for the rounding of the stored float rotations).
    internal static double JointError(PhysicsWorld3D w)
    {
        double worst = 0;
        var store = w.Engine.Articulations;
        for (int i = 0; i < store.Handles.HighWater; i++)
        {
            if (!store.Handles.IsLive(i)) { continue; }
            var st = store.States[i]!;
            var m = st.Model;
            for (int l = 1; l < m.N; l++)
            {
                var parent = w.GetBody(w.Core.HandleOfBody(st.Body[m.Parent[l]]));
                var child = w.GetBody(w.Core.HandleOfBody(st.Body[l]));
                var o = m.Kind[l] == RecursiveAI.OnePhysics.Articulations.JointKind.Prismatic ? m.PlacePos[l] + m.PlaceRot[l].Rotate(m.Axis[l] * m.Q[m.PosOff[l]]) : m.PlacePos[l];
                var r = parent.Rotation.Rotate(new Vector3F((float)o.X, (float)o.Y, (float)o.Z));
                double dx = child.Position.X - (parent.Position.X + r.X), dy = child.Position.Y - (parent.Position.Y + r.Y), dz = child.Position.Z - (parent.Position.Z + r.Z);
                worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy + dz * dz));
            }
        }

        return worst;
    }

    // A seven-joint arm on a table-height base reaches over a pushed box: the tool point is steered to a place above and behind the box, down to the box's height, then forward until the box is at the goal. Two more boxes lie near the path.
    private static Scene ArmScene(int threads)
    {
        var w = NewWorld(threads, 1341);
        w.AddBox(new Vector3F(6, 1, 6), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        var arm = w.AddArticulation(ArticulationDef.RobotArm(0.9f, 7, 8f), new Vector3D(0, 0, 0));
        var ready = new[] { 0f, 0.9f, 0f, -1.4f, 0f, 1.0f, 0f };
        for (int j = 0; j < ready.Length; j++) { arm.SetJointPosition(j + 1, ready[j]); arm.SetJointTarget(j + 1, ready[j]); }
        var push = w.AddBox(new Vector3F(0.12f, 0.12f, 0.12f), at: new Vector3D(0, 0.06, 0.38), material: new Material3D(0.4f, 0f), density: 300f);
        var other = w.AddBox(new Vector3F(0.1f, 0.1f, 0.1f), at: new Vector3D(0.22, 0.05, 0.5), material: new Material3D(0.5f, 0f), density: 300f);
        var tall = w.AddBox(new Vector3F(0.08f, 0.2f, 0.08f), at: new Vector3D(-0.25, 0.1, 0.45), material: new Material3D(0.5f, 0f), density: 300f);
        var sc = new Scene { Name = "articulation-arm-3d", World = w, Steps = 600, Boxes = new[] { push.Handle, other.Handle, tall.Handle }, Goal = 0.55 };
        var local = new Vector3F(0, 0.1286f, 0);
        sc.Script = step =>
        {
            var a = sc.World.GetArticulations()[0];
            int tip = a.LinkCount - 1;
            var toolNow = a.GetPointPosition(tip, local);
            // Closed loop on the world state alone (so a restored copy continues the same way): descend behind the box, then push forward a little ahead of the tool until the box is near the goal, then hold.
            Vector3D target;
            if (toolNow.Z > 0.26 || toolNow.Y <= 0.10)
            {
                target = sc.World.GetBody(sc.Boxes[0]).Position.Z < sc.Goal - 0.012 ? new Vector3D(0, 0.075, toolNow.Z + 0.03) : toolNow;
            }
            else { target = new Vector3D(0, 0.075, 0.2); }
            a.SetTaskTarget(tip, local, target, gain: 3f, damping: 0.05f, maxStep: 0.06f);
        };
        return sc;
    }

    // Two quadrupeds, one on a box floor and one on a rolling height field, each pushed by a force on its torso for a tenth of a second at step 240 (about 1 m/s), with boxes dropped near them.
    private static Scene QuadrupedScene(int threads)
    {
        var w = NewWorld(threads, 1342);
        w.AddBox(new Vector3F(8, 1, 8), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        const int n = 41;
        var h = new float[n * n];
        for (int z = 0; z < n; z++) { for (int x = 0; x < n; x++) { h[z * n + x] = 0.02f * (float)Math.Sin(x * 0.5) * (float)Math.Cos(z * 0.4); } }
        w.AddHeightField(h, n, n, new Vector3F(0.25f, 1f, 0.25f), new Vector3D(4, 0, -5), material: new Material3D(0.8f, 0f, 0f));
        w.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(0, 0.62, 0));
        w.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(6.5, 0.62, 0));
        var boxes = new List<BodyHandle>();
        for (int i = 0; i < 3; i++) { boxes.Add(w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(0.8 + 0.1 * i, 0.4 + 0.25 * i, 0.9), density: 200f).Handle); }
        for (int i = 0; i < 3; i++) { boxes.Add(w.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), at: new Vector3D(7.3, 0.4 + 0.25 * i, 0.9 + 0.05 * i), density: 200f).Handle); }
        var sc = new Scene { Name = "quadruped-3d", World = w, Steps = 600, Boxes = boxes.ToArray() };
        sc.Script = step =>
        {
            if (step < 240 || step >= 246) { return; }
            var arts = sc.World.GetArticulations();
            arts[0].ApplyForce(0, new Vector3F(120, 0, 0), arts[0].GetLink(0).Position);
            arts[1].ApplyForce(0, new Vector3F(-100, 0, 0), arts[1].GetLink(0).Position);
        };
        return sc;
    }

    /// <summary>Runs both scenes (shortened) and mixes their per-step world hashes; with <paramref name="restoreHalfway"/> the world is snapshotted and restored into a fork half-way, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway)
    {
        ulong hash = 0xCBF29CE484222325UL;
        foreach (var name in Names)
        {
            var s = Build(name, threads);
            var w = s.World;
            for (int i = 0; i < 150; i++)
            {
                if (i == 70 && restoreHalfway) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); s.Rebind(w); }
                s.Script(i);
                w.Step(Dt);
                hash = (hash ^ w.ComputeHash()) * 0x100000001B3UL; hash ^= hash >> 29;
            }
        }

        return hash;
    }
}
