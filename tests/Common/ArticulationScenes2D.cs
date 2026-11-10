#pragma warning disable OPHYS0002
#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

// The planar articulation scene: a 64-link chain with a 1000:1 end mass, a lighter chain that whips into a floor and a pile of boxes, and a planar arm that holds a pose and pushes a box along a table. The hash-matrix and smoke scene
// and the stability scene (articulation-chain-2d) in one place, driven by scripts that read only world state.
internal static class ArticulationScenes2D
{
    internal const float Dt = 1f / 60f;
    internal static readonly string[] Names = { "articulation-chain-2d" };

    internal sealed class Scene
    {
        internal string Name = "";
        internal PhysicsWorld2D World = null!;
        internal int Steps = 600;
        internal Action<int> Script = _ => { };
        internal BodyHandle[] Boxes = Array.Empty<BodyHandle>();
        internal readonly SortedDictionary<string, double> M = new();

        /// <summary>Points the scene at a restored or forked copy of its world.</summary>
        internal void Rebind(PhysicsWorld2D w) { World = w; }

        internal void Measure(int step)
        {
            var w = World;
            double maxSpeed = M.GetValueOrDefault("maxSpeed"), nan = M.GetValueOrDefault("nan"), esc = M.GetValueOrDefault("escapes"), pen = M.GetValueOrDefault("maxPenetration"), joint = M.GetValueOrDefault("jointError");
            double tunnelled = M.GetValueOrDefault("tunnelled");
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) { continue; }
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type == BodyType.Static) { continue; }
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y));
                if (Math.Abs(p.X) > 200 || p.Y < -2 || p.Y > 100) { esc++; }
                if (body.Type == BodyType.Dynamic)
                {
                    foreach (var c in w.GetContacts(body)) { for (int i = 0; i < c.PointCount; i++) { pen = Math.Max(pen, -(double)c.Separation(i)); } }
                    if (p.Y < -0.2) { tunnelled++; }
                }
            }

            double linkPen = M.GetValueOrDefault("linkPenetration");
            foreach (var art in w.GetArticulations()) { linkPen = Math.Max(linkPen, art.DeepestPenetration); }
            joint = Math.Max(joint, JointError(w));
            M["maxSpeed"] = maxSpeed; M["nan"] = nan; M["escapes"] = esc; M["maxPenetration"] = pen; M["linkPenetration"] = linkPen; M["tunnelled"] = tunnelled; M["jointError"] = joint;
        }
    }

    private static PhysicsWorld2D NewWorld(int threads, int id) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    internal static Scene Build(string name, int threads) => name switch
    {
        "articulation-chain-2d" => ChainScene(threads),
        _ => throw new ArgumentOutOfRangeException(nameof(name), name, "unknown scene"),
    };

    // The distance between each link's frame and the place on its parent that the joint puts it at, worst over the articulations, measured from the stored body poses.
    internal static double JointError(PhysicsWorld2D w)
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
                var r = parent.Rotation;
                double ex = parent.Position.X + r.C * o.X - r.S * o.Y, ey = parent.Position.Y + r.S * o.X + r.C * o.Y;
                worst = Math.Max(worst, Math.Sqrt((child.Position.X - ex) * (child.Position.X - ex) + (child.Position.Y - ey) * (child.Position.Y - ey)));
            }
        }

        return worst;
    }

    // A 64-link chain with a 1000:1 end mass hangs from a ceiling point; a 12-link chain with a 5:1 end whips down into a floor and a pile of boxes; a three-joint arm pushes a crate along a shelf with a task target.
    private static Scene ChainScene(int threads)
    {
        var w = NewWorld(threads, 1343);
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.7f, 0f, 0f));
        var heavy = ArticulationDef.Chain(64, 0.25f, 1f, 1000f);
        w.AddArticulation(heavy, new Vector2D(-10, 28));
        var whip = ArticulationDef.Chain(12, 0.25f, 1f, 5f);
        w.AddArticulation(whip, new Vector2D(2.5, 2.2));
        var boxes = new List<BodyHandle>();
        for (int i = 0; i < 4; i++) { boxes.Add(w.AddBox(new Vector2F(0.3f, 0.3f), new Vector2D(5.6 + 0.35 * i, 0.15)).Handle); }
        boxes.Add(w.AddBox(new Vector2F(0.3f, 0.3f), new Vector2D(5.8, 0.45)).Handle);
        var arm = ArticulationDef.Arm(0.9f, 3, 4f);
        var armBody = w.AddArticulation(arm, new Vector2D(12, 0));
        armBody.SetJointPosition(1, 0.9f); armBody.SetJointTarget(1, 0.9f);
        armBody.SetJointPosition(2, -1.5f); armBody.SetJointTarget(2, -1.5f);
        armBody.SetJointPosition(3, 0.9f); armBody.SetJointTarget(3, 0.9f);
        boxes.Add(w.AddBox(new Vector2F(0.12f, 0.12f), new Vector2D(12.45, 0.06), material: new Material2D(0.4f, 0f, 0f), density: 300f).Handle);
        var sc = new Scene { Name = "articulation-chain-2d", World = w, Steps = 600, Boxes = boxes.ToArray() };
        var local = new Vector2F(0, 0.3f);
        sc.Script = step =>
        {
            var a = sc.World.GetArticulations()[2];
            int tip = a.LinkCount - 1;
            var toolNow = a.GetPointPosition(tip, local);
            Vector2D target;
            // Closed loop on the world state alone: descend beside the crate, then push along the shelf.
            if (toolNow.X > 12.3 || toolNow.Y <= 0.12) { target = new Vector2D(toolNow.X + 0.03, 0.1); }
            else { target = new Vector2D(12.25, 0.1); }
            a.SetTaskTarget(tip, local, target, gain: 3f, damping: 0.05f, maxStep: 0.06f);
        };
        return sc;
    }

    /// <summary>Runs the scene (shortened) and mixes its per-step world hashes; with <paramref name="restoreHalfway"/> the world is snapshotted and restored into a fork half-way, which must not change a bit.</summary>
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
