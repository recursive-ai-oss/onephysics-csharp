using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene that exercises the world services: layers (a runtime matrix change), a sensor volume, per-pair events over a triangle mesh, hit events and queries every step.
// Shared by the hash-matrix runner and both smoke apps. Everything the services produce goes into the returned per-step hash, so any platform-dependent order or value shows up.
internal static class Services3DScene
{
    /// <summary>How many events of each kind the last <see cref="Run"/> on this thread saw.</summary>
    [ThreadStatic]
    internal static int[]? Seen;

    /// <summary>Runs 120 steps; <paramref name="perStep"/> receives the running hash after each. At step 60 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 31 });
        var floor = w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        floor.GetShapes()[0].Layer = 1;

        // A bumpy mesh patch the spheres roll over: one event per body pair, however many triangles touch.
        var verts = new Vector3F[9 * 9];
        for (int j = 0; j < 9; j++) { for (int i = 0; i < 9; i++) { verts[j * 9 + i] = new Vector3F(i, 0.15f * ((i * 3 + j) % 3), j); } }
        var idx = new int[8 * 8 * 6];
        int n = 0;
        for (int j = 0; j < 8; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                int a = j * 9 + i;
                idx[n++] = a; idx[n++] = a + 9; idx[n++] = a + 1; idx[n++] = a + 1; idx[n++] = a + 9; idx[n++] = a + 10;
            }
        }

        w.AddMesh(verts, idx, new Vector3D(-4, 1, -4));

        // A sensor volume the falling bodies pass through.
        var sensorBody = w.CreateBody(BodyType.Static, new Vector3D(6, 3, 0));
        w.CreateBox(sensorBody, new Vector3F(2f, 0.5f, 2f), 0f, new ShapeDef { IsSensor = true, Density = 0f });

        for (int i = 0; i < 10; i++)
        {
            var s = w.AddSphere(0.3f, new Vector3D(-3 + (i % 5) * 1.5, 4 + (i / 5) * 0.8, -2 + (i % 3) * 2.0));
            s.GetShapes()[0].Layer = 2;
        }

        for (int i = 0; i < 8; i++)
        {
            var b = w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(6 + (i % 2) * 0.4, 6 + i * 0.9, 0.2 * i));
            b.GetShapes()[0].Layer = 3;
        }

        var seen = Seen = new int[16];
        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong v) { hash = (hash ^ v) * 0x100000001B3UL; hash ^= hash >> 29; }
        byte[]? snap = null;
        var buf = new QueryHit3D[16];
        for (int step = 0; step < 120; step++)
        {
            if (step == 60 && restoreHalfway)
            {
                snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 31 });
                w.Restore(snap);
            }

            // Spheres stop colliding with the boxes at step 70 (the pairs that touch end), and collide again at step 100.
            if (step == 70) { w.IgnoreLayerCollision(2, 3); }
            if (step == 100) { w.SetLayerCollision(2, 3, true); }
            w.Step(1f / 60f);
            Mix(w.ComputeHash());
            foreach (var e in w.Events)
            {
                seen[(int)e.Kind]++;
                Mix((ulong)e.Kind); Mix((ulong)e.BodyA.Index); Mix((ulong)e.BodyB.Index); Mix((ulong)e.ShapeA.Index); Mix((ulong)e.ShapeB.Index);
                Mix((ulong)(uint)e.SubShapeA); Mix((ulong)(uint)e.SubShapeB); Mix((ulong)BitConverter.SingleToUInt32Bits(e.Value));
            }

            // Queries each step: results enter the hash.
            var from = new Vector3D(-3 + (step % 7), 8, -2 + (step % 5));
            if (w.Raycast(from, new Vector3F(0, -1, 0.05f), out var rh, 20f))
            {
                Mix((ulong)BitConverter.DoubleToUInt64Bits(rh.Point.X)); Mix((ulong)BitConverter.DoubleToUInt64Bits(rh.Point.Y)); Mix((ulong)BitConverter.DoubleToUInt64Bits(rh.Point.Z));
                Mix((ulong)BitConverter.SingleToUInt32Bits(rh.Normal.Y)); Mix((ulong)(uint)rh.SubShapeId);
            }

            if (w.SphereCast(from, 0.4f, new Vector3F(0.3f, -1, 0), out var ch, 20f))
            {
                Mix((ulong)BitConverter.SingleToUInt32Bits(ch.Fraction)); Mix((ulong)BitConverter.DoubleToUInt64Bits(ch.Point.Y));
            }

            int count = w.OverlapSphereAll(new Vector3D(6, 3, 0), 3f, buf);
            Mix((ulong)count);
            for (int k = 0; k < count; k++) { Mix((ulong)buf[k].Shape.RawHandle.Index); }
            if (w.ClosestPoint(new Vector3D(0, 2, 0), out var cp, 5f)) { Mix((ulong)BitConverter.SingleToUInt32Bits(cp.Distance)); }
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
