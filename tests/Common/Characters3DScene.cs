using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with characters: walkers climbing stairs and a ramp, one riding a sliding platform and one turning platform, one pushing crates, a jumper and a rigid character. Shared by the hash-matrix runner and
// both smoke apps. Character state (position, ground, velocity) enters the per-step hash beside the world hash, so any platform-dependent value shows up.
internal static class Characters3DScene
{
    /// <summary>Runs 150 steps; <paramref name="perStep"/> receives the running hash after each. At step 70 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 41 });
        w.SetLayerCollision(1, 1, false);
        w.AddBox(new Vector3F(80f, 1f, 30f), new Vector3D(10, -0.5, 0), type: BodyType.Static);

        // Stairs of 0.25 m risers and 0.5 m treads, built from boxes.
        for (int k = 1; k <= 8; k++) { w.AddBox(new Vector3F(0.5f, 0.25f * k, 4f), new Vector3D(2 + 0.5 * (k - 1), 0.125 * k, -8), type: BodyType.Static); }

        // A ramp of 30 degrees as a mesh (two triangles) and a block field with a seam between two tiles.
        w.AddMesh([new Vector3F(0, 0, 0), new Vector3F(0, 0, 4), new Vector3F(8, 4.6188f, 4), new Vector3F(8, 4.6188f, 0)], [0, 1, 2, 0, 2, 3], new Vector3D(14, 0, -10));
        var blocks = new Vector3F[16]; var bi = new int[3 * 6];
        for (int j = 0; j < 2; j++) { for (int i = 0; i < 4; i++) { blocks[j * 4 + i] = new Vector3F(i, i >= 2 ? 0.25f : 0f, j * 4); } }
        for (int i = 0; i < 3; i++) { int a = i, b = i + 4, c = i + 1, d = i + 5; bi[6 * i] = a; bi[6 * i + 1] = b; bi[6 * i + 2] = c; bi[6 * i + 3] = c; bi[6 * i + 4] = b; bi[6 * i + 5] = d; }
        w.AddMesh(blocks, bi, new Vector3D(30, 0, -2));

        // A platform sliding along x and a turning one.
        var slide = w.AddBox(new Vector3F(4f, 0.5f, 4f), new Vector3D(10, 0.25, 8), type: BodyType.Kinematic);
        slide.LinearVelocity = new Vector3F(1.5f, 0, 0);
        var turn = w.AddBox(new Vector3F(4f, 0.5f, 4f), new Vector3D(-10, 0.25, 8), type: BodyType.Kinematic);
        turn.AngularVelocity = new Vector3F(0, 0.4f, 0);

        // Crates for the pusher.
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(-4 + 1.5 * i, 0.3005, 3), density: 100f); }

        var chars = new Character[8];
        var plan = new (double X, double Y, double Z, float Push, int Jump)[]
        {
            (-1, 1.0, -8, 0f, 0), (-1, 1.0, -6, 0f, 0), (11, 1.0, -8, 0f, 0), (28, 1.0, -2, 0f, 0),
            (9, 1.55, 8, 0f, 0), (-8, 1.55, 8, 0f, 0), (-7, 1.0, 3, 0.8f, 0), (-3, 1.0, -1, 0f, 40),
        };
        for (int i = 0; i < chars.Length; i++)
        {
            var def = CharacterDef.Humanoid;
            def.PushStrength = plan[i].Push; def.Layer = 1; def.LayerMask = ~(1u << 1);
            chars[i] = w.AddCharacter(new Vector3D(plan[i].X, plan[i].Y, plan[i].Z), def);
        }

        var rigid = w.AddCharacter(new Vector3D(0, 1.0, 12), CharacterDef.RigidHumanoid);
        var vy = new float[chars.Length];
        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong v) { hash = (hash ^ v) * 0x100000001B3UL; hash ^= hash >> 29; }
        byte[]? snap = null;
        const float dt = 1f / 60f;
        for (int step = 0; step < 150; step++)
        {
            if (step == 70 && restoreHalfway)
            {
                snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 41 });
                w.Restore(snap);
                for (int i = 0; i < chars.Length; i++) { chars[i] = w.GetCharacters()[i]; }
                rigid = w.GetCharacters()[chars.Length];
            }

            for (int i = 0; i < chars.Length; i++)
            {
                var ch = chars[i];
                float walk = i == 5 ? 0f : 2f;
                if (ch.IsGrounded && vy[i] <= 0f) { vy[i] = plan[i].Jump > 0 && step > 0 && step % plan[i].Jump == 0 ? 5f : -1f; } else { vy[i] -= 10f * dt; }
                var cc = ch.Move(new Vector3F(walk * dt, vy[i] * dt, 0f), dt);
                Mix((ulong)cc);
            }

            rigid.Move(new Vector3F(0.5f * dt, 0, 0), dt);
            w.Step(dt);
            Mix(w.ComputeHash());
            for (int i = 0; i < chars.Length; i++)
            {
                var ch = chars[i]; var p = ch.Position; var v = ch.Velocity;
                Mix(BitConverter.DoubleToUInt64Bits(p.X)); Mix(BitConverter.DoubleToUInt64Bits(p.Y)); Mix(BitConverter.DoubleToUInt64Bits(p.Z));
                Mix(BitConverter.SingleToUInt32Bits(v.X)); Mix(BitConverter.SingleToUInt32Bits(v.Y)); Mix((ulong)ch.Ground);
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }
}
