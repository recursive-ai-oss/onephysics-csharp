using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// The bake cache reader: any bytes either open as a clip that can be sampled everywhere, or are refused with BakeFormatException.
internal static class FuzzBake
{
    public static void Run(byte[] bytes)
    {
        // Past the checksum first, so mutations reach the structure; the checksummed path too.
        Exercise(bytes, false);
        Exercise(bytes, true);
    }

    private static void Exercise(byte[] bytes, bool verify)
    {
        try
        {
            var clip = BakeClip.Parse(bytes, verify);
            var points = new Vector3D[4096];
            double step = Math.Max(clip.FrameDuration * 2.7, 1e-6);
            foreach (var e in clip.Rigid) { for (double t = -step; t <= clip.Duration + step; t += step) { e.TrySampleRigid(t, out _); } }
            foreach (var track in new[] { clip.Soft, clip.Rods, clip.Particles })
            {
                foreach (var e in track)
                {
                    if (e.PointCount > points.Length) { continue; }
                    for (double t = -step; t <= clip.Duration + step; t += step) { e.TrySamplePoints(t, points); }
                }
            }

            _ = clip.Events.Count + clip.Fractures.Count + clip.FragmentGeometry.Count;
            if (clip.FrameCount > 0) { clip.GetFrameHash(clip.FrameCount - 1); }
        }
        catch (BakeFormatException) { }
    }

    public static byte[] Seed(int dimension)
    {
        var o = new BakeOptions { LibraryVersion = "seed", SceneName = "seed", BlockFrames = 8 };
        if (dimension == 3)
        {
            var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 7393 });
            w.AddBox(new Vector3F(10, 1, 10), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var crate = w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D(2, 3, 0)); crate.LinearVelocity = new Vector3F(0, -6, 0);
            crate.SetFracturable(new P3.FractureDef { CellCount = 5, ImpulseThreshold = 3f, Seed = 5 });
            w.AddSoftBall(0.3f, new Vector3D(0, 1.5, 3), P3.SoftMaterial.Rubber, 2);
            w.AddRope(new Vector3D(-1, 3, 3), new Vector3D(1, 3, 3), 6);
            w.FillFluidBox(w.AddParticleFluid(ParticleFluidDef.Water(0.1f)), new Vector3D(-2, 0.2, -2), new Vector3D(-1.6, 0.5, -1.7));
            using var rec = w.BeginBake(o);
            for (int i = 0; i < 70; i++) { w.Step(1f / 60f); }
            return rec.Finish();
        }
        else
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7394 });
            w.AddBox(new Vector2F(10, 1), new Vector2D(0, -0.5), type: BodyType.Static);
            var crate = w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(2, 3)); crate.LinearVelocity = new Vector2F(0, -6);
            crate.SetFracturable(new P2.FractureDef { CellCount = 5, ImpulseThreshold = 3f, Seed = 5 });
            w.AddSoftCircle(0.3f, new Vector2D(0, 1.5), P2.SoftMaterial.Rubber, 2);
            w.AddRope(new Vector2D(-1, 3), new Vector2D(1, 3), 6);
            w.FillFluidBox(w.AddParticleFluid(ParticleFluidDef.Water(0.1f)), new Vector2D(-2, 0.2), new Vector2D(-1.6, 0.5));
            using var rec = w.BeginBake(o);
            for (int i = 0; i < 70; i++) { w.Step(1f / 60f); }
            return rec.Finish();
        }
    }
}
