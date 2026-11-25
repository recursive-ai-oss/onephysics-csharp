using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Common;

/// <summary>Worlds for the bake tests, the hash matrix and the smoke apps: rigid bodies, a soft body, cloth, a rope, a particle fluid and a fracturing crate in one world, in both dimensions.</summary>
internal static class BakeScenes
{
    internal const float Dt = 1f / 60f;

    internal static P3.PhysicsWorld3D Spatial(int threads, int worldId = 93)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = worldId, ThreadCount = threads });
        w.AddBox(new Vector3F(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(-3, 0.3 + 0.62 * i, 0), new QuaternionF(0, 0.05f * i, 0, 1f).Normalized()); }
        var ball = w.AddSphere(0.3f, new Vector3D(-3.1, 4, 0.1));
        ball.LinearVelocity = new Vector3F(0, 0, 0);
        var crate = w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D(3, 3.0, 0));
        crate.LinearVelocity = new Vector3F(0, -6, 0);
        crate.SetFracturable(new P3.FractureDef { CellCount = 6, ImpulseThreshold = 3f, Seed = 5 });
        w.AddSoftBall(0.3f, new Vector3D(0, 1.5, 3), P3.SoftMaterial.Rubber, 2);
        w.AddCloth(new P3.ClothDef { Width = 1.2f, Depth = 1.2f, SegmentsX = 8, SegmentsZ = 8, Position = new Vector3D(0, 2.2, -3), Material = P3.ClothMaterial.Cotton });
        w.AddRope(new Vector3D(-1, 3, 3), new Vector3D(1, 3, 3), 8);
        // The fluid falls inside a small tank of boxes.
        w.AddBox(new Vector3F(1.6f, 0.2f, 1.6f), new Vector3D(6, 0.1, 0), type: BodyType.Static);
        var fluid = w.AddParticleFluid(ParticleFluidDef.Water(0.1f));
        w.FillFluidBox(fluid, new Vector3D(5.5, 0.4, -0.3), new Vector3D(6.3, 0.9, 0.3));
        return w;
    }

    internal static P2.PhysicsWorld2D Planar(int threads, int worldId = 94)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = worldId, ThreadCount = threads });
        w.AddBox(new Vector2F(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(-3, 0.3 + 0.62 * i), 0.03f * i); }
        w.AddCircle(0.3f, new Vector2D(-3.05, 4));
        var crate = w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(3, 3.0));
        crate.LinearVelocity = new Vector2F(0, -6);
        crate.SetFracturable(new P2.FractureDef { CellCount = 6, ImpulseThreshold = 3f, Seed = 5 });
        w.AddSoftCircle(0.3f, new Vector2D(0, 1.5), P2.SoftMaterial.Rubber, 2);
        w.AddStrip(1.2f, 0.04f, 12, new Vector2D(-1, 3), 0.08f, pinStart: true);
        w.AddRope(new Vector2D(1, 3), new Vector2D(2.5, 3), 8);
        w.AddBox(new Vector2F(1.6f, 0.2f), new Vector2D(6, 0.1), type: BodyType.Static);
        var fluid = w.AddParticleFluid(ParticleFluidDef.Water(0.1f));
        w.FillFluidBox(fluid, new Vector2D(5.5, 0.4), new Vector2D(6.3, 0.9));
        return w;
    }
}
