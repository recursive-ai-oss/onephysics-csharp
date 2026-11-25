using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Presentation;
using RecursiveAI.OnePhysics.Rods;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// The presentation-world scenes, shared by the tests, the hash-matrix rows and the smoke apps. An authoritative world holds a floor, a platform that slides back and forth, a "character" (a kinematic capsule
// walking along a line) and a block that is fractured at a fixed step. The follower holds hair on the character, a cape (3D) or a flag (2D), a rope hanging from the platform and, from the fracture, debris.
// Motion is written without trigonometry (a triangle wave), so every platform builds the same worlds.
internal static class PresentationScenes
{
    internal const float Dt = 1f / 60f;
    internal const int FractureStep = 60;

    internal static double Triangle(int step, int period, double amplitude)
    {
        int phase = step % period, half = period / 2;
        double v = phase < half ? (double)phase / half : (double)(period - phase) / half;
        return amplitude * (2 * v - 1);
    }

    // ------------------------------------------------------------------ 3D

    internal sealed class Scene3
    {
        public P3.PhysicsWorld3D Authority = null!;
        public P3.PresentationWorld3D? Follower;
        public P3.Body Floor, Platform, Character, Block;
        public int Step;
        /// <summary>Set to time the two parts of a frame (Stopwatch ticks of the last frame).</summary>
        public bool Timing { get; set; }
        public long AuthorityTicks, FollowerTicks;

        /// <summary>One frame: drive the authority's kinematic bodies, step it, fracture the block at its step, then let the follower observe and step.</summary>
        public void Advance()
        {
            Platform.MoveKinematic(new Vector3D(Triangle(Step, 120, 3), 1.0, -3), QuaternionF.Identity);
            Character.MoveKinematic(new Vector3D(0, 1.0, -2 + 0.02 * Step), QuaternionF.Identity);
            long t0 = Timing ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Authority.Step(Dt);
            if (Step == FractureStep) Authority.Fracture(Block, new P3.FractureDef { CellCount = 6, Seed = 7 });
            long t1 = Timing ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Follower?.Step(Authority, Dt);
            if (Timing) { AuthorityTicks = t1 - t0; FollowerTicks = System.Diagnostics.Stopwatch.GetTimestamp() - t1; }
            Step++;
        }
    }

    internal static Scene3 Build3(int threads, bool withFollower, int followerWorldId = 82, PresentationQuality? quality = null)
    {
        var s = new Scene3();
        s.Authority = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 81, ThreadCount = threads });
        var a = s.Authority;
        s.Floor = a.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        s.Platform = a.AddBox(new Vector3F(2, 0.2f, 2), at: new Vector3D(0, 1.0, -3), type: BodyType.Kinematic);
        s.Character = a.AddCapsule(0.5f, 0.3f, at: new Vector3D(0, 1.0, -2), type: BodyType.Kinematic);
        s.Block = a.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(4, 0.5, 0), density: 500f);
        if (!withFollower) return s;
        var f = new P3.PresentationWorld3D(new P3.WorldOptions3D { WorldId = followerWorldId, ThreadCount = threads }, quality);
        s.Follower = f;
        f.Follow(s.Floor);
        var platform = f.Follow(s.Platform);
        var character = f.Follow(s.Character);
        f.Follow(s.Block);
        var w = f.World;
        // Hair: three strands hanging from the character's head, attached to its mirror.
        for (int i = 0; i < 3; i++)
        {
            double x = -0.1 + 0.1 * i;
            var points = new Vector3D[9];
            for (int k = 0; k < points.Length; k++) points[k] = new Vector3D(x, 1.55 - 0.05 * k, -2.0);
            w.AddRod(new P3.RodDef { Points = points, Material = RodMaterial.Hair, Attachments = [new P3.RodAttachment(0, character.RawHandle, new Vector3F((float)x, 0.55f, 0))], Substeps = 4, Iterations = 2 });
        }

        // A cape: an edge of the cloth is attached to the character's back.
        var cloth = w.AddCloth(0.5f, 0.6f, 6, new Vector3D(0, 1.3, -1.7));
        var edge = new int[7]; for (int i = 0; i < edge.Length; i++) edge[i] = i;
        w.AttachClothEdge(cloth, edge, character);
        // A rope hanging from the platform.
        var rope = w.AddRope(new Vector3D(0, 0.9, -3), new Vector3D(0, -0.1, -3), 10, pinStart: false);
        rope.Attach(0, platform, new Vector3F(0, -0.1f, 0));
        return s;
    }

    // ------------------------------------------------------------------ 2D

    internal sealed class Scene2
    {
        public P2.PhysicsWorld2D Authority = null!;
        public P2.PresentationWorld2D? Follower;
        public P2.Body Floor, Platform, Character, Block;
        public int Step;
        public bool Timing { get; set; }
        public long AuthorityTicks, FollowerTicks;

        public void Advance()
        {
            Platform.MoveKinematic(new Vector2D(Triangle(Step, 120, 3), 1.0), 0f);
            Character.MoveKinematic(new Vector2D(-2 + 0.02 * Step, 1.0), 0f);
            long t0 = Timing ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Authority.Step(Dt);
            if (Step == FractureStep) Authority.Fracture(Block, new P2.FractureDef { CellCount = 6, Seed = 7 });
            long t1 = Timing ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Follower?.Step(Authority, Dt);
            if (Timing) { AuthorityTicks = t1 - t0; FollowerTicks = System.Diagnostics.Stopwatch.GetTimestamp() - t1; }
            Step++;
        }
    }

    internal static Scene2 Build2(int threads, bool withFollower, int followerWorldId = 84, PresentationQuality? quality = null)
    {
        var s = new Scene2();
        s.Authority = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 83, ThreadCount = threads });
        var a = s.Authority;
        s.Floor = a.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        s.Platform = a.AddBox(new Vector2F(2, 0.2f), at: new Vector2D(0, 1.0), type: BodyType.Kinematic);
        s.Character = a.AddCapsule(0.5f, 0.3f, at: new Vector2D(-2, 1.0), angleRadians: 0f, type: BodyType.Kinematic);
        s.Block = a.AddBox(new Vector2F(1, 1), at: new Vector2D(4, 0.5), density: 500f);
        if (!withFollower) return s;
        var f = new P2.PresentationWorld2D(new P2.WorldOptions2D { WorldId = followerWorldId, ThreadCount = threads }, quality);
        s.Follower = f;
        f.Follow(s.Floor);
        var platform = f.Follow(s.Platform);
        var character = f.Follow(s.Character);
        f.Follow(s.Block);
        var w = f.World;
        for (int i = 0; i < 3; i++)
        {
            double x = -2.1 + 0.1 * i;
            var points = new Vector2D[9];
            for (int k = 0; k < points.Length; k++) points[k] = new Vector2D(x, 1.55 - 0.05 * k);
            w.AddRod(new P2.RodDef { Points = points, Material = RodMaterial.Hair, Attachments = [new P2.RodAttachment(0, character.RawHandle, new Vector2F((float)(x + 2), 0.55f))], Substeps = 4, Iterations = 2 });
        }

        // A flag (a soft strip pinned in the world; 2D soft strips have no rigid attachment) and a rope hanging from the platform.
        w.AddStrip(0.8f, 0.04f, 12, new Vector2D(3, 2.5), 0.08f, pinStart: true);
        var rope = w.AddRope(new Vector2D(0, 0.9), new Vector2D(0, -0.1), 10, pinStart: false);
        rope.Attach(0, platform, new Vector2F(0, -0.1f));
        return s;
    }
}
