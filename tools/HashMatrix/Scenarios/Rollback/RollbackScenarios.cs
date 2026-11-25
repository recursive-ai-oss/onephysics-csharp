using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

// Mark and RollbackTo as a rollback host uses them: a ring of marks taken every frame, a late input that rolls back three frames and resimulates under a changed input, and a second rollback
// to the same mark. The world hash after every step, and the snapshot bytes after each rollback, must be the same on every platform.
internal static class RollbackScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("rollback/ring-2d", Ring2);
        Scenarios.Register("rollback/ring-3d", Ring3);
    }

    private const int Ring = 4;

    private static void Ring2(ScenarioContext ctx)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 811, ThreadCount = ctx.Threads });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        P2.Body first = default;
        for (int i = 0; i < 24; i++) { var b = w.AddBox(new Vector2F(0.8f, 0.8f), at: new Vector2D(0.15 * (i % 4), 0.5 + 0.9 * i)); if (i == 0) first = b; }
        w.AddCircle(0.4f, at: new Vector2D(3, 6));
        var ring = new WorldMark[Ring];
        int frame = 0;
        void Advance(bool mark)
        {
            if (mark) { ring[frame % Ring] = w.Mark(); w.ReleaseOldMarks(Ring); }
            w.Step(1f / 60);
            ctx.Write((long)w.ComputeHash()); ctx.Write(w.Events.Length); ctx.EndStep();
            frame++;
        }

        for (int i = 0; i < 70; i++) { Advance(true); }
        for (int round = 0; round < 2; round++)
        {
            int target = frame - 3;
            var changed = w.RollbackTo(ring[target % Ring]);
            ctx.Write((int)changed); ctx.Write(w.Snapshot());
            frame = target;
            first.LinearVelocity = new Vector2F(4 + round, 2);          // the late input
            using (w.SuppressCallbacks()) { for (int i = 0; i < 3; i++) { Advance(i > 0); } }
            for (int i = 0; i < 20; i++) { Advance(true); }
        }
    }

    private static void Ring3(ScenarioContext ctx)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 812, ThreadCount = ctx.Threads });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        P3.Body first = default;
        for (int i = 0; i < 24; i++) { var b = w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), at: new Vector3D(0.15 * (i % 4), 0.5 + 0.9 * i, 0.1 * (i % 3))); if (i == 0) first = b; }
        w.AddSphere(0.4f, at: new Vector3D(3, 6, 0));
        var ring = new WorldMark[Ring];
        int frame = 0;
        void Advance(bool mark)
        {
            if (mark) { ring[frame % Ring] = w.Mark(); w.ReleaseOldMarks(Ring); }
            w.Step(1f / 60);
            ctx.Write((long)w.ComputeHash()); ctx.Write(w.Events.Length); ctx.EndStep();
            frame++;
        }

        for (int i = 0; i < 70; i++) { Advance(true); }
        for (int round = 0; round < 2; round++)
        {
            int target = frame - 3;
            var changed = w.RollbackTo(ring[target % Ring]);
            ctx.Write((int)changed); ctx.Write(w.Snapshot());
            frame = target;
            first.LinearVelocity = new Vector3F(4 + round, 2, 1);
            using (w.SuppressCallbacks()) { for (int i = 0; i < 3; i++) { Advance(i > 0); } }
            for (int i = 0; i < 20; i++) { Advance(true); }
        }
    }
}
