using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>This library's 3D world behind the shared 3D driver loop.</summary>
sealed class OnePhysics3Sim : ISim3
{
    P3.PhysicsWorld3D world = null!;
    P3.Body[] bodies = [];
    int threads;

    public string Name => "onephysics";
    public string Version => "0.1.0-dev";
    public string Config => $"threads={threads}, substeps=4, default options";

    public void Build(Scene s, int threads)
    {
        this.threads = threads;
        (world, bodies, _) = SceneBuilder3D.Build(s, threads);
    }

    public void Step(double dt) => world.Step((float)dt);

    public void WakeAll(Scene s)
    {
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Type == "dynamic") bodies[i].Wake();
    }

    public void Snapshot(double[] d)
    {
        for (int i = 0; i < bodies.Length; i++)
        {
            int o = i * 14; var b = bodies[i];
            var p = b.Position; var q = b.Rotation; var v = b.LinearVelocity; var w = b.AngularVelocity;
            d[o] = p.X; d[o + 1] = p.Y; d[o + 2] = p.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
            d[o + 7] = v.X; d[o + 8] = v.Y; d[o + 9] = v.Z; d[o + 10] = w.X; d[o + 11] = w.Y; d[o + 12] = w.Z;
            d[o + 13] = b.IsAwake ? 1 : 0;
        }
    }
}
