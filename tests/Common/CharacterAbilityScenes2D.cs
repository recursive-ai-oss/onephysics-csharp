#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Rods;

// 2D scenes for what characters do besides walking on rigid ground: stand on and push soft bodies and ropes, climb ledges on a tile grid, climb ladders and ride platforms. The hash-matrix runs a shortened
// version of each (snapshotted and restored half-way); the stability runner plays the full 600 steps and reads the scene's own measurements. Scripts use only arithmetic on the step number and the state
// of the world, so the scenes are byte-identical on every platform.
internal static class CharacterAbilityScenes2D
{
    internal const float Dt = 1f / 60f;
    internal const float G = 9.81f;

    internal static readonly string[] Names = ["characters-soft-2d", "climbing-2d"];

    internal sealed class Scene
    {
        internal required string Name;
        internal required PhysicsWorld2D World { get; set; }
        internal int Steps = 600;
        internal Action<int> Script = _ => { };
        internal SortedDictionary<string, double> Metrics = new();
        internal List<CharacterHandle> Characters = new();
        internal Action<int> Observe = _ => { };

        internal Character C(int i) => World.GetCharacter(Characters[i]);

        internal void Rebind(PhysicsWorld2D w) => World = w;
    }

    private static PhysicsWorld2D NewWorld(int threads, int id) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    private static Body Floor(PhysicsWorld2D w) => w.AddBox(new Vector2F(200f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);

    private sealed class Mover
    {
        internal CharacterHandle H;
        internal float Vx, Vy;

        internal void Tick(PhysicsWorld2D w)
        {
            var c = w.GetCharacter(H);
            if (c.Mode != CharacterMode.Free) { return; }
            Vy = c.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            c.Move(new Vector2F(Vx * Dt, Vy * Dt), Dt);
        }

        internal void Release(Vector2F v) { Vx = v.X; Vy = v.Y; }
    }

    internal static Scene Build(string name, int threads) => name switch
    {
        "characters-soft-2d" => SoftScene(threads),
        "climbing-2d" => ClimbingScene(threads),
        _ => throw new ArgumentException("unknown scene " + name),
    };

    private static Scene SoftScene(int threads)
    {
        var w = NewWorld(threads, 85);
        Floor(w);
        // A jelly pad to walk across, a soft bar to stand on (pinned at both ends, with slack), a rope bridge, and a jelly box to push.
        var pad = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(3f, 0.4f), 12, 2), new Vector2D(-3, 0.21));
        pad.ParticleRadius = 0.15f;
        w.AddSoftBody(pad);
        const int Nx = 30, Ny = 2;
        var bar = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(3.0f, 0.2f), Nx, Ny), new Vector2D(6, 1.5));
        bar.ParticleRadius = 0.09f; bar.Mass = 400f; bar.EdgeCompliance = 0f; bar.LinearDamping = 0.5f;
        var pins = new List<int>();
        for (int j = 0; j <= Ny; j++) { pins.Add(j * (Nx + 1)); pins.Add(j * (Nx + 1) + Nx); }
        bar.PinnedParticles = pins;
        var barBody = w.AddSoftBody(bar);
        for (int i = 0; i < barBody.ParticleCount; i++) { var p = barBody.GetPosition(i); barBody.SetPosition(i, new Vector2D(6 + (p.X - 6) * 0.9, p.Y)); }
        w.AddSoftBox(new Vector2F(0.6f, 0.6f), new Vector2D(-3, 0.31 + 4), SoftMaterial.Jelly, 4);
        var s = new Scene { Name = "characters-soft-2d", World = w };
        var walker = new Mover { H = w.AddCharacter(new Vector2D(-4.9, 0.91)).RawHandle };
        var stander = new Mover { H = w.AddCharacter(new Vector2D(6, 1.5 + 0.9 + 0.2)).RawHandle };
        var pd = CharacterDef.Humanoid; pd.PushStrength = 1f;
        var pusher = new Mover { H = w.AddCharacter(new Vector2D(-12, 0.91), pd).RawHandle };
        w.AddSoftBox(new Vector2F(0.6f, 0.6f), new Vector2D(-10.5, 0.31), SoftMaterial.Jelly, 4);
        s.Characters.AddRange([walker.H, stander.H, pusher.H]);
        double standerLast = double.NaN, jitter = 0; int jitterN = 0;
        s.Script = step =>
        {
            walker.Vx = ((step / 200) % 2 == 0) ? 0.9f : -0.9f;
            pusher.Vx = 0.5f;
            walker.Tick(s.World); stander.Tick(s.World); pusher.Tick(s.World);
        };
        s.Observe = step =>
        {
            if (step >= s.Steps - 200)
            {
                double y = s.C(1).Position.Y;
                if (!double.IsNaN(standerLast)) { jitter += Math.Abs(y - standerLast); jitterN++; }
                standerLast = y;
            }

            double grounded = 0, falls = 0;
            for (int i = 0; i < 3; i++) { var ch = s.C(i); if (ch.IsGrounded) { grounded++; } if (ch.Position.Y < -1.0) { falls++; } }
            s.Metrics["grounded"] = grounded; s.Metrics["falls"] = falls; s.Metrics["standerJitter"] = jitterN == 0 ? 0 : jitter / jitterN;
        };
        return s;
    }

    private static Scene ClimbingScene(int threads)
    {
        var w = NewWorld(threads, 86);
        Floor(w);
        var s = new Scene { Name = "climbing-2d", World = w };
        var def = CharacterDef.Humanoid; def.CanClimb = true;
        // A platformer level on a tile grid: ledges 2 m high at x = 4 and x = 14, a wall too tall to grab at x = 24 and a flat-topped step too low at x = 34; tiles are 0.5 m.
        const int W = 90, H = 14;
        var tiles = new TileKind2D[W * H];
        void Block(int x0, int x1, int rows) { for (int y = 0; y < rows; y++) { for (int x = x0; x < x1; x++) { tiles[y * W + x] = TileKind2D.Solid; } } }
        Block(8, 20, 4); Block(28, 40, 4); Block(48, 60, 12); Block(68, 80, 1);
        w.AddTileGrid(W, H, tiles, 0.5f, new Vector2D(0, 0));
        // A ladder: a climbable wall box.
        var ladder = w.AddBox(new Vector2F(3f, 5f), at: new Vector2D(101.5, 2.5), type: BodyType.Static);
        ladder.GetShapes()[0].Climbable = true;
        var movers = new List<Mover>();
        void Climber(double x) { var m = new Mover { H = w.AddCharacter(new Vector2D(x, 0.91), def).RawHandle }; movers.Add(m); s.Characters.Add(m.H); }
        Climber(3.5); Climber(13.5); Climber(23.5); Climber(33.5); Climber(99.5);
        var phase = new int[movers.Count]; var hangFrom = new int[movers.Count];
        double grabs = 0, falseGrabs = 0, ladderTop = 0;
        s.Script = step =>
        {
            for (int i = 0; i < movers.Count; i++)
            {
                var m = movers[i]; var c = s.World.GetCharacter(m.H);
                if (i < 4)
                {
                    if (phase[i] == 0 && step == 20 + 3 * i) { m.Vy = 7.6f; phase[i] = 1; }
                    if (phase[i] == 1 && c.Mode == CharacterMode.Free && m.Vy > 0f && c.TryGrabLedge(new Vector2F(1, 0)))
                    {
                        phase[i] = 2; hangFrom[i] = step;
                        if (i < 2) { grabs++; } else { falseGrabs++; }
                    }

                    if (phase[i] == 2 && step - hangFrom[i] >= 40) { if (c.ClimbUp()) { phase[i] = 3; m.Vy = 0; m.Vx = 1.2f; } else { c.Release(); phase[i] = 4; } }
                    if (phase[i] == 3 && step - hangFrom[i] >= 100) { m.Vx = 0; }
                }
                else
                {
                    if (phase[i] == 0 && step == 20) { if (c.TryClimb(new Vector2F(1, 0))) { phase[i] = 2; } }
                    if (phase[i] == 2 && c.Mode == CharacterMode.Climbing) { c.Climb(0.025f); }
                    if (phase[i] == 2 && c.Mode == CharacterMode.Free) { phase[i] = 3; m.Vy = 0; m.Vx = 0.3f; ladderTop = 1; }
                    if (phase[i] == 3 && step > 500) { m.Vx = 0; }
                }

                m.Tick(s.World);
            }
        };
        s.Observe = step =>
        {
            double onTop = 0, falls = 0;
            for (int i = 0; i < 2; i++) { var c = s.C(i); if (c.IsGrounded && c.Position.Y > 2.8) { onTop++; } }
            for (int i = 0; i < movers.Count; i++) { if (s.C(i).Position.Y < -1.0) { falls++; } }
            s.Metrics["grabs"] = grabs; s.Metrics["falseGrabs"] = falseGrabs; s.Metrics["onTop"] = onTop; s.Metrics["ladderTop"] = ladderTop; s.Metrics["falls"] = falls;
            s.Metrics["ladderOnTop"] = s.C(4).IsGrounded && s.C(4).Position.Y > 5.8 ? 1 : 0;
        };
        return s;
    }

    internal static ulong Run(string name, int threads, bool restoreHalfway, int steps = 160, Action<ulong>? perStep = null)
    {
        var s = Build(name, threads);
        var w = s.World;
        ulong acc = 14695981039346656037UL;
        for (int i = 0; i < steps; i++)
        {
            if (restoreHalfway && i == steps / 2) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); s.Rebind(w); }
            s.Script(i);
            w.Step(Dt);
            s.Observe(i);
            ulong h = w.ComputeHash();
            for (int c = 0; c < s.Characters.Count; c++)
            {
                var ch = s.C(c);
                h = h * 1099511628211UL ^ (ulong)ch.Mode;
                h = h * 1099511628211UL ^ (ulong)BitConverter.DoubleToInt64Bits(ch.Position.Y);
            }

            acc = (acc ^ h) * 1099511628211UL;
            perStep?.Invoke(h);
        }

        return acc;
    }
}
