using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Bullets and the character mover together: bullets fired through crates and a kinematic platform, walking characters with steps, slopes and pushes, a character on a jointed plank, layers, a sensor,
/// and a snapshot restored halfway. Every step writes the world hash, every character's state and move result, the bullets' positions, the delivered events and the character debug entries.
/// </summary>
internal static class CharacterScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("world2d/characters", Run);

    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Range(ref ulong s, float lo, float hi) => lo + (hi - lo) * ((Next(ref s) >> 40) * (1f / 16777216f));

    private static PhysicsWorld2D Build(int threads, out Body[] bullets, out Character[] chars)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 12, Events = EventMask.All });
        ulong s = 7788;
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(0, -0.5)), 40f, 0.5f);
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(40, 5)), 0.5f, 6f);
        w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-40, 5)), 0.5f, 6f);
        // Stairs and a ramp for the movers.
        for (int i = 0; i < 4; i++) { w.CreateBox(w.CreateBody(BodyType.Static, new Vector2D(-6 + i * 0.5, 0.1 + 0.2 * i)), 0.25f, 0.1f + 0.2f * i); }
        w.CreatePolygon(w.CreateBody(BodyType.Static, new Vector2D(12, 0)), [new Vector2F(-3, 0), new Vector2F(3, 0), new Vector2F(3, 2)], 0f);
        // A kinematic platform sweeping back and forth, a sensor zone, a hinged plank.
        var platform = w.CreateBody(new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(-20, 3), LinearVelocity = new Vector2F(1.5f, 0f) });
        w.CreateBox(platform, 3f, 0.25f);
        var zone = w.CreateBody(new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(-14, 1), GravityScale = 0f });
        w.CreateBox(zone, 0.05f, 0.05f, new Vector2F(0, 8), 0f, 1f, new ShapeDef { Layer = 4 });
        w.CreateBox(zone, 1f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        w.IgnoreLayerCollision(4, 0);
        var pivot = w.AddCircle(0.1f, new Vector2D(24, 4), BodyType.Static);
        var plank = w.AddBox(new Vector2F(6f, 0.3f), at: new Vector2D(24, 4));
        w.AddHinge(pivot, plank, new Vector2D(24, 4));
        plank.AngularVelocity = 0.2f;
        // Crates, thin plates for the bullets to hit, and the bullets.
        for (int i = 0; i < 24; i++) { w.AddBox(new Vector2F(0.8f, 0.8f), at: new Vector2D(2 + i * 0.9, 0.4 + (i % 3) * 0.85)); }
        for (int i = 0; i < 6; i++)
        {
            var plate = w.CreateBody(BodyType.Dynamic, new Vector2D(30 + i * 1.5, 6));
            w.CreateBox(plate, 0.03f, 0.9f);
        }

        bullets = new Body[8];
        for (int i = 0; i < bullets.Length; i++)
        {
            var b = w.CreateBody(new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(Range(ref s, 26, 28), 5.2 + 0.2 * i), IsBullet = true, GravityScale = 0f });
            w.CreateCircle(b, 0.08f);
            b.LinearVelocity = new Vector2F(Range(ref s, 120, 240), Range(ref s, -3, 3));
            bullets[i] = b;
        }

        chars = new Character[14];
        for (int i = 0; i < chars.Length; i++)
        {
            var def = i % 2 == 0 ? Character.Humanoid : Character.Child;
            def.PushStrength = i % 3 == 0 ? 0.5f : 0f;
            double x = i < 4 ? -8 + 0.6 * i : i < 8 ? 6 + 0.9 * (i - 4) : i < 11 ? -21 + 1.2 * (i - 8) : 22 + 1.2 * (i - 11);
            double y = i < 11 ? (i >= 8 ? 3.5 : 1.0) : 4.6;
            chars[i] = w.AddCharacter(new Vector2D(x, y), def);
        }

        return w;
    }

    private static void Run(ScenarioContext ctx)
    {
        var w = Build(ctx.Threads, out var bullets, out var chars);
        var geometry = new DebugGeometry();
        byte[]? snapshot = null;
        var hits = new MoverHit2D[4];
        const float Dt = 1f / 60f;
        for (int step = 0; step < 240; step++)
        {
            if (step == 100) { snapshot = w.Snapshot(); ctx.Write(snapshot); }
            if (step == 130) { w.Restore(snapshot!); chars = w.GetCharacters(); }
            if (step == 180) { chars[5].Destroy(); chars = w.GetCharacters(); }

            for (int i = 0; i < chars.Length; i++)
            {
                var c = chars[i];
                float vx = (((step / 70) + i) & 1) == 0 ? 2.6f : -1.8f;
                float vy = c.IsGrounded ? ((step + 11 * i) % 97 == 0 ? 5f : -1f) : c.Velocity.Y - 9.81f * Dt;
                if (i % 5 == 4) { c.MoveVelocity(new Vector2F(vx, vy), Dt); }
                else { c.Move(new Vector2F(vx * Dt, vy * Dt), Dt); }
                ctx.Write(c.Position.X); ctx.Write(c.Position.Y); ctx.Write(c.IsGrounded ? 1 : 0); ctx.Write(c.Velocity.X); ctx.Write(c.Velocity.Y);
                ctx.Write(c.GroundNormal.X); ctx.Write(c.GroundNormal.Y);
            }

            w.Step(Dt);
            ctx.Write((long)w.ComputeHash());
            foreach (var b in bullets) { ctx.Write(b.Position.X); ctx.Write(b.Position.Y); ctx.Write(b.LinearVelocity.X); }
            foreach (var e in w.Events)
            {
                ctx.Write((int)e.Kind); ctx.Write(e.BodyA.Index); ctx.Write(e.BodyB.Index); ctx.Write(e.ShapeA.Index); ctx.Write(e.ShapeB.Index); ctx.Write(e.Value);
            }

            if (step % 10 == 0)
            {
                w.GetDebugGeometry(DebugContent.Characters | DebugContent.Contacts, geometry);
                ctx.Write(geometry.Characters.Length); ctx.Write(geometry.Contacts.Length);
                foreach (var c in geometry.Characters) { ctx.Write(c.Center.X); ctx.Write(c.Center.Y); ctx.Write(c.IsGrounded ? 1 : 0); ctx.Write(c.GroundNormal.X); ctx.Write(c.GroundBody.Index); }
            }

            ctx.EndStep();
        }
    }
}
