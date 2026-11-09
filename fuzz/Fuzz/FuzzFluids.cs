using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fluids;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Particle fluids: definitions built from arbitrary floats (a typed argument error or a world that steps, snapshots and restores to the same bytes) and the section reader fed damaged bytes.
internal static class FuzzFluids
{
    private static float F(byte[] b, int at) => at + 4 <= b.Length ? BitConverter.ToSingle(b, at) : 1f;

    internal static void Definition(byte[] input)
    {
        if (input.Length < 8) return;
        bool planar = (input[0] & 1) == 0;
        try
        {
            var def = new ParticleFluidDef
            {
                Spacing = 0.1f * (1 + input[1] % 4), RestDensity = F(input, 8), KernelRadiusScale = 1.2f + (input[2] % 28) / 10f, Viscosity = F(input, 12), Vorticity = F(input, 16),
                TensileCorrection = F(input, 20), Relaxation = F(input, 24), Friction = F(input, 28), MaxSpeed = F(input, 32), GravityScale = F(input, 36), Layer = input[3] % 40, GroupIndex = (sbyte)input[4],
            };
            var emitter = new FluidEmitterDef { Position = new Vector3D(F(input, 40), F(input, 44), 0), Direction = new Vector3F(F(input, 48), F(input, 52), F(input, 56)), Size = new Vector3F(F(input, 60), F(input, 64), 0.1f), Speed = F(input, 68), Kind = (FluidVolumeKind)(input[5] & 1), MaxEmitted = input[6] };
            var drain = new FluidDrainDef { Position = new Vector3D(F(input, 72), F(input, 76), 0), Size = new Vector3F(F(input, 80), F(input, 84), 1f), Kind = (FluidVolumeKind)(input[7] & 1) };
            var settings = new FluidSettings { Substeps = 1 + input[1] / 64, Iterations = 1 + input[2] / 64, MaxParticles = 200 + input[3], SleepSpeed = 0.02f, TimeToSleep = 0.3f, WakeBodySpeed = 0.05f };
            if (planar)
            {
                var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 41 });
                w.AddBox(new Vector2F(4, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
                w.FluidSettings = settings;
                var f = w.AddParticleFluid(def);
                w.FillFluidBox(f, new Vector2D(-0.4, 0), new Vector2D(0.4, 0.4));
                f.AddEmitter(emitter); f.AddDrain(drain);
                for (int i = 0; i < 4; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Fluid round trip changed bytes.");
            }
            else
            {
                var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 41 });
                w.AddBox(new Vector3F(4, 0.2f, 4), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
                w.FluidSettings = settings;
                var f = w.AddParticleFluid(def);
                w.FillFluidBox(f, new Vector3D(-0.3, 0, -0.3), new Vector3D(0.3, 0.3, 0.3));
                f.AddEmitter(emitter); f.AddDrain(drain);
                for (int i = 0; i < 3; i++) w.Step(1f / 60);
                var bytes = w.Snapshot(); w.Restore(bytes);
                if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Fluid round trip changed bytes.");
            }
        }
        catch (ArgumentException) { }
        catch (FluidBudgetException) { }
    }

    private static byte[]? s_planar, s_spatial;

    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 42 });
            w.AddBox(new Vector2F(4, 0.2f), at: new Vector2D(0, -0.1), type: BodyType.Static);
            var f = w.AddParticleWater(0.1f);
            w.FillFluidBox(f, new Vector2D(-0.5, 0), new Vector2D(0.5, 0.4));
            f.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.5, 0), Direction = new Vector3F(0, -1, 0), Kind = FluidVolumeKind.Box, Size = new Vector3F(0.2f, 0.1f, 0.1f), Speed = 1f });
            f.AddDrain(new FluidDrainDef { Position = new Vector3D(2, 0, 0), Size = new Vector3F(0.1f, 0.1f, 0.1f) });
            w.AddBox(new Vector2F(0.3f, 0.3f), at: new Vector2D(0, 0.5), density: 400f);
            for (int i = 0; i < 20; i++) w.Step(1f / 60);
            return s_planar = w.Snapshot();
        }

        if (s_spatial != null) return s_spatial;
        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 42 });
        w3.AddBox(new Vector3F(4, 0.2f, 4), at: new Vector3D(0, -0.1, 0), type: BodyType.Static);
        var f3 = w3.AddParticleWater(0.12f);
        w3.FillFluidBox(f3, new Vector3D(-0.4, 0, -0.4), new Vector3D(0.4, 0.4, 0.4));
        f3.AddEmitter(new FluidEmitterDef { Position = new Vector3D(0, 1.5, 0), Direction = new Vector3F(0, -1, 0), Size = new Vector3F(0.1f, 0.1f, 0.1f), Speed = 1f });
        w3.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(0, 0.5, 0), density: 400f);
        for (int i = 0; i < 20; i++) w3.Step(1f / 60);
        return s_spatial = w3.Snapshot();
    }

    internal static void Snapshot(byte[] input)
    {
        bool planar = input.Length == 0 || (input[0] & 1) == 0;
        var bytes = (byte[])Baseline(planar).Clone();
        for (int i = 1; i + 2 < input.Length; i += 3)
        {
            int offset = bytes.Length - 1 - (((input[i] << 8) | input[i + 1]) % Math.Min(4000, bytes.Length));
            bytes[offset] ^= input[i + 2];
        }

        if (input.Length > 0 && (input[0] & 15) == 15) Array.Resize(ref bytes, input[^1] * bytes.Length / 256);
        if (planar)
        {
            var world = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 42 });
            var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected fluid snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted fluid snapshot changed bytes.");
            try { world.Step(1f / 60); } catch (P2.BodyOutOfBoundsException) { } catch (FluidBudgetException) { }
        }
        else
        {
            var world = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 42 });
            var before = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected fluid snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted fluid snapshot changed bytes.");
            try { world.Step(1f / 60); } catch (P3.BodyOutOfBoundsException) { } catch (FluidBudgetException) { }
        }
    }
}
