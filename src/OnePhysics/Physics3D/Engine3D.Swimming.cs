using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    /// <summary>Character water events of the last step, in character slot order (cleared at the start of the next step).</summary>
    internal readonly List<CharacterWaterRecord> CharacterWaterEvents = new();

    private const int ParticleProbeSamples = 12;

    private bool AnyWaterForCharacters()
    {
        if (_water != null && _water.Count > 0) { return true; }
        if (FluidHandles.Count > 0 || ParticleFluids.FluidHandles.Count > 0) { return true; }
        var defs = Fields.Definitions;
        for (int i = 0; i < defs.Length; i++) { if (defs[i] != null && defs[i]!.Kind == FieldKind.Buoyancy) { return true; } }
        return false;
    }

    /// <summary>The geometry of a character's capsule along the vertical: the lowest point and the vertical extent (an upright capsule exactly; a tilted one by its vertical extent).</summary>
    private void CharacterVertical(int c, Vector3D centre, out double bottom, out float extent)
    {
        var def = CharDef[c];
        var up = UnitUp(def.Up);
        float half = (0.5f * def.Height - def.Radius) * Math.Abs(up.Y) + def.Radius;
        bottom = centre.Y - half;
        extent = 2f * half;
    }

    /// <summary>Looks for the deepest water at a character (shallow water, buoyancy volumes and particle fluids, in that order for ties).</summary>
    private void ProbeCharacterWater(int c, Vector3D centre, out WaterProbe probe, out double bottom, out float extent)
    {
        probe = default;
        CharacterVertical(c, centre, out bottom, out extent);
        if (_water != null && _water.Count > 0) { _water.Probe(centre.X, centre.Z, bottom, extent, ref probe); }
        for (int f = 0; f < FluidHandles.HighWater; f++)
        {
            if (!FluidHandles.IsLive(f)) { continue; }
            var d = Fluids[f]!;
            probe.OfferPlane(CharacterWaterSource.BuoyancyVolume, d.SurfaceNormal, d.SurfaceOffset, d.Density, d.Flow, d.LimitToRegion, d.RegionMin, d.RegionMax, centre.X, centre.Z, bottom, extent);
        }

        var defs = Fields.Definitions;
        for (int i = 0; i < defs.Length; i++)
        {
            var d = defs[i];
            if (d == null || d.Kind != FieldKind.Buoyancy) { continue; }
            probe.OfferPlane(CharacterWaterSource.BuoyancyVolume, d.Axis, d.SurfaceHeight, d.Density, d.Flow, true, d.Minimum, d.Maximum, centre.X, centre.Z, bottom, extent);
        }

        var pf = ParticleFluids;
        if (pf.FluidHandles.Count > 0 && pf.Count > 0)
        {
            pf.PrepareQueries();
            float off = CharDef[c].Radius + 0.75f * pf.Spacing;
            int wet = 0;
            float rest = 0f;
            Vector3F flow = default;
            for (int i = 0; i < ParticleProbeSamples; i++)
            {
                double y = bottom + (i + 0.5) * extent / ParticleProbeSamples;
                // The character's own body pushes particles out of its volume, so the fluid is looked for just outside it, on four sides.
                float density = 0f, r0 = 0f; Vector3F vel = default;
                for (int k = 0; k < 4; k++)
                {
                    double ox = k == 0 ? off : k == 1 ? -off : 0.0, oz = k == 2 ? off : k == 3 ? -off : 0.0;
                    float dk = pf.SampleFlow(centre.X + ox, y, centre.Z + oz, out var vk, out float rk);
                    if (dk > density) { density = dk; vel = vk; r0 = rk; }
                }

                if (r0 > 0f && density >= 0.4f * r0)
                {
                    wet++; flow = new Vector3F(flow.X + vel.X, flow.Y + vel.Y, flow.Z + vel.Z);
                    if (rest == 0f) { rest = r0; }
                }
            }

            if (wet > 0)
            {
                float inv = 1f / wet;
                probe.Offer(CharacterWaterSource.ParticleFluid, (double)wet * extent / ParticleProbeSamples, rest, new Vector3F(flow.X * inv, flow.Y * inv, flow.Z * inv), extent);
            }
        }
    }

    /// <summary>Nothing under the character's feet and it is not moving up: what it meets in water it falls into.</summary>
    private bool Airborne(int c) => CharGround[c] != GroundState.OnGround && CharVy[c] <= 0.5f;

    private float SwimGravity() => Math.Max(-Opt.Gravity.Y, 0f);

    /// <summary>
    /// A virtual character's move, adjusted for the water it is in: slowed and carried a little while wading; while swimming, the displacement is a swim intent relative to the water and the move that comes
    /// back follows the swim law (current, buoyancy, drag). <paramref name="swimming"/> tells the caller to hand the move's real result back through <see cref="SwimCommit"/>.
    /// </summary>
    public Vector3F SwimAdjust(int c, Vector3F displacement, float dt, out bool swimming)
    {
        swimming = false;
        if (!(dt > 0f) || !float.IsFinite(dt)) { return displacement; }
        var st = Extensions.At(c);
        if (st.NoSwim || (st.Water == CharacterWaterState.Dry && !AnyWaterForCharacters())) { return displacement; }
        var centre = CharacterPosition(c);
        ProbeCharacterWater(c, centre, out var probe, out _, out float extent);
        if (!probe.Found) { return displacement; }
        float immersion = probe.Depth / extent;
        float charDensity = Swimming.DensityOf(in st);
        var state = Swimming.Next(st.Water, immersion, charDensity / probe.Density, Airborne(c));
        if (state == CharacterWaterState.Dry) { return displacement; }
        if (state == CharacterWaterState.Wading)
        {
            float t = immersion >= Swimming.SwimEnter ? 1f : immersion / Swimming.SwimEnter;
            float k = Swimming.WadeFactor(immersion);
            float carry = Swimming.WadeCarry * t * dt;
            return new Vector3F(displacement.X * k + probe.Flow.X * carry, displacement.Y, displacement.Z * k + probe.Flow.Z * carry);
        }

        float def = Swimming.SubmergedFraction(false, CharDef[c].Radius, extent, probe.Depth);
        float g = SwimGravity();
        float up = g > 0f ? g * (probe.Density * def / charDensity - 1f) : 0f;
        var v0 = st.Water == CharacterWaterState.Swimming ? st.SwimVelocity : new Vector3F(CharVx[c], CharVy[c], CharVz[c]);
        var v = Swimming.Integrate(v0, displacement / dt, probe.Flow, up, Swimming.DampingOf(in st), dt, false);
        swimming = true;
        st.SwimVelocity = v;
        Extensions.Set(c, in st);
        return v * dt;
    }

    /// <summary>After a swimming move: the swimmer's velocity is what the mover let it do (a wall or the bed stops it).</summary>
    public void SwimCommit(int c, Vector3F moved, float dt)
    {
        var st = Extensions.At(c);
        st.SwimVelocity = moved / dt;
        Extensions.Set(c, in st);
    }

    /// <summary>
    /// A rigid character's move adjusted for water: wading slows its walking; swimming replaces the walking step by a relaxation of its velocity towards the water's plus the swim intent (buoyancy, weight and
    /// water drag are the solver's). Returns true when the character swims and the walking code must not run.
    /// </summary>
    public bool SwimRigid(int c, ref Vector3F displacement, float dt)
    {
        var st = Extensions.At(c);
        if (st.NoSwim || (st.Water == CharacterWaterState.Dry && !AnyWaterForCharacters())) { return false; }
        var centre = CharacterPosition(c);
        ProbeCharacterWater(c, centre, out var probe, out _, out float extent);
        if (!probe.Found) { return false; }
        float immersion = probe.Depth / extent;
        var state = Swimming.Next(st.Water, immersion, Swimming.DensityOf(in st) / probe.Density, Airborne(c));
        if (state == CharacterWaterState.Dry) { return false; }
        if (state == CharacterWaterState.Wading)
        {
            float t = immersion >= Swimming.SwimEnter ? 1f : immersion / Swimming.SwimEnter;
            float k = Swimming.WadeFactor(immersion);
            float carry = Swimming.WadeCarry * t * dt;
            displacement = new Vector3F(displacement.X * k + probe.Flow.X * carry, displacement.Y, displacement.Z * k + probe.Flow.Z * carry);
            return false;
        }

        int b = CharBody[c];
        var lin = Core.Bodies.LinearVelocityOf(b);
        float rate = Swimming.DampingOf(in st);
        float blend = rate * dt / (1f + rate * dt);
        var intent = displacement / dt;
        // Horizontal velocity follows the water plus the stroke; the vertical one only when the stroke has a vertical part, so buoyancy and weight decide the rest.
        lin[0] += (probe.Flow.X + intent.X - lin[0]) * blend;
        lin[2] += (probe.Flow.Z + intent.Z - lin[2]) * blend;
        if (intent.Y != 0f) { lin[1] += (probe.Flow.Y + intent.Y - lin[1]) * blend; }
        GravityScale[b] = 1f;
        MarkInAir(c);
        CharVx[c] = lin[0]; CharVy[c] = lin[1]; CharVz[c] = lin[2];
        new Body(World!, Core.HandleOfBody(b)).Wake();
        return true;
    }

    /// <summary>
    /// After the step: each character's water state is looked at again from where the step left it; a change is recorded as an event, in character slot order. A character that is no longer
    /// swimming forgets its swim velocity.
    /// </summary>
    public void StepCharacterWater()
    {
        CharacterWaterEvents.Clear();
        if (CharHandles.Count == 0) { return; }
        bool any = AnyWaterForCharacters();
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            var st = Extensions.At(c);
            if (st.NoSwim) { continue; }
            if (!any && st.Water == CharacterWaterState.Dry) { continue; }
            var centre = CharacterPosition(c);
            ProbeCharacterWater(c, centre, out var probe, out _, out float extent);
            float immersion = probe.Found ? probe.Depth / extent : 0f;
            var next = probe.Found ? Swimming.Next(st.Water, immersion, Swimming.DensityOf(in st) / probe.Density, Airborne(c)) : CharacterWaterState.Dry;
            var prev = st.Water;
            if (next == prev && immersion == st.Immersion) { continue; }
            int gen = CharHandles.GenerationOf(c);
            if (prev == CharacterWaterState.Dry && next != CharacterWaterState.Dry) { CharacterWaterEvents.Add(new CharacterWaterRecord { Kind = CharacterWaterEventKind.EnteredWater, Character = c, Generation = gen, Source = probe.Source, Immersion = immersion }); }
            if (prev != CharacterWaterState.Swimming && next == CharacterWaterState.Swimming) { CharacterWaterEvents.Add(new CharacterWaterRecord { Kind = CharacterWaterEventKind.StartedSwimming, Character = c, Generation = gen, Source = probe.Source, Immersion = immersion }); }
            if (prev == CharacterWaterState.Swimming && next != CharacterWaterState.Swimming) { CharacterWaterEvents.Add(new CharacterWaterRecord { Kind = CharacterWaterEventKind.StoppedSwimming, Character = c, Generation = gen, Source = probe.Source, Immersion = immersion }); }
            if (prev != CharacterWaterState.Dry && next == CharacterWaterState.Dry) { CharacterWaterEvents.Add(new CharacterWaterRecord { Kind = CharacterWaterEventKind.LeftWater, Character = c, Generation = gen, Source = probe.Source, Immersion = immersion }); }
            st.Water = next;
            st.Immersion = immersion;
            if (next != CharacterWaterState.Swimming) { st.SwimVelocity = default; }
            Extensions.Set(c, in st);
        }
    }
}
