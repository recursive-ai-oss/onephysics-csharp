using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>Character water events of the last step, in character slot order (cleared at the start of the next step).</summary>
    internal readonly List<CharacterWaterRecord> CharacterWaterEvents = new();

    private const int ParticleProbeSamples = 12;

    private bool AnyWaterForCharacters()
    {
        if (_water != null && _water.Count > 0) { return true; }
        if (ParticleFluids.FluidHandles.Count > 0) { return true; }
        var defs = Fields.Definitions;
        for (int i = 0; i < defs.Length; i++) { if (defs[i] != null && defs[i]!.Kind == FieldKind.Buoyancy) { return true; } }
        return false;
    }

    /// <summary>The geometry of a character's capsule along the vertical: its lowest point and its height (a 2D capsule always stands along y).</summary>
    private void CharacterVertical(int c, double centreY, out double bottom, out float extent)
    {
        extent = CharDef[c].Height;
        bottom = centreY - 0.5 * extent;
    }

    /// <summary>Looks for the deepest water at a character (shallow water, buoyancy volumes and particle fluids, in that order for ties).</summary>
    private void ProbeCharacterWater(int c, double cx, double cy, out WaterProbe probe, out double bottom, out float extent)
    {
        probe = default;
        CharacterVertical(c, cy, out bottom, out extent);
        if (_water != null && _water.Count > 0) { _water.Probe(cx, 0.0, bottom, extent, ref probe); }
        var defs = Fields.Definitions;
        for (int i = 0; i < defs.Length; i++)
        {
            var d = defs[i];
            if (d == null || d.Kind != FieldKind.Buoyancy) { continue; }
            probe.OfferPlane(CharacterWaterSource.BuoyancyVolume, new Vector3F(d.Axis.X, d.Axis.Y, 0f), d.SurfaceHeight, d.Density, d.Flow, true, d.Minimum, d.Maximum, cx, 0.0, bottom, extent);
        }

        var pf = ParticleFluids;
        if (pf.FluidHandles.Count > 0 && pf.Count > 0)
        {
            pf.PrepareQueries();
            int wet = 0;
            float rest = 0f;
            Vector3F flow = default;
            for (int i = 0; i < ParticleProbeSamples; i++)
            {
                double y = bottom + (i + 0.5) * extent / ParticleProbeSamples;
                float density = pf.SampleFlow(cx, y, 0.0, out var vel, out float r0);
                if (r0 > 0f && density >= 0.5f * r0)
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

    private float SwimGravity() => Math.Max(-Opt.Gravity.Y, 0f);

    /// <summary>
    /// A character's move adjusted for the water it is in: slowed and carried a little while wading; while swimming, the displacement is a swim intent relative to the water and the move that comes
    /// back follows the swim law (current, buoyancy, drag). <paramref name="swimming"/> tells the caller to hand the move's real result back through <see cref="SwimCommit"/>.
    /// </summary>
    public Vector2F SwimAdjust(int c, Vector2F displacement, float dt, out bool swimming)
    {
        swimming = false;
        if (!(dt > 0f) || !float.IsFinite(dt)) { return displacement; }
        var st = Extensions.At(c);
        if (st.NoSwim || (st.Water == CharacterWaterState.Dry && !AnyWaterForCharacters())) { return displacement; }
        CharacterPosition(c, out double cx, out double cy);
        ProbeCharacterWater(c, cx, cy, out var probe, out _, out float extent);
        if (!probe.Found) { return displacement; }
        float immersion = probe.Depth / extent;
        float charDensity = Swimming.DensityOf(in st);
        var state = Swimming.Next(st.Water, immersion, charDensity / probe.Density);
        if (state == CharacterWaterState.Dry) { return displacement; }
        if (state == CharacterWaterState.Wading)
        {
            float t = immersion >= Swimming.SwimEnter ? 1f : immersion / Swimming.SwimEnter;
            float k = Swimming.WadeFactor(immersion);
            float carry = Swimming.WadeCarry * t * dt;
            return new Vector2F(displacement.X * k + probe.Flow.X * carry, displacement.Y);
        }

        float fraction = Swimming.SubmergedFraction(true, CharDef[c].Radius, extent, probe.Depth);
        float g = SwimGravity();
        float up = g > 0f ? g * (probe.Density * fraction / charDensity - 1f) : 0f;
        var v0 = st.Water == CharacterWaterState.Swimming ? st.SwimVelocity : new Vector3F(CharVx[c], CharVy[c], 0f);
        var v = Swimming.Integrate(v0, new Vector3F(displacement.X / dt, displacement.Y / dt, 0f), probe.Flow, up, Swimming.DampingOf(in st), dt, true);
        swimming = true;
        st.SwimVelocity = v;
        Extensions.Set(c, in st);
        return new Vector2F(v.X * dt, v.Y * dt);
    }

    /// <summary>After a swimming move: the swimmer's velocity is what the mover let it do (a wall or the bed stops it).</summary>
    public void SwimCommit(int c, Vector2F moved, float dt)
    {
        var st = Extensions.At(c);
        st.SwimVelocity = new Vector3F(moved.X / dt, moved.Y / dt, 0f);
        Extensions.Set(c, in st);
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
            CharacterPosition(c, out double cx, out double cy);
            ProbeCharacterWater(c, cx, cy, out var probe, out _, out float extent);
            float immersion = probe.Found ? probe.Depth / extent : 0f;
            var next = probe.Found ? Swimming.Next(st.Water, immersion, Swimming.DensityOf(in st) / probe.Density) : CharacterWaterState.Dry;
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
