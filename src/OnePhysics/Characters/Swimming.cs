using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Characters;

/// <summary>
/// The water a character stands in and the swim model, shared by both engines. A swimmer's velocity relaxes towards the water's velocity plus its own swim intent at rate k (a linear drag law), while buoyancy
/// minus weight accelerates it vertically: the velocity v obeys dv/dt = a_y + k (flow + intent - v), so an idle swimmer drifts at the water's velocity and sinks or rises at the terminal speed a_y / k relative to it.
/// </summary>
internal static class Swimming
{
    /// <summary>Immersion (fraction of the height under water) at which a dry character counts as in the water, and at which it counts as dry again.</summary>
    public const float EnterWet = 0.05f, ExitWet = 0.02f;

    /// <summary>Immersion at which a wader starts to swim, and below which a swimmer stops (both lowered for a character light enough to float on less water).</summary>
    public const float SwimEnter = 0.75f, SwimExit = 0.6f;

    /// <summary>Fraction of walking speed lost at the depth where swimming starts; less in shallower water.</summary>
    public const float WadeSlowdown = 0.5f;

    /// <summary>Fraction of the current that carries a wader at the depth where swimming starts.</summary>
    public const float WadeCarry = 0.5f;

    /// <summary>A person's average density, kg/m³ (a little less than water's: lungs).</summary>
    public const float DefaultDensity = 900f;

    /// <summary>Default rate, per second, at which a swimmer's velocity relaxes towards the water's plus its intent.</summary>
    public const float DefaultDamping = 4f;

    /// <summary>Volume (area in 2D) of an upright capsule: its cylinder and its two caps.</summary>
    public static float CapsuleVolume(bool planar, float radius, float height)
    {
        double r = radius, l = Math.Max(height - 2.0 * radius, 0.0);
        return (float)(planar ? 2.0 * r * l + Math.PI * r * r : Math.PI * r * r * l + 4.0 / 3.0 * Math.PI * r * r * r);
    }

    public static float DensityOf(in CharacterExtState s) => s.SwimDensity > 0f ? s.SwimDensity : DefaultDensity;

    public static float DampingOf(in CharacterExtState s) => s.SwimDamping > 0f ? s.SwimDamping : DefaultDamping;

    /// <summary>The fraction of an upright capsule's volume (area in 2D) under a surface <paramref name="depth"/> above its lowest point: exact, from the caps and the cylinder (or the circular segments in 2D).</summary>
    public static float SubmergedFraction(bool planar, float radius, float height, float depth)
    {
        if (!(depth > 0f)) { return 0f; }
        if (depth >= height) { return 1f; }
        double r = radius, h = height, d = depth, l = Math.Max(h - 2.0 * r, 0.0);
        double total, wet;
        if (planar)
        {
            total = 2.0 * r * l + Math.PI * r * r;
            if (d <= r) { wet = Segment(r, d); }
            else if (d <= r + l) { wet = 0.5 * Math.PI * r * r + 2.0 * r * (d - r); }
            else { wet = total - Segment(r, h - d); }
        }
        else
        {
            total = Math.PI * r * r * l + 4.0 / 3.0 * Math.PI * r * r * r;
            if (d <= r) { wet = Cap(r, d); }
            else if (d <= r + l) { wet = 2.0 / 3.0 * Math.PI * r * r * r + Math.PI * r * r * (d - r); }
            else { wet = total - Cap(r, h - d); }
        }

        double f = wet / total;
        return f < 0.0 ? 0f : f > 1.0 ? 1f : (float)f;
    }

    // Volume of a sphere cap of height e (Archimedes).
    private static double Cap(double r, double e) => Math.PI * e * e * (3.0 * r - e) / 3.0;

    // Area of a circular segment of height e.
    private static double Segment(double r, double e)
    {
        if (e <= 0.0) { return 0.0; }
        if (e >= 2.0 * r) { return Math.PI * r * r; }
        double c = (r - e) / r;
        c = c < -1.0 ? -1.0 : c > 1.0 ? 1.0 : c;
        double s = 2.0 * r * e - e * e;
        return r * r * Math64.Acos(c) - (r - e) * Math.Sqrt(s < 0.0 ? 0.0 : s);
    }

    /// <summary>Immersion at which a character that has nothing under its feet and is not rising starts to swim: it is in water too deep to walk in.</summary>
    public const float AirborneEnter = 0.35f;

    /// <summary>Immersion below which a swimmer with nothing under its feet stops swimming (it has left the water).</summary>
    public const float AirborneExit = 0.2f;

    /// <summary>
    /// The state after a look at the water, with hysteresis; <paramref name="ratio"/> is the character's density over the water's (a light character floats on less water). <paramref name="airborne"/>: nothing is under
    /// the character's feet and it is not rising, so water over its knees is water it falls into, not water it wades in.
    /// </summary>
    public static CharacterWaterState Next(CharacterWaterState previous, float immersion, float ratio, bool airborne = false)
    {
        float enter = Math.Min(SwimEnter, 0.9f * ratio), exit = Math.Min(SwimExit, 0.7f * ratio);
        if (airborne) { enter = Math.Min(enter, AirborneEnter); exit = Math.Min(exit, AirborneExit); }
        if (enter < EnterWet) { enter = EnterWet; }
        if (exit < ExitWet) { exit = ExitWet; }
        switch (previous)
        {
            case CharacterWaterState.Swimming:
                if (immersion >= exit) { return CharacterWaterState.Swimming; }
                return immersion < ExitWet ? CharacterWaterState.Dry : CharacterWaterState.Wading;
            case CharacterWaterState.Wading:
                if (immersion < ExitWet) { return CharacterWaterState.Dry; }
                return immersion >= enter ? CharacterWaterState.Swimming : CharacterWaterState.Wading;
            default:
                if (immersion < EnterWet) { return CharacterWaterState.Dry; }
                return immersion >= enter ? CharacterWaterState.Swimming : CharacterWaterState.Wading;
        }
    }

    /// <summary>One implicit step of the swim law (see the class remarks); planar swimmers keep z = 0.</summary>
    public static Vector3F Integrate(Vector3F v, Vector3F intent, Vector3F flow, float upwardAcceleration, float rate, float dt, bool planar)
    {
        float inv = 1f / (1f + rate * dt), rk = rate * dt;
        float vx = (v.X + rk * (flow.X + intent.X)) * inv;
        float vy = (v.Y + dt * upwardAcceleration + rk * (flow.Y + intent.Y)) * inv;
        float vz = planar ? 0f : (v.Z + rk * (flow.Z + intent.Z)) * inv;
        return new Vector3F(vx, vy, vz);
    }

    /// <summary>The horizontal slowdown factor of walking in water of the given immersion.</summary>
    public static float WadeFactor(float immersion)
    {
        float t = immersion >= SwimEnter ? 1f : immersion / SwimEnter;
        return 1f - WadeSlowdown * t;
    }
}

/// <summary>The deepest water found at a character: the surface above the capsule's lowest point, and what the water does there.</summary>
internal struct WaterProbe
{
    public bool Found;
    public CharacterWaterSource Source;

    /// <summary>Water density, kg/m³ (kg/m² in a planar world, per metre of depth).</summary>
    public float Density;

    /// <summary>Velocity of the water at the character.</summary>
    public Vector3F Flow;

    /// <summary>Depth of the water over the capsule's lowest point, clamped to the capsule's height.</summary>
    public float Depth;

    /// <summary>Offers a horizontal-ish water plane (normal dot position = offset) with an optional region; the deepest water wins and the first one offered wins a tie.</summary>
    public void OfferPlane(CharacterWaterSource source, Vector3F normal, double offset, float density, Vector3F flow, bool limited, Vector3D min, Vector3D max, double x, double z, double bottom, float height)
    {
        if (!(normal.Y > 0.5f)) { return; }
        if (limited && (x < min.X || x > max.X || z < min.Z || z > max.Z || bottom > max.Y)) { return; }
        double surface = (offset - normal.X * x - normal.Z * z) / normal.Y;
        Offer(source, surface - bottom, density, flow, height);
    }

    /// <summary>Offers water whose depth over the capsule's lowest point is known.</summary>
    public void Offer(CharacterWaterSource source, double depth, float density, Vector3F flow, float height)
    {
        if (!(depth > 0.0)) { return; }
        float d = depth >= height ? height : (float)depth;
        if (Found && !(d > Depth)) { return; }
        Found = true; Source = source; Density = density; Flow = flow; Depth = d;
    }
}

/// <summary>One character water event as an engine collects it.</summary>
internal struct CharacterWaterRecord
{
    public CharacterWaterEventKind Kind;
    public int Character, Generation;
    public CharacterWaterSource Source;
    public float Immersion;
}

