using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A character entered or left water, or started or stopped swimming (see <see cref="PhysicsWorld3D.CharacterWaterEvents"/>).</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct CharacterWaterEvent
{
    internal CharacterWaterEvent(CharacterWaterEventKind kind, Character character, CharacterWaterSource source, float immersion) { Kind = kind; Character = character; Source = source; Immersion = immersion; }

    /// <summary>What happened.</summary>
    public CharacterWaterEventKind Kind { get; }

    /// <summary>The character.</summary>
    public Character Character { get; }

    /// <summary>The kind of water the character is in (for a leave event, the water it last touched).</summary>
    public CharacterWaterSource Source { get; }

    /// <summary>The fraction of the character's height under water after the step.</summary>
    public float Immersion { get; }
}

public readonly partial struct Character
{
    /// <summary>Whether the character is dry, wading or swimming, as of the end of the last step. Characters with <see cref="CharacterDef.CanSwim"/> false are always dry.</summary>
    public CharacterWaterState WaterState { get { var e = Resolve(out int c); return e.Extensions.At(c).Water; } }

    /// <summary>True while <see cref="WaterState"/> is <see cref="CharacterWaterState.Swimming"/>. While it is, the displacement given to <see cref="Move(Vector3F, float)"/> is a swim intent relative to the water (not a walking step and not gravity): the engine adds the current, buoyancy and drag.</summary>
    public bool IsSwimming => WaterState == CharacterWaterState.Swimming;

    /// <summary>The fraction of the character's height under water at the end of the last step, 0 to 1.</summary>
    public float Immersion { get { var e = Resolve(out int c); return e.Extensions.At(c).Immersion; } }

    /// <summary>A swimming virtual character's velocity (zero when not swimming); a rigid character's is its body's velocity.</summary>
    public Vector3F SwimVelocity { get { var e = Resolve(out int c); var v = e.Extensions.At(c).SwimVelocity; return new Vector3F(v.X, v.Y, v.Z); } }

    /// <summary>Whether water changes how the character moves; see <see cref="CharacterDef.CanSwim"/>. Turning it off makes the character dry again at the next step.</summary>
    public bool CanSwim
    {
        get { var e = Resolve(out int c); return !e.Extensions.At(c).NoSwim; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Character.CanSwim");
            var e = Resolve(out int c);
            var st = e.Extensions.At(c);
            if (st.NoSwim == !value) { return; }
            st.NoSwim = !value;
            if (!value) { st.Water = CharacterWaterState.Dry; st.Immersion = 0f; st.SwimVelocity = default; }
            e.Extensions.Set(c, in st);
        }
    }

    /// <summary>The density the character swims with, kg/m³ (kg/m² in 2D); see <see cref="CharacterDef.SwimDensity"/>. Setting it to 0 restores the default: 900 for a virtual character, the real density (mass over capsule volume) for a rigid one.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is not within [0, 100000].</exception>
    public float SwimDensity
    {
        get { var e = Resolve(out int c); return e.SwimDensityOf(c, e.Extensions.At(c)); }
        set
        {
            if (!(value >= 0f && value <= 1e5f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "SwimDensity must be within [0, 100000]."); }
            using var replayMutation = _world?.RecordMutation("Character.SwimDensity");
            var e = Resolve(out int c);
            var st = e.Extensions.At(c);
            st.SwimDensity = value;
            e.Extensions.Set(c, in st);
        }
    }

    /// <summary>The swim relaxation rate, per second; see <see cref="CharacterDef.SwimDamping"/>. Setting it to 0 restores the default.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is not within [0, 1000].</exception>
    public float SwimDamping
    {
        get { var e = Resolve(out int c); return Swimming.DampingOf(e.Extensions.At(c)); }
        set
        {
            if (!(value >= 0f && value <= 1e3f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "SwimDamping must be within [0, 1000]."); }
            using var replayMutation = _world?.RecordMutation("Character.SwimDamping");
            var e = Resolve(out int c);
            var st = e.Extensions.At(c);
            st.SwimDamping = value;
            e.Extensions.Set(c, in st);
        }
    }
}

public sealed partial class PhysicsWorld3D
{
    private CharacterWaterEvent[] _characterWaterScratch = Array.Empty<CharacterWaterEvent>();

    /// <summary>The character water events of the last step (entered and left water, started and stopped swimming), in character slot order; valid until the next step, rebuilt on each read.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public ReadOnlySpan<CharacterWaterEvent> CharacterWaterEvents
    {
        get
        {
            var ev = _engine.CharacterWaterEvents;
            if (ev.Count == 0) { return default; }
            if (_characterWaterScratch.Length < ev.Count) { _characterWaterScratch = new CharacterWaterEvent[Math.Max(ev.Count, 2 * _characterWaterScratch.Length)]; }
            for (int i = 0; i < ev.Count; i++)
            {
                var e = ev[i];
                _characterWaterScratch[i] = new CharacterWaterEvent(e.Kind, new Character(this, e.Character, e.Generation), e.Source, e.Immersion);
            }

            return _characterWaterScratch.AsSpan(0, ev.Count);
        }
    }

    /// <summary>Raised after a step for each character water event (unless callbacks are suppressed), in the order of <see cref="CharacterWaterEvents"/>.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public event Action<CharacterWaterEvent>? CharacterWaterChanged;

    private void DispatchCharacterWaterEvents()
    {
        var h = CharacterWaterChanged;
        if (h == null) { return; }
        var events = CharacterWaterEvents.ToArray();
        for (int i = 0; i < events.Length; i++) { h(events[i]); }
    }
}
