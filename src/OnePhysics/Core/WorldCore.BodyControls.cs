using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    internal BodyControlStore Controls { get; }
    /// <summary>Gets optional axis locks and speed caps; default inherits world limits. Validates the handle.</summary>
    public BodyControlSettings GetBodyControls(BodyHandle body) => Controls.Get(Resolve(body)).Settings;
    /// <summary>Sets optional axis locks and caps, waking the body. Call between steps. Dimension-inapplicable axes throw ArgumentOutOfRangeException. FreezeAll is accepted in 2D.</summary>
    public void SetBodyControls(BodyHandle body, BodyControlSettings settings)
    {
        int b = Resolve(body); BodyControlStore.ValidateAxes(settings, Layout.PositionLanes);
        var s = Controls.Get(b); s.Settings = settings; Controls.Set(b, s); Wake(body);
        ProjectBodyVelocity(b);
    }
    internal float AxisMass(int b, int axis) => ((int)Controls.ConstraintsOf(b) & (1 << axis)) != 0 ? 0 : Bodies.InverseMass[b];
    internal void ProjectBodyVelocity(int b) => ProjectBodyVelocity(b, Bodies.LinearVelocityOf(b), Bodies.AngularVelocityOf(b));
    /// <summary>Clears the locked axes of a body's velocity given as its linear and angular lanes, for an engine that keeps the velocity outside the body store while it solves.</summary>
    internal void ProjectBodyVelocity(int b, Span<float> v, Span<float> w)
    {
        int mask = (int)Controls.ConstraintsOf(b);
        for (int i = 0; i < v.Length; i++) if ((mask & (1 << i)) != 0) v[i] = 0;
        for (int i = 0; i < w.Length; i++) if ((mask & (1 << (Layout.PositionLanes == 2 ? 5 : 3+i))) != 0) w[i] = 0;
    }
}
