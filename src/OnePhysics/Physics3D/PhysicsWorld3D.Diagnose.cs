using RecursiveAI.OnePhysics.Diagnosis;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Looks the world over for what is wrong with it and returns findings: each has a stable code, a severity, the bodies and joints involved, the measured values, a plain-language explanation and a suggested change.
    /// The catalogue (codes, thresholds and the evidence for them) is in docs/spec/diagnosis.md. The current state is examined first (settings, sizes, masses, anchors, speeds, contacts); then, unless
    /// <see cref="DiagnoseOptions.ProbeSteps"/> is 0 or every body is asleep, a copy of the world is stepped <see cref="DiagnoseOptions.ProbeSteps"/> times so that what needs time can be judged (convergence,
    /// motors and limits, bodies that never sleep, tunnelling). This world is never stepped or changed: its snapshot bytes and hash are the same afterwards. The copy has no event handlers, pair filter or custom-field callbacks.
    /// Cost: one structural copy plus the probe steps. The result is the same on every platform. Call between steps; allocates.
    /// </summary>
    /// <param name="options">Settings; null means the defaults (90 probe steps of 1/60 s).</param>
    /// <exception cref="System.ArgumentOutOfRangeException">An option is out of range.</exception>
    public DiagnosisReport Diagnose(DiagnoseOptions? options = null) => DiagnosisRunner.Live(new Host3D(this), options);
}
