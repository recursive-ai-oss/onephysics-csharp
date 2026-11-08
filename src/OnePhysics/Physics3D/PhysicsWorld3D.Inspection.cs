namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    private void UpdateInspectionCounters()
    {
        Counters.Characters = _engine.CharHandles.Count;
        Counters.Vehicles = _engine.VehHandles.Count;
    }

    /// <summary>Reads body diagnostics. Throws for a default, stale or foreign body. Read between steps.</summary>
    public BodyInspection InspectBody(Body body) => new(_core.InspectBody(body.RawHandle));

    /// <summary>Always-on counters of the last step; reused on the next step. Not simulation or snapshot state.</summary>
    public StepCounters Counters => _core.Counters;

    /// <summary>Per-stage timestamp differences. Set its TimestampSource between steps to enable timing; null by default. Not simulation or snapshot state.</summary>
    public StepProfile Profile => _core.Profile;
}
