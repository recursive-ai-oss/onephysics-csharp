namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Body
{
    /// <summary>Reads island, sleep and constraint information without allocating. Throws for a default or stale handle. Read between steps only.</summary>
    public BodyInspection Inspect() { var (w, b) = Resolve(); return new(w.Core.InspectBody(RawHandle)); }
}
