namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Shape
{
    /// <summary>Body-frame surface behavior; default is stationary, two-sided and isotropic. Changes apply next step and are recorded. Throws InvalidHandleException for a stale shape or ArgumentOutOfRangeException for invalid properties.</summary>
    public SurfaceProperties Surface
    {
        get { Resolve(); return _world!.Core.GetSurfaceProperties(_handle); }
        set { using var mutation = _world?.RecordMutation("Shape.Surface"); Resolve(); _world!.Core.SetSurfaceProperties(_handle, value); }
    }
}
