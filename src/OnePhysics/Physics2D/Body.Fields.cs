using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Buoyant lift multiplier, default 1, finite 0..1e6. Zero disables lift but not drag. Setting wakes the body; stale handles throw.</summary>
    public float BuoyancyFactor
    {
        get {var (world,b)=Resolve();return world.Engine.Fields.BuoyancyFactor(b);}
        set
        {
            var (world,b)=Resolve();
            if(!(value>=0&&value<=1e6f))throw new System.ArgumentOutOfRangeException(nameof(value),value,"BuoyancyFactor must be within 0..1e6.");
            using var mutation=world.RecordMutation("Body.BuoyancyFactor");world.Engine.Fields.SetBuoyancyFactor(b,value);WakeBody(world,b);
        }
    }
    /// <summary>World-oriented magnetic dipole moment; zero by default. Stale handles throw. Moment units are normalized by the magnetism field Strength.</summary>
    public Vector2F MagneticMoment
    {
        get {var (world,b)=Resolve();var moment=world.Engine.Fields.Moment(b);return new(moment.X,moment.Y);}
        set {var (world,_)=Resolve();world.SetMagneticMoment(RawHandle,value);}
    }
}
