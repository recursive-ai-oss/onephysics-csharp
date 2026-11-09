using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Body
{
    /// <summary>World-oriented magnetic dipole moment; zero by default. Stale handles throw. Moment units are normalized by the magnetism field Strength.</summary>
    public Vector3F MagneticMoment
    {
        get {var (world,b)=Resolve();var moment=world.Engine.Fields.Moment(b);return new(moment.X,moment.Y,moment.Z);}
        set {var (world,_)=Resolve();world.SetMagneticMoment(RawHandle,value);}
    }
}
