using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IRodHost
{
    private RodStore? _rods;
    internal RodStore Rods => _rods ??= new RodStore(this);
    RodStore IRodHost.Rods => Rods;
    WorldCore IRodHost.RodCore => Core;
    int IRodHost.EngineSectionId => Id;
    bool IRodHost.Planar => false;
    Vector3F IRodHost.Gravity => new(Opt.Gravity.X,Opt.Gravity.Y,Opt.Gravity.Z);
    Vector3F IRodHost.LocalCenter(int b) => LocalCenterOf(b);
    float IRodHost.BodyExtent(int b) => MaxExtent[b];
    void IRodHost.MovedBody(int b)
    {
        for(int s=ShapeHead[b];s>=0;s=NextShape[s])
            Bp.MoveProxy(Proxy[s],TightBox(b,s),0.0,0.0,0.0);
    }
    bool IRodHost.Layers(int a,int b)=>((LayerMatrix[a]>>b)&1u)!=0;
    bool IRodHost.Matches(RodParticles rod,int s)
    {
        if((ShapeFlags[s]&FlagSensor)!=0||((LayerMatrix[rod.Layer]>>ShapeLayer[s])&1u)==0)return false;
        if(Group[s]==rod.Group&&rod.Group!=0)return rod.Group>0;
        return (Category[s]&rod.Mask)!=0&&(Mask[s]&rod.Category)!=0;
    }
    ContactMaterial IRodHost.Mix(int s,int subshape,float friction,float restitution)
    {
        var material=Core.ContactMaterials.Resolve(s,subshape,new ContactMaterial(Friction[s],Restitution[s],Rolling[s]));
        if(Opt.MaterialMixer is {} mixer)
        { var mixed=mixer(new Material3D(material.Friction,material.Restitution,material.RollingResistance),new Material3D(friction,restitution,0));return new(mixed.Friction,mixed.Restitution,mixed.RollingResistance); }
        return new((float)System.Math.Sqrt(material.Friction*friction),System.Math.Max(material.Restitution,restitution),material.RollingResistance);
    }
    void IRodHost.Bounds(int s,out Vector3D minimum,out Vector3D maximum)
    {
        int b=Core.Shapes.Owner[s];var box=TightBox(b,s);
        minimum=new(box.MinX,box.MinY,box.MinZ);maximum=new(box.MaxX,box.MaxY,box.MaxZ);
    }
    ReplayMutationScope IRodHost.RodMutation(string name) => World!.RecordMutation(name);
}
