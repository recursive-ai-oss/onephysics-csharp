using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IFieldHost
{
    private FieldStore? _fields;
    internal FieldStore Fields => _fields ??= new FieldStore(this);
    FieldStore IFieldHost.Fields => Fields;
    WorldCore IFieldHost.FieldCore => Core;
    int IFieldHost.EngineSectionId => Id;
    Vector3F IFieldHost.AngularTorque(int b,Vector3F alpha) => default;
    Vector3F IFieldHost.Gravity => new(Opt.Gravity.X,Opt.Gravity.Y,0);
    float IFieldHost.GravityScaleOf(int b) => GravityScale[b];
    ReplayMutationScope IFieldHost.FieldMutation(string name) => World!.RecordMutation(name);
    bool IFieldHost.Matches(int b,FieldDefinition d)
    {
        bool layer=false;
        for(int s=ShapeHead[b];s>=0;s=NextShape[s]) if(!IsSensor(s)&&((d.Layers>>ShapeLayer[s])&1u)!=0) {layer=true;break;}
        if(!layer)return false;
        if(d.Sensor<0)return true;
        if(!Core.Shapes.Handles.IsAlive(d.Sensor,d.SensorGeneration)||!IsSensor(d.Sensor))return false;
        for(int p=0;p<PairHigh;p++)
        {
            var pair=Pairs[p];
            if(!pair.Live||pair.Cons!=SensorOverlap)continue;
            if((pair.A==d.Sensor&&Core.Shapes.Owner[pair.B]==b)||(pair.B==d.Sensor&&Core.Shapes.Owner[pair.A]==b))return true;
        }
        return false;
    }
    void IFieldHost.AddForce(int b,Vector3F force,Vector3F torque)
    {
        Force[2*b]+=force.X;Force[2*b+1]+=force.Y;
        Torque[b]+=torque.Z;
    }
    void IFieldHost.Buoyancy(int b,FieldDefinition d,float dt) { ApplyFieldBuoyancy(b,d,dt); }
}
