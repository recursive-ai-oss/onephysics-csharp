using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IFieldHost
{
    private FieldStore? _fields;
    internal FieldStore Fields => _fields ??= new FieldStore(this);
    FieldStore IFieldHost.Fields => Fields;
    WorldCore IFieldHost.FieldCore => Core;
    int IFieldHost.EngineSectionId => Id;
    Vector3F IFieldHost.AngularTorque(int b,Vector3F alpha) => FieldAngularTorque(b,alpha);
    Vector3F IFieldHost.Gravity => Opt.Gravity;
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
            if(!PairLive[p]||PairCons[p]!=SensorOverlap)continue;
            if((PairA[p]==d.Sensor&&Core.Shapes.Owner[PairB[p]]==b)||(PairB[p]==d.Sensor&&Core.Shapes.Owner[PairA[p]]==b))return true;
        }
        return false;
    }
    void IFieldHost.AddForce(int b,Vector3F force,Vector3F torque)
    {
        Force[3*b]+=force.X;Force[3*b+1]+=force.Y;Force[3*b+2]+=force.Z;
        Torque[3*b]+=torque.X;Torque[3*b+1]+=torque.Y;Torque[3*b+2]+=torque.Z;
    }
    void IFieldHost.Buoyancy(int b,FieldDefinition d,float dt) => ApplyFieldBuoyancy(b,d,dt);
}
