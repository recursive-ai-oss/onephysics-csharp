using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Applies a radial impulse in N s to simulated bodies within radius metres, in body-slot order.
    /// Default constant falloff; inverse laws soften at 0.1 m. When occluded (default), five static-geometry rays sample
    /// the centre and four points at 1/4 body extent on perpendicular axes. Impulse is multiplied by the visible fraction.
    /// Sensors and moving shapes do not occlude. Centre-coincident bodies receive +Y impulse.
    /// Throws ArgumentOutOfRangeException for invalid centre, strength or radius; negative strength attracts.
    /// </summary>
    public void Explode(Vector3D center,float impulse,float radius,bool occluded=true,FieldFalloff falloff=FieldFalloff.Constant,uint layers=uint.MaxValue)
    {
        var point=center;
        FieldDefinition.CheckPoint(point,nameof(center));FieldDefinition.Check(impulse,nameof(impulse));FieldDefinition.Positive(radius,nameof(radius));
        var definition=new FieldDefinition {Strength=impulse,Radius=radius,Falloff=falloff,Layers=layers};definition.Validate();
        using var mutation=RecordMutation("World.Explode");
        var bodies=_core.Bodies;
        var filter=new QueryFilter3D {Predicate=static shape=>shape.Body.IsStatic};
        for(int b=0;b<bodies.HighWater;b++)
        {
            if(!bodies.IsLive(b)||!bodies.IsDynamic(b)||!_engine.Fields.Matches(b,definition))continue;
            var p=bodies.PositionOf(b);var target=new Vector3D(p[0],p[1],p[2]);
            var delta=target-center;float length=(float)Math.Sqrt(delta.X*delta.X+delta.Y*delta.Y+delta.Z*delta.Z);
            if(length>radius)continue;
            var direction=length>0?new Vector3F((float)(delta.X/length),(float)(delta.Y/length),(float)(delta.Z/length)):new Vector3F(0,1,0);
            float visible=1;
            if(occluded&&length>0)
            {
                visible=0;float extent=_engine.MaxExtent[b]*0.25f;
                for(int ray=0;ray<5;ray++)
                {
                    var sample=target;
                    if(ray==1)sample+=new Vector3D(extent,0,0);
                    if(ray==2)sample-=new Vector3D(extent,0,0);
                    if(ray==3)sample+=new Vector3D(0,extent,0);
                    if(ray==4)sample-=new Vector3D(0,extent,0);
                    var r=sample-center;float distance=(float)Math.Sqrt(r.X*r.X+r.Y*r.Y+r.Z*r.Z);
                    if(distance==0||!Raycast(center,new Vector3F((float)r.X,(float)r.Y,(float)r.Z),out _,distance,in filter))visible+=0.2f;
                }
            }
            new Body(this,_core.HandleOfBody(b)).ApplyImpulse(direction*(FieldStore.Falloff(definition,length)*visible));
        }
    }
}
