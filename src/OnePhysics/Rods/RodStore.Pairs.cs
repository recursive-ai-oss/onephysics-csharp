using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    private struct SegmentPair { internal int A,SA,B,SB; }
    private static bool MaterialNeighbours(RodParticles d,int a,int b)
    {
        if(a>b)(a,b)=(b,a);if(b-a<=1)return true;
        double gap=0;
        for(int i=a+1;i<b;i++){gap+=d.Length[i];if(gap>2*d.Material.Radius+1e-7)return false;}
        return true;
    }
    private SegmentPair[] _segmentPairs=new SegmentPair[16];
    private void ConfigurePair(ref Contact c,RodParticles a,RodParticles b,float u,float v,Vector3F normal)
    {
        var velocity=a.VelocityData[c.Segment]*(1-u)+a.VelocityData[c.Segment+1]*u-b.VelocityData[c.OtherSegment]*(1-v)-b.VelocityData[c.OtherSegment+1]*v;
        c.Approach=Math.Min(0,Vector3F.Dot(velocity,normal));
        if(Host.RodCore.ContactModifier is not {} modifier)return;
        Array.Clear(_contactPayload);Array.Clear(_contactExtra);_contactPoints[0]=1;
        _contactPayload[LowLevel.ManifoldLayout.Normal]=-normal.X;_contactPayload[LowLevel.ManifoldLayout.Normal+1]=-normal.Y;_contactPayload[LowLevel.ManifoldLayout.Normal+2]=-normal.Z;
        _contactPayload[LowLevel.ManifoldLayout.Friction]=c.Friction;_contactPayload[LowLevel.ManifoldLayout.Restitution]=c.Restitution;
        var point=a.PositionData[c.Segment]*(1-u)+a.PositionData[c.Segment+1]*u;var other=b.PositionData[c.OtherSegment]*(1-v)+b.PositionData[c.OtherSegment+1]*v;
        float separation=(float)(RodGeometry.Dot(point-other,RodGeometry.Double(normal))-a.Material.Radius-b.Material.Radius);
        _contactPayload[LowLevel.ManifoldLayout.Separation(0)]=separation;_contactExtra[1]=float.MaxValue;_contactExtra[3]=c.Friction;
        var view=new ContactModification(Host.RodCore,c.Rod,c.Segment,c.Other,c.OtherSegment,true,_contactPayload,_contactPoints,_contactExtra);
        modifier(ref view);c.Enabled=view.Enabled;c.Friction=view.Friction;c.Friction2=_contactExtra[3];c.Restitution=view.Restitution;c.SeparationOffset=view.GetSeparation(0)-separation;c.NormalLimit=view.NormalImpulseLimit;c.TangentSpeed=view.TangentSpeed;
    }
    private bool SweepPair(RodParticles a,int sa,RodParticles b,int sb,ref int stops,out float u,out float v,out Vector3F normal)
    {
        var a0=a.Previous[sa];var a1=a.Previous[sa+1];var b0=b.Previous[sb];var b1=b.Previous[sb+1];
        var da0=a.PositionData[sa]-a0;var da1=a.PositionData[sa+1]-a1;var db0=b.PositionData[sb]-b0;var db1=b.PositionData[sb+1]-b1;
        double speed=Math.Max(RodGeometry.Length(da0),RodGeometry.Length(da1))+Math.Max(RodGeometry.Length(db0),RodGeometry.Length(db1));
        double radius=a.Material.Radius+b.Material.Radius,skin=Math.Max(Skin(a),Skin(b));float time=0;u=v=0;normal=default;
        for(int iteration=0;iteration<64;iteration++)
        {
            RodGeometry.Closest(a0+da0*time,a1+da1*time,b0+db0*time,b1+db1*time,out u,out v,out var x,out var y);
            var d=x-y;double distance=RodGeometry.Length(d);
            if(distance>1e-12)normal=RodGeometry.Float(d/distance);
            if(distance-radius<=skin)
            {
                if(normal==default)
                {
                    RodGeometry.Closest(a0,a1,b0,b1,out _,out _,out x,out y);d=x-y;distance=RodGeometry.Length(d);
                    if(distance>1e-12)normal=RodGeometry.Float(d/distance);
                    else
                    {
                        var tangent=RodGeometry.Float(a1-a0);
                        normal=Host.Planar?new Vector3F(tangent.Y,-tangent.X,0):RodParticles.Cross(tangent,Math.Abs(tangent.X)<.9f?new Vector3F(1,0,0):new Vector3F(0,1,0));
                        float length=(float)Math.Sqrt(normal.LengthSquared());normal=length>1e-12f?normal/length:new Vector3F(0,1,0);
                    }
                }
                return true;
            }
            if(!(speed>1e-15))return false;
            double advance=.8*(distance-radius-skin)/speed;if(time+advance>1)return false;
            if(advance<1e-8)return true;time+=(float)advance;
        }
        stops++;return normal!=default;
    }
    private void PairVelocity(ref Contact c,float h)
    {
        var a=Data[c.Rod]!;var b=Data[c.Other]!;float u=c.U,v=c.V;
        var velocity=a.VelocityData[c.Segment]*(1-u)+a.VelocityData[c.Segment+1]*u-b.VelocityData[c.OtherSegment]*(1-v)-b.VelocityData[c.OtherSegment+1]*v;
        var leverA=c.Normal*(-a.Material.Radius);var leverB=c.Normal*b.Material.Radius;
        velocity+=RodParticles.Cross(a.AngularVelocity[c.Segment],leverA)-RodParticles.Cross(b.AngularVelocity[c.OtherSegment],leverB);
        float weight=(1-u)*(1-u)*a.InverseMass[c.Segment]+u*u*a.InverseMass[c.Segment+1]+(1-v)*(1-v)*b.InverseMass[c.OtherSegment]+v*v*b.InverseMass[c.OtherSegment+1];
        if(!(weight>0))return;float vn=Vector3F.Dot(velocity,c.Normal);
        float normal=Math.Min(Math.Max(0,(-c.Restitution*c.Approach-vn)/weight),Math.Max(0,c.NormalLimit-c.NormalLambda/h));var impulse=c.Normal*normal;
        SurfaceContacts.Tangents(-c.Normal,Host.Planar?2:3,default,out var t1,out _);
        var tangent=velocity-c.Normal*vn+t1*c.TangentSpeed;float speed=(float)Math.Sqrt(tangent.LengthSquared());
        if(speed>1e-12f)
        {
            var axis=tangent/speed;var ca=RodParticles.Cross(leverA,axis);var cb=RodParticles.Cross(leverB,axis);
            float tw=weight+Vector3F.Dot(ca,a.InverseAngularImpulse(c.Segment,ca))+Vector3F.Dot(cb,b.InverseAngularImpulse(c.OtherSegment,cb));
            impulse-=axis*Math.Min(speed/tw,c.Friction*(normal+c.NormalLambda/h));
        }
        a.VelocityData[c.Segment]+=impulse*((1-u)*a.InverseMass[c.Segment]);a.VelocityData[c.Segment+1]+=impulse*(u*a.InverseMass[c.Segment+1]);
        b.VelocityData[c.OtherSegment]-=impulse*((1-v)*b.InverseMass[c.OtherSegment]);b.VelocityData[c.OtherSegment+1]-=impulse*(v*b.InverseMass[c.OtherSegment+1]);
        a.AngularVelocity[c.Segment]+=a.InverseAngularImpulse(c.Segment,RodParticles.Cross(leverA,impulse));b.AngularVelocity[c.OtherSegment]+=b.InverseAngularImpulse(c.OtherSegment,RodParticles.Cross(leverB,-impulse));
    }
}
