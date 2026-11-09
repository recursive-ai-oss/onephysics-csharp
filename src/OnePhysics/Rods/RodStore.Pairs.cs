using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    private struct SegmentBox
    {
        internal int Rod,Segment;
        internal double MinX,MinY,MinZ,MaxX,MaxY,MaxZ;
    }
    private struct SegmentPair { internal int A,SA,B,SB; }
    private readonly struct BoxOrder:IComparer<SegmentBox>
    {
        public int Compare(SegmentBox a,SegmentBox b)
        {int c=a.MinX.CompareTo(b.MinX);if(c!=0)return c;c=a.Rod.CompareTo(b.Rod);return c!=0?c:a.Segment.CompareTo(b.Segment);}
    }
    private readonly struct PairOrder:IComparer<SegmentPair>
    {
        public int Compare(SegmentPair a,SegmentPair b)
        {int c=a.A.CompareTo(b.A);if(c!=0)return c;c=a.SA.CompareTo(b.SA);if(c!=0)return c;c=a.B.CompareTo(b.B);return c!=0?c:a.SB.CompareTo(b.SB);}
    }
    private static bool MaterialNeighbours(RodParticles d,int a,int b)
    {
        if(a>b)(a,b)=(b,a);if(b-a<=1)return true;
        double gap=0;
        for(int i=a+1;i<b;i++){gap+=d.Length[i];if(gap>2*d.Material.Radius+1e-7)return false;}
        return true;
    }
    private static void HeapSort<T,TOrder>(Span<T> values,TOrder order) where TOrder:struct,IComparer<T>
    {
        for(int root=values.Length/2-1;root>=0;root--)Sift(values,root,values.Length,order);
        for(int end=values.Length-1;end>0;end--){(values[0],values[end])=(values[end],values[0]);Sift(values,0,end,order);}
    }
    private static void Sift<T,TOrder>(Span<T> values,int root,int count,TOrder order) where TOrder:struct,IComparer<T>
    {
        while(root<count/2)
        {
            int child=2*root+1;if(child+1<count&&order.Compare(values[child],values[child+1])<0)child++;
            if(order.Compare(values[root],values[child])>=0)return;
            (values[root],values[child])=(values[child],values[root]);root=child;
        }
    }
    private SegmentBox[] _segmentBoxes=new SegmentBox[16];
    private SegmentPair[] _segmentPairs=new SegmentPair[16];
    private void PairContacts(float h)
    {
        int count=0,pairs=0;
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];if(d==null||!d.CollisionEnabled)continue;
            for(int s=0;s<d.Length.Length;s++)
            {
                if(count==_segmentBoxes.Length)Array.Resize(ref _segmentBoxes,_segmentBoxes.Length*2);
                var a=d.Previous[s];var b=d.Previous[s+1];var c=d.PositionData[s];var e=d.PositionData[s+1];double radius=d.Material.Radius+Skin(d);
                _segmentBoxes[count++]=new SegmentBox{Rod=r,Segment=s,MinX=Math.Min(Math.Min(a.X,b.X),Math.Min(c.X,e.X))-radius,MaxX=Math.Max(Math.Max(a.X,b.X),Math.Max(c.X,e.X))+radius,
                    MinY=Math.Min(Math.Min(a.Y,b.Y),Math.Min(c.Y,e.Y))-radius,MaxY=Math.Max(Math.Max(a.Y,b.Y),Math.Max(c.Y,e.Y))+radius,MinZ=Math.Min(Math.Min(a.Z,b.Z),Math.Min(c.Z,e.Z))-radius,MaxZ=Math.Max(Math.Max(a.Z,b.Z),Math.Max(c.Z,e.Z))+radius};
            }
        }
        HeapSort(_segmentBoxes.AsSpan(0,count),new BoxOrder());
        for(int i=0;i<count;i++)
        {
            var a=_segmentBoxes[i];var da=Data[a.Rod]!;
            for(int j=i+1;j<count&&_segmentBoxes[j].MinX<=a.MaxX;j++)
            {
                var b=_segmentBoxes[j];if(a.MinY>b.MaxY||b.MinY>a.MaxY||a.MinZ>b.MaxZ||b.MinZ>a.MaxZ)continue;
                var db=Data[b.Rod]!;
                if(a.Rod==b.Rod){if(!da.SelfCollision||MaterialNeighbours(da,a.Segment,b.Segment))continue;}
                else
                {
                    if(!Host.Layers(da.Layer,db.Layer))continue;
                    if(da.Group==db.Group&&da.Group!=0){if(da.Group<0)continue;}
                    else if((da.Category&db.Mask)==0||(db.Category&da.Mask)==0)continue;
                }
                if(pairs>=1048576)throw new InvalidOperationException("Rod candidate pair limit 1048576 exceeded.");
                if(pairs==_segmentPairs.Length)Array.Resize(ref _segmentPairs,_segmentPairs.Length*2);
                bool swap=a.Rod>b.Rod||(a.Rod==b.Rod&&a.Segment>b.Segment);
                _segmentPairs[pairs++]=swap?new(){A=b.Rod,SA=b.Segment,B=a.Rod,SB=a.Segment}:new(){A=a.Rod,SA=a.Segment,B=b.Rod,SB=b.Segment};
            }
        }
        HeapSort(_segmentPairs.AsSpan(0,pairs),new PairOrder());
        for(int i=0;i<pairs;i++)
        {
            var pair=_segmentPairs[i];var a=Data[pair.A]!;var b=Data[pair.B]!;
            if(!SweepPair(a,pair.SA,b,pair.SB,out float u,out float v,out var normal))continue;
            var key=(pair.A,pair.SA,-pair.B-1,pair.SB);
            if(!_contactKeys.TryGetValue(key,out int index))
            {
                index=AddContact(key);_contacts[index]=new Contact{Rod=pair.A,Segment=pair.SA,Other=pair.B,OtherSegment=pair.SB,Shape=-1,Body=-1,Enabled=true,NormalLimit=float.MaxValue,Friction=(float)Math.Sqrt(a.Friction*b.Friction),Friction2=(float)Math.Sqrt(a.Friction*b.Friction),Restitution=Math.Max(a.Restitution,b.Restitution)};
            }
            ref var contact=ref _contacts[index];
            if(contact.Pass==0 && contact.Normal==default)ConfigurePair(ref contact,a,b,u,v,normal);
            contact.Pass=_contactPass;contact.U=u;contact.V=v;contact.Normal=normal;
            if(!contact.Enabled)continue;
            var point=a.PositionData[pair.SA]*(1-u)+a.PositionData[pair.SA+1]*u;
            var other=b.PositionData[pair.SB]*(1-v)+b.PositionData[pair.SB+1]*v;
            double separation=RodGeometry.Dot(point-other,RodGeometry.Double(normal))-a.Material.Radius-b.Material.Radius+contact.SeparationOffset;
            double wa=(1-u)*(1-u)*a.InverseMass[pair.SA]+u*u*a.InverseMass[pair.SA+1];
            double wb=(1-v)*(1-v)*b.InverseMass[pair.SB]+v*v*b.InverseMass[pair.SB+1];
            if(separation>=0||wa+wb<=0)continue;
            float lambda=Math.Min((float)(-separation/(wa+wb)),Math.Max(0,contact.NormalLimit*h-contact.NormalLambda));contact.NormalLambda+=lambda;var impulse=normal*lambda;
            Move(a,pair.SA,impulse*((1-u)*a.InverseMass[pair.SA]));Move(a,pair.SA+1,impulse*(u*a.InverseMass[pair.SA+1]));
            Move(b,pair.SB,-impulse*((1-v)*b.InverseMass[pair.SB]));Move(b,pair.SB+1,-impulse*(v*b.InverseMass[pair.SB+1]));
        }
    }
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
    private bool SweepPair(RodParticles a,int sa,RodParticles b,int sb,out float u,out float v,out Vector3F normal)
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
        _conservativeStops++;return normal!=default;
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
