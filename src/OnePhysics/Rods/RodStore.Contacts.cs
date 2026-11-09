using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    private struct Contact
    {
        internal int Rod,Segment,Other,OtherSegment,Shape,Body,Subshape,Pass;
        internal float U,V,Friction,Friction2,Spin,Rolling,Restitution,SeparationOffset,NormalLimit,NormalLambda,Approach,TangentSpeed,TangentSpeed2;
        internal Vector3F Normal,BodyNormal,BodyAnchor,Tangent1,Tangent2;
        internal Vector3D Surface;
        internal bool Enabled;
    }
    // Keys are looked up only, never enumerated; first encounters append in geometry order.
    private readonly Dictionary<(int,int,int,int),int> _contactKeys=new();
    private Contact[] _contacts=new Contact[16];
    private int _contactCount,_contactPass;
    private readonly float[] _contactPayload=new float[104],_contactExtra=new float[10];
    private readonly byte[] _contactPoints=new byte[1];
    private static float Skin(RodParticles d)=>Math.Max(1e-6f,d.Material.Radius*.002f);
    private bool AttachedTo(RodParticles d,int body)
    {
        for(int p=0;p<d.AttachmentBody.Length;p++)if(d.AttachmentBody[p]==body&&Host.RodCore.Bodies.Handles.IsAlive(body,d.AttachmentGeneration[p]))return true;
        return false;
    }
    private bool Near(int shape,Vector3D a0,Vector3D b0,Vector3D a1,Vector3D b1,float radius)
    {
        Host.Bounds(shape,out var min,out var max);int body=Host.RodCore.Shapes.Owner[shape];
        BodyPose(body,_time1,out var position,out _);var shift=position-_bodyEnd[body];
        double pad=BodyMotionBound(body,_time1-_time0)+radius+.001;
        min+=shift-new Vector3D(pad,pad,pad);max+=shift+new Vector3D(pad,pad,pad);
        return Math.Min(Math.Min(a0.X,b0.X),Math.Min(a1.X,b1.X))<=max.X&&Math.Max(Math.Max(a0.X,b0.X),Math.Max(a1.X,b1.X))>=min.X
            &&Math.Min(Math.Min(a0.Y,b0.Y),Math.Min(a1.Y,b1.Y))<=max.Y&&Math.Max(Math.Max(a0.Y,b0.Y),Math.Max(a1.Y,b1.Y))>=min.Y
            &&(Host.Planar||(Math.Min(Math.Min(a0.Z,b0.Z),Math.Min(a1.Z,b1.Z))<=max.Z&&Math.Max(Math.Max(a0.Z,b0.Z),Math.Max(a1.Z,b1.Z))>=min.Z));
    }
    private void WorldContacts(float h)
    {
        var shapes=Host.RodCore.Shapes;
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];if(d==null||!d.CollisionEnabled)continue;
            for(int segment=0;segment<d.Length.Length;segment++)
            {
                if(d.InverseMass[segment]+d.InverseMass[segment+1]==0)continue;
                for(int s=0;s<shapes.HighWater;s++)
                {
                    if(!shapes.Handles.IsLive(s)||!Host.Matches(d,s))continue;
                    int body=shapes.Owner[s];if(!d.CollideWithAttachedBodies&&AttachedTo(d,body))continue;
                    var a0=d.Previous[segment];var b0=d.Previous[segment+1];var a1=d.PositionData[segment];var b1=d.PositionData[segment+1];
                    if(!Near(s,a0,b0,a1,b1,d.Material.Radius))continue;
                    if(!SweepWorld(s,a0,b0,a1,b1,d.Material.Radius,Skin(d),out var witness,out float fraction))continue;
                    WorldWitness(r,d,segment,segment,s,witness,fraction,h);
                }
            }
            // End-cap sweeps complement the interior segment witness. A face can constrain
            // both ends even when GJK deterministically returns only one closest feature.
            for(int p=0;p<d.PositionData.Length;p++)
            {
                if(d.InverseMass[p]==0)continue;
                int segment=Math.Min(p,d.Length.Length-1);float u=p==d.Length.Length?1:0;
                for(int s=0;s<shapes.HighWater;s++)
                {
                    if(!shapes.Handles.IsLive(s)||!Host.Matches(d,s))continue;
                    if(!d.CollideWithAttachedBodies&&AttachedTo(d,shapes.Owner[s]))continue;
                    var start=d.Previous[p];var end=d.PositionData[p];
                    if(!Near(s,start,start,end,end,d.Material.Radius))continue;
                    if(!SweepWorld(s,start,start,end,end,d.Material.Radius,Skin(d),out var witness,out float fraction))continue;
                    witness.Fraction=u;WorldWitness(r,d,segment,-p-1,s,witness,fraction,h);
                }
            }
        }
    }
    private void WorldWitness(int r,RodParticles d,int segment,int keySegment,int s,RodWitness witness,float fraction,float h)
    {
        int body=Host.RodCore.Shapes.Owner[s];var key=(r,keySegment,s,witness.Subshape);
        if(!_contactKeys.TryGetValue(key,out int index))
        {
            index=AddContact(key);
            _contacts[index]=new Contact{Rod=r,Segment=segment,Other=-1,Shape=s,Body=body,Subshape=witness.Subshape,Enabled=true,NormalLimit=float.MaxValue};
            ref var fresh=ref _contacts[index];var material=Host.Mix(s,witness.Subshape,d.Friction,d.Restitution);
            fresh.Rolling=material.RollingResistance;fresh.Friction=material.Friction;fresh.Friction2=material.Friction;fresh.Restitution=material.Restitution;Configure(ref fresh,d,witness,h);
        }
        ref var contact=ref _contacts[index];contact.Pass=_contactPass;contact.U=witness.Fraction;
        float time=_time0+(_time1-_time0)*fraction;
        BodyCollisionPose(body,time,out var p,out var q);
        contact.BodyAnchor=RodParticles.Rotate(RodParticles.Conjugate(q),RodGeometry.Float(witness.Surface-p));
        contact.BodyNormal=RodParticles.Rotate(RodParticles.Conjugate(q),witness.Normal);
        BodyPose(body,_time1,out p,out q);
        contact.Normal=RodParticles.Rotate(q,contact.BodyNormal);contact.Surface=p+RodGeometry.Double(RodParticles.Rotate(q,contact.BodyAnchor));
        SolveContact(ref contact,h);
    }
    private int AddContact((int,int,int,int) key)
    {
        if(_contactCount>=1048576)throw new InvalidOperationException("Rod contact limit 1048576 exceeded.");
        if(_contactCount==_contacts.Length)Array.Resize(ref _contacts,_contacts.Length*2);
        int index=_contactCount++;_contactKeys.Add(key,index);return index;
    }
    private bool SweepWorld(int shape,Vector3D a0,Vector3D b0,Vector3D a1,Vector3D b1,float radius,float skin,out RodWitness witness,out float fraction)
    {
        double speed=Math.Max(RodGeometry.Length(a1-a0),RodGeometry.Length(b1-b0))+BodyMotionBound(Host.RodCore.Shapes.Owner[shape],_time1-_time0);
        fraction=0;witness=default;
        // Conservative advancement uses a Lipschitz speed bound for both endpoints and the rigid shape, including rotation.
        for(int i=0;i<64;i++)
        {
            var a=a0+(a1-a0)*fraction;var b=b0+(b1-b0)*fraction;
            witness=Host.Distance(shape,a,b,radius,_time0+(_time1-_time0)*fraction);
            if(!float.IsFinite(witness.Separation))return false;
            if(witness.Separation<=skin)return witness.Normal.LengthSquared()>0;
            if(!(speed>1e-15))return false;
            double advance=.8*(witness.Separation-skin)/speed;
            if(fraction+advance>1)return false;
            if(advance<1e-8)return true;
            fraction+=(float)advance;
        }
        // Exhausting the bound is a conservative contact, never permission to step through a thin object.
        _conservativeStops++;return witness.Normal.LengthSquared()>0;
    }
    private void Configure(ref Contact c,RodParticles d,RodWitness witness,float h)
    {
        var n=witness.Normal;var surface=Host.RodCore.Surfaces.Get(c.Shape);
        BodyCollisionPose(c.Body,_time1,out var position,out var rotation);
        var arm=RodGeometry.Float(witness.Surface-position);
        var pointVelocity=d.VelocityData[c.Segment]*(1-witness.Fraction)+d.VelocityData[c.Segment+1]*witness.Fraction;
        var relative=pointVelocity-BodyPointVelocity(c.Body,arm);c.Approach=Math.Min(0,Vector3F.Dot(relative,n));
        c.Spin=surface.SpinningFriction;
        var oneWay=RodParticles.Rotate(rotation,surface.OneWayDirection);
        if(oneWay!=default)
        {
            oneWay/= (float)Math.Sqrt(oneWay.LengthSquared());
            c.Enabled=Vector3F.Dot(oneWay,n)>=.5f&&Vector3F.Dot(relative,oneWay)<=.01f;
        }
        var axis=RodParticles.Rotate(rotation,surface.FrictionAxis);
        SurfaceContacts.Tangents(n,Host.Planar?2:3,axis,out c.Tangent1,out c.Tangent2);
        var conveyor=RodParticles.Rotate(rotation,surface.Velocity);
        c.TangentSpeed=Vector3F.Dot(conveyor,c.Tangent1);c.TangentSpeed2=Vector3F.Dot(conveyor,c.Tangent2);
        if(axis!=default)
        {
            axis/=(float)Math.Sqrt(axis.LengthSquared());float alignment=Vector3F.Dot(axis,c.Tangent1);alignment*=alignment;
            float friction=(float)Math.Sqrt(surface.LongitudinalFriction*surface.LongitudinalFriction*alignment+surface.TransverseFriction*surface.TransverseFriction*(1-alignment));
            c.Friction=(float)Math.Sqrt(d.Friction*friction);
            float transverse=Vector3F.Dot(axis,c.Tangent2);transverse*=transverse;
            float second=(float)Math.Sqrt(surface.LongitudinalFriction*surface.LongitudinalFriction*transverse+surface.TransverseFriction*surface.TransverseFriction*(1-transverse));
            c.Friction2=(float)Math.Sqrt(d.Friction*second);
        }
        if(Host.RodCore.ContactModifier is not {} modifier)return;
        Array.Clear(_contactPayload);Array.Clear(_contactExtra);_contactPoints[0]=c.Enabled?(byte)1:(byte)0;
        _contactPayload[ManifoldLayout.Normal]=n.X;_contactPayload[ManifoldLayout.Normal+1]=n.Y;_contactPayload[ManifoldLayout.Normal+2]=n.Z;
        _contactPayload[ManifoldLayout.Friction]=c.Friction;_contactPayload[ManifoldLayout.Restitution]=c.Restitution;
        _contactPayload[ManifoldLayout.Separation(0)]=witness.Separation;
        _contactExtra[0]=c.TangentSpeed;_contactExtra[1]=float.MaxValue;_contactExtra[2]=c.TangentSpeed2;_contactExtra[3]=c.Friction2;
        var view=new ContactModification(Host.RodCore,c.Shape,c.Subshape,c.Rod,c.Segment,_contactPayload,_contactPoints,_contactExtra);
        modifier(ref view);c.Enabled=view.Enabled;c.Friction=view.Friction;c.Friction2=_contactExtra[3];c.Restitution=view.Restitution;c.SeparationOffset=view.GetSeparation(0)-witness.Separation;c.NormalLimit=view.NormalImpulseLimit;c.TangentSpeed=view.TangentSpeed;
    }
    private void SolveContact(ref Contact c,float h)
    {
        if(!c.Enabled)return;
        var a=Data[c.Rod]!;float u=c.U;var normal=c.Normal;
        var point=a.PositionData[c.Segment]*(1-u)+a.PositionData[c.Segment+1]*u;
        double separation=RodGeometry.Dot(point-c.Surface,RodGeometry.Double(normal))-a.Material.Radius+c.SeparationOffset;
        var arm=RodGeometry.Float(c.Surface-(_bodyStart[c.Body]+(_bodyEnd[c.Body]-_bodyStart[c.Body])*_time1+_bodyDelta[c.Body]));
        BodyPose(c.Body,_time1,out _,out var rotation);
        var cross=RodParticles.Cross(arm,normal);
        double weight=(1-u)*(1-u)*a.InverseMass[c.Segment]+u*u*a.InverseMass[c.Segment+1]+BodyWeight(c.Body)+Vector3F.Dot(cross,Inertia(c.Body,rotation,cross));
        if(!(weight>0)||separation>=0)return;
        float lambda=(float)(-separation/weight);
        lambda=Math.Min(lambda,Math.Max(0,c.NormalLimit*h-c.NormalLambda));c.NormalLambda+=lambda;
        var impulse=normal*lambda;
        Move(a,c.Segment,impulse*((1-u)*a.InverseMass[c.Segment]));Move(a,c.Segment+1,impulse*(u*a.InverseMass[c.Segment+1]));
        BodyPush(c.Body,-impulse,arm,h,_time1,true);
    }
    private void ContactVelocities(float h,int lastPass)
    {
        for(int i=0;i<_contactCount;i++)
        {
            ref var c=ref _contacts[i];if(!c.Enabled||(c.Pass!=lastPass&&c.NormalLambda<=0))continue;
            if(c.Other>=0){PairVelocity(ref c,h);continue;}
            var a=Data[c.Rod]!;float u=c.U;
            BodyPose(c.Body,_time1,out var position,out var rotation);var arm=RodParticles.Rotate(rotation,c.BodyAnchor);
            var lever=c.Normal*(-a.Material.Radius);
            var velocity=a.VelocityData[c.Segment]*(1-u)+a.VelocityData[c.Segment+1]*u+RodParticles.Cross(a.AngularVelocity[c.Segment],lever)-BodyPointVelocity(c.Body,arm);
            float vn=Vector3F.Dot(velocity,c.Normal);
            float pointWeight=(1-u)*(1-u)*a.InverseMass[c.Segment]+u*u*a.InverseMass[c.Segment+1];
            var cross=RodParticles.Cross(arm,c.Normal);float weight=pointWeight+BodyWeight(c.Body)+Vector3F.Dot(cross,Inertia(c.Body,rotation,cross));
            float normalImpulse=weight>0?Math.Min(Math.Max(0,(-c.Restitution*c.Approach-vn)/weight),Math.Max(0,c.NormalLimit-c.NormalLambda/h)):0;
            var impulse=c.Normal*normalImpulse;
            SurfaceContacts.Tangents(c.Normal,Host.Planar?2:3,c.Tangent1,out var t1,out var t2);
            int rigid=c.Body;int rodSegment=c.Segment;
            float TangentWeight(Vector3F axis)
            { var torque=RodParticles.Cross(arm,axis);var spin=RodParticles.Cross(lever,axis);return pointWeight+BodyWeight(rigid)+Vector3F.Dot(torque,Inertia(rigid,rotation,torque))+Vector3F.Dot(spin,a.InverseAngularImpulse(rodSegment,spin)); }
            float tw1=TangentWeight(t1),tw2=TangentWeight(t2);
            float j1=tw1>0?-(Vector3F.Dot(velocity,t1)-c.TangentSpeed)/tw1:0;
            float j2=!Host.Planar&&tw2>0?-(Vector3F.Dot(velocity,t2)-c.TangentSpeed2)/tw2:0;
            float support=normalImpulse+c.NormalLambda/h,limit1=c.Friction*support,limit2=c.Friction2*support;
            if(limit1==0)j1=0;if(limit2==0)j2=0;
            double ellipse=(limit1>0?(double)j1*j1/((double)limit1*limit1):0)+(limit2>0?(double)j2*j2/((double)limit2*limit2):0);
            if(ellipse>1){float scale=(float)(1/Math.Sqrt(ellipse));j1*=scale;j2*=scale;}
            impulse+=t1*j1+t2*j2;
            a.VelocityData[c.Segment]+=impulse*((1-u)*a.InverseMass[c.Segment]);a.VelocityData[c.Segment+1]+=impulse*(u*a.InverseMass[c.Segment+1]);BodyPush(c.Body,-impulse,arm,h,_time1,false);
            a.AngularVelocity[c.Segment]+=a.InverseAngularImpulse(c.Segment,RodParticles.Cross(lever,impulse));
            if(!Host.Planar&&(c.Spin>0||c.Rolling>0))
            {
                var w=Host.RodCore.Bodies.AngularVelocityOf(c.Body);
                var relative=a.AngularVelocity[c.Segment]-new Vector3F(w[0],w[1],w[2])-_bodyAngularDelta[c.Body];
                AngularContact(a,c.Segment,c.Body,rotation,c.Normal,Vector3F.Dot(relative,c.Normal),c.Spin*support);
                var roll=relative-c.Normal*Vector3F.Dot(relative,c.Normal);float rolling=(float)Math.Sqrt(roll.LengthSquared());
                if(rolling>1e-12f)AngularContact(a,c.Segment,c.Body,rotation,roll/rolling,rolling,c.Rolling*support*a.Material.Radius);
            }
        }
    }
    private void AngularContact(RodParticles rod,int segment,int body,QuaternionF rotation,Vector3F axis,float speed,float limit)
    {
        if(!(limit>0))return;
        var a=rod.InverseAngularImpulse(segment,axis);var b=Inertia(body,rotation,axis);
        float weight=Vector3F.Dot(axis,a+b);if(!(weight>0))return;
        float impulse=Math.Clamp(-speed/weight,-limit,limit);
        rod.AngularVelocity[segment]+=a*impulse;
        if(BodyWeight(body)>0){_bodyAngularDelta[body]-=b*impulse;_bodyChanged[body]=true;}
    }

}
