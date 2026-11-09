using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    private Vector3D[] _bodyStart = [], _bodyEnd = [], _bodyDelta = [];
    private QuaternionF[] _rotationStart = [], _rotationEnd = [], _rotationDelta = [];
    private Vector3F[] _bodyVelocityDelta = [], _bodyAngularDelta = [];
    private bool[] _bodyChanged = [];
    private Vector3D[] _bodySubStart=[];
    private QuaternionF[] _rotationSubStart=[];
    private float _time0,_time1;
    private float _stepDt;
    internal void BeginStep()
    {
        int n=Host.RodCore.Bodies.HighWater;
        if (_bodyStart.Length<n)
        {
            int size=Math.Max(n,Math.Max(8,_bodyStart.Length*2));
            Array.Resize(ref _bodySubStart,size);Array.Resize(ref _rotationSubStart,size);
            Array.Resize(ref _bodyStart,size);Array.Resize(ref _bodyEnd,size);Array.Resize(ref _bodyDelta,size);
            Array.Resize(ref _rotationStart,size);Array.Resize(ref _rotationEnd,size);Array.Resize(ref _rotationDelta,size);
            Array.Resize(ref _bodyVelocityDelta,size);Array.Resize(ref _bodyAngularDelta,size);Array.Resize(ref _bodyChanged,size);
        }
        for(int b=0;b<n;b++)
        {
            if(!Host.RodCore.Bodies.IsLive(b))continue;
            _bodyStart[b]=BodyPosition(b); _rotationStart[b]=BodyRotation(b);
        }
    }
    private Vector3D BodyPosition(int b)
    { var p=Host.RodCore.Bodies.PositionOf(b);return new(p[0],p[1],Host.Planar?0:p[2]); }
    private QuaternionF BodyRotation(int b)
    {
        var r=Host.RodCore.Bodies.RotationOf(b);
        if(!Host.Planar)return new(r[0],r[1],r[2],r[3]);
        var (s,c)=Math32.SinCos(0.5f*Math32.Atan2(r[1],r[0])); return new(0,0,s,c);
    }
    internal void BodyPose(int b,float time,out Vector3D position,out QuaternionF rotation)
    {
        position=_bodyStart[b]+(_bodyEnd[b]-_bodyStart[b])*time+_bodyDelta[b];
        var a=_rotationStart[b];var z=_rotationEnd[b];
        float sign=a.X*z.X+a.Y*z.Y+a.Z*z.Z+a.W*z.W<0?-1:1;
        var q=RodParticles.Unit(new(a.X+(sign*z.X-a.X)*time,a.Y+(sign*z.Y-a.Y)*time,a.Z+(sign*z.Z-a.Z)*time,a.W+(sign*z.W-a.W)*time));
        rotation=_rotationDelta[b]*q;
    }
    internal void BodyCollisionPose(int b,float time,out Vector3D position,out QuaternionF rotation)
    {
        BodyPose(b,_time1,out var end,out var q);
        float t=_time1>_time0?Math.Clamp((time-_time0)/(_time1-_time0),0,1):1;
        position=_bodySubStart[b]+(end-_bodySubStart[b])*t;
        var a=_rotationSubStart[b];float sign=a.X*q.X+a.Y*q.Y+a.Z*q.Z+a.W*q.W<0?-1:1;
        rotation=RodParticles.Unit(new(a.X+(sign*q.X-a.X)*t,a.Y+(sign*q.Y-a.Y)*t,a.Z+(sign*q.Z-a.Z)*t,a.W+(sign*q.W-a.W)*t));
    }
    internal float BodyMotionBound(int b,float interval)
    {
        BodyPose(b,_time1,out var end,out var z);var d=end-_bodySubStart[b];var a=_rotationSubStart[b];
        float sign=a.X*z.X+a.Y*z.Y+a.Z*z.Z+a.W*z.W<0?-1:1;
        double x=sign*z.X-a.X,y=sign*z.Y-a.Y,w=sign*z.W-a.W,zz=sign*z.Z-a.Z;
        return (float)(Math.Sqrt(d.X*d.X+d.Y*d.Y+d.Z*d.Z)+4*Math.Sqrt(x*x+y*y+zz*zz+w*w)*Host.BodyExtent(b));
    }
    private float BodyWeight(int b)=>Host.RodCore.Bodies.IsDynamic(b)?Host.RodCore.Bodies.InverseMass[b]:0;
    private Vector3F Inertia(int b,QuaternionF rotation,Vector3F torque)
    {
        if(BodyWeight(b)==0)return default;
        var inertia=Host.RodCore.Bodies.InverseInertiaOf(b);
        if(Host.Planar)return new(0,0,torque.Z*inertia[0]);
        var t=RodParticles.Rotate(RodParticles.Conjugate(rotation),torque);
        var local=new Vector3F(inertia[0]*t.X+inertia[1]*t.Y+inertia[2]*t.Z,inertia[1]*t.X+inertia[3]*t.Y+inertia[4]*t.Z,inertia[2]*t.X+inertia[4]*t.Y+inertia[5]*t.Z);
        return RodParticles.Rotate(rotation,local);
    }
    private void BodyPush(int b,Vector3F impulse,Vector3F arm,float h,float time,bool positional)
    {
        float weight=BodyWeight(b);if(weight==0)return;
        NoteBodyPush(b);
        BodyPose(b,time,out _,out var rotation);
        var angle=Inertia(b,rotation,RodParticles.Cross(arm,impulse));
        if(positional)
        {
            _bodyDelta[b]+=new Vector3D((double)impulse.X*weight,(double)impulse.Y*weight,(double)impulse.Z*weight);
            _rotationDelta[b]=RodParticles.Turn(_rotationDelta[b],angle);
        }
        float scale=positional?1/h:1;
        _bodyVelocityDelta[b]+=impulse*(weight*scale);_bodyAngularDelta[b]+=angle*scale;_bodyChanged[b]=true;
    }
    internal Vector3F BodyPointVelocity(int b,Vector3F arm)
    {
        var v=Host.RodCore.Bodies.LinearVelocityOf(b);var w=Host.RodCore.Bodies.AngularVelocityOf(b);
        return new Vector3F(v[0],v[1],Host.Planar?0:v[2])+_bodyVelocityDelta[b]+RodParticles.Cross(new Vector3F(Host.Planar?0:w[0],Host.Planar?0:w[1],Host.Planar?w[0]:w[2])+_bodyAngularDelta[b],arm);
    }
    internal void ValidateAttachment(BodyHandle body,Vector3F local,float breakForce)
    {
        if(body.IsNull||body.World!=Host.RodCore.Id||!Host.RodCore.Bodies.Handles.IsAlive(body.Index,body.Generation))throw new ArgumentException("Attachment body must belong to this world and be alive.",nameof(body));
        if(!Finite(local)||Math.Abs(local.X)>2097152||Math.Abs(local.Y)>2097152||Math.Abs(local.Z)>2097152||(Host.Planar&&local.Z!=0))throw new ArgumentOutOfRangeException(nameof(local),local,"Local anchor must be finite within ±2097152 metres, with Z zero in a planar world.");
        RodMaterial.Check(breakForce,float.Epsilon,float.MaxValue,nameof(breakForce));
    }
    internal void Bind(RodParticles d,int p,BodyHandle body,Vector3F anchor,float breakForce)
    {
        d.SetPinned(p,false);d.AttachmentBody[p]=body.Index;d.AttachmentGeneration[p]=body.Generation;d.AttachmentLocal[p]=anchor;d.AttachmentBreak[p]=breakForce;d.AttachmentBroken[p]=false;d.AttachmentForce[p]=default;
        Host.RodCore.Wake(body);
    }
    private static Vector3F Solve3(Vector3F x,Vector3F y,Vector3F z,Vector3F rhs)
    {
        double a=x.X,b=y.X,c=z.X,d=y.Y,e=z.Y,f=z.Z;
        double aa=d*f-e*e,bb=c*e-b*f,cc=b*e-c*d,dd=a*f-c*c,ee=b*c-a*e,ff=a*d-b*b;
        double det=a*aa+b*bb+c*cc;if(!(det>1e-30))return default;
        return new((float)((aa*rhs.X+bb*rhs.Y+cc*rhs.Z)/det),(float)((bb*rhs.X+dd*rhs.Y+ee*rhs.Z)/det),(float)((cc*rhs.X+ee*rhs.Y+ff*rhs.Z)/det));
    }
    private void Attachments(float h,float time)
    {
        foreach(var d in Data)
        {
            if(d==null)continue;
            for(int p=0;p<d.PositionData.Length;p++)
            {
                int b=d.AttachmentBody[p];if(b<0)continue;
                if(!Host.RodCore.Bodies.Handles.IsAlive(b,d.AttachmentGeneration[p])){d.AttachmentBody[p]=-1;d.AttachmentBroken[p]=true;continue;}
                BodyPose(b,time,out var position,out var rotation);
                var arm=RodParticles.Rotate(rotation,d.AttachmentLocal[p]-Host.LocalCenter(b));
                var delta=d.PositionData[p]-position-new Vector3D(arm.X,arm.Y,arm.Z);
                float weight=d.InverseMass[p]+BodyWeight(b);
                Vector3F Column(Vector3F axis)=>axis*weight+RodParticles.Cross(Inertia(b,rotation,RodParticles.Cross(arm,axis)),arm);
                var impulse=Solve3(Column(new(1,0,0)),Column(new(0,1,0)),Column(new(0,0,1)),new((float)-delta.X,(float)-delta.Y,(float)-delta.Z));
                var total=d.AttachmentLambda[p]+impulse;
                double force=Math.Sqrt((double)total.X*total.X+(double)total.Y*total.Y+(double)total.Z*total.Z)/((double)h*h);
                if(force>d.AttachmentBreak[p]){d.AttachmentBody[p]=-1;d.AttachmentBroken[p]=true;continue;}
                d.AttachmentLambda[p]=total;Move(d,p,impulse*d.InverseMass[p]);BodyPush(b,-impulse,arm,h,time,true);
                d.AttachmentForce[p]+=impulse/(h*_stepDt);
            }
        }
    }
    private static void Move(RodParticles d,int p,Vector3F delta)
    {
        d.PositionData[p]+=new Vector3D(WorldCore.Grid(delta.X),WorldCore.Grid(delta.Y),WorldCore.Grid(delta.Z));
    }
    private bool BodyChangeUsable(int b)
    {
        var delta=_bodyDelta[b];var q=_rotationDelta[b]*_rotationEnd[b];var dv=_bodyVelocityDelta[b];var dw=_bodyAngularDelta[b];var p=_bodyEnd[b];
        double x=p.X+delta.X,y=p.Y+delta.Y,z=p.Z+delta.Z;
        return Math.Abs(x)<=4194304&&Math.Abs(y)<=4194304&&Math.Abs(z)<=4194304
            &&float.IsFinite(q.X+q.Y+q.Z+q.W)&&float.IsFinite(dv.X+dv.Y+dv.Z)&&float.IsFinite(dw.X+dw.Y+dw.Z);
    }
    private void FinishBodies()
    {
        var store=Host.RodCore.Bodies;
        for(int b=0;b<store.HighWater;b++)
        {
            if(!_bodyChanged[b]||!store.IsLive(b))continue;
            // A rod driven to non-finite or enormous impulses (damaged state) must not write NaN into a body: the body keeps the state it had,
            // and the rod's own bounds check raises the typed error at the next step.
            if(!BodyChangeUsable(b))continue;
            var p=store.PositionOf(b);var delta=_bodyDelta[b];p[0]+=WorldCore.Grid(delta.X);p[1]+=WorldCore.Grid(delta.Y);if(!Host.Planar)p[2]+=WorldCore.Grid(delta.Z);
            var rotation=_rotationDelta[b]*_rotationEnd[b];var r=store.RotationOf(b);
            if(Host.Planar){r[0]=1-2*rotation.Z*rotation.Z;r[1]=2*rotation.Z*rotation.W;}
            else{r[0]=rotation.X;r[1]=rotation.Y;r[2]=rotation.Z;r[3]=rotation.W;}
            var v=store.LinearVelocityOf(b);var dv=_bodyVelocityDelta[b];v[0]+=dv.X;v[1]+=dv.Y;if(!Host.Planar)v[2]+=dv.Z;
            var w=store.AngularVelocityOf(b);var dw=_bodyAngularDelta[b];if(Host.Planar)w[0]+=dw.Z;else{w[0]+=dw.X;w[1]+=dw.Y;w[2]+=dw.Z;}
            Host.RodCore.Wake(Host.RodCore.HandleOfBody(b));Host.MovedBody(b);
        }
    }
}
