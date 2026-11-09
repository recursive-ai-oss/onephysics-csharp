using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Rods;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

// Shared deterministic fixtures for the hash runner, stability measurements and indicative joint-chain comparison.
internal sealed class RodScene
{
    internal readonly string Name;
    internal readonly P3.PhysicsWorld3D? Spatial;
    internal readonly P2.PhysicsWorld2D? Planar;
    internal readonly List<P3.Rod> Rods3=[];
    internal readonly List<P2.Rod> Rods2=[];
    internal readonly List<P3.Body[]> Chains=[];
    internal readonly List<double[]> ChainLengths=[];
    internal readonly bool JointChain;
    private P3.Body _head;
    private P2.Rod _swing;
    internal const float Dt=1f/60;
    private RodScene(string name,int threads,bool chain)
    {
        Name=name;JointChain=chain;
        if(name=="rope-2d")Planar=new(new(){WorldId=790,ThreadCount=threads,SleepEnabled=false});
        else Spatial=new(new(){WorldId=791,ThreadCount=threads,SleepEnabled=false,Gravity=name=="rope-whip-3d"?default:new(0,-9.81f,0)});
    }
    internal static RodScene Create(string name,int threads,bool jointChain=false,int strands=1000)
    {
        var scene=new RodScene(name,threads,jointChain);
        switch(name)
        {
            case "rope-bridge-3d":scene.Bridge();break;
            case "rope-whip-3d":scene.Whip();break;
            case "hair-3d":scene.Hair(strands);break;
            case "rope-2d":scene.Swing();break;
            default:throw new ArgumentException("Unknown rod fixture.",nameof(name));
        }
        return scene;
    }
    private int AddLine(Vector3D from,Vector3D to,int segments,RodMaterial material,bool pinStart,bool pinEnd,int substeps=8,int iterations=4,float damping=1,bool collideAttached=false)
    {
        var w=Spatial!;var points=new Vector3D[segments+1];for(int i=0;i<=segments;i++)points[i]=from+(to-from)*((double)i/segments);
        if(!JointChain)
        {
            var rod=w.AddRod(new(){Points=points,Material=material,PinnedParticles=pinStart?(pinEnd?[0,segments]:[0]):(pinEnd?[segments]:[]),Substeps=substeps,Iterations=iterations,Damping=damping,CollideWithAttachedBodies=collideAttached});
            Rods3.Add(rod);return Rods3.Count-1;
        }
        var bodies=new P3.Body[segments+1];var lengths=new double[segments];double length=(to-from).Length()/segments;
        for(int i=0;i<=segments;i++)
        {
            bool pinned=(i==0&&pinStart)||(i==segments&&pinEnd);
            bodies[i]=w.AddSphere(material.Radius,at:points[i],type:pinned?BodyType.Static:BodyType.Dynamic);
            if(!pinned)bodies[i].Mass=(float)(Math.PI*material.Radius*material.Radius*material.Density*length*(i==0||i==segments?.5:1));
            if(i>0){w.AddDistance(bodies[i-1],bodies[i],points[i-1],points[i]);lengths[i-1]=length;}
        }
        Chains.Add(bodies);ChainLengths.Add(lengths);return Chains.Count-1;
    }
    private void Attach(int line,int particle,P3.Body body,Vector3F local)
    {
        if(!JointChain){Rods3[line].Attach(particle,body,local);return;}
        var point=body.Position+RodGeometryDouble(body.Rotation.Rotate(local));Spatial!.AddPoint(Chains[line][particle],body,point);
    }
    private static Vector3D RodGeometryDouble(Vector3F p)=>new(p.X,p.Y,p.Z);
    private void Bridge()
    {
        var w=Spatial!;w.AddBox(new(20,1,8),at:new(0,-2,0),type:BodyType.Static);
        var material=new RodMaterial(8e7f,3e7f,.025f,800);
        int left=AddLine(new(-3,1,-.6),new(3,1,-.6),24,material,true,true);
        int right=AddLine(new(-3,1,.6),new(3,1,.6),24,material,true,true);
        for(int i=0;i<12;i++)
        {
            var plank=w.AddBox(new(.45f,.12f,1.3f),at:new(-2.75+.5*i,1.06,0),density:50);
            Attach(left,1+2*i,plank,new(0,-.06f,-.6f));Attach(right,1+2*i,plank,new(0,-.06f,.6f));
        }
        for(int i=0;i<6;i++)w.AddBox(new(.35f,.35f,.35f),at:new(-1.5+.6*i,3+.2*(i%2),0),density:100);
    }
    private void Whip()
    {
        var w=Spatial!;w.AddBox(new(.01f,12,12),type:BodyType.Static);
        AddLine(new(-.6,-1,0),new(-.6,1,0),16,new RodMaterial(1e7f,4e6f,.01f,1000),false,false,8,4,0);
        if(!JointChain)for(int p=0;p<Rods3[0].ParticleCount;p++)Rods3[0].SetVelocity(p,new(30,0,0));
        else foreach(var p in Chains[0])p.LinearVelocity=new(30,0,0);
    }
    private void Hair(int count)
    {
        var w=Spatial!;_head=w.AddSphere(.35f,at:new(0,2,0),type:BodyType.Kinematic);
        for(int strand=0;strand<count;strand++)
        {
            double y=1-2*(strand+.5)/count;double radius=Math.Sqrt(1-y*y);var (sin,cos)=Math64.SinCos(strand*2.39996322972865332);
            var normal=new Vector3F((float)(radius*cos),(float)y,(float)(radius*sin));
            var root=normal*.3501f;var direction=normal*.4f+new Vector3F(0,-.8f,0);float inward=Vector3F.Dot(direction,normal);
            if(inward<.2f)direction+=normal*(.2f-inward);direction/=(float)Math.Sqrt(direction.LengthSquared());
            var from=new Vector3D(root.X,2+root.Y,root.Z);var to=from+RodGeometryDouble(direction*.18f);
            int line=AddLine(from,to,6,RodMaterial.Hair,false,false,4,2,2,true);Attach(line,0,_head,root);
        }
    }
    private void Swing()
    {
        var w=Planar!;w.AddBox(new(20,1),at:new(0,-.5),type:BodyType.Static);
        w.AddBox(new(.06f,1.2f),at:new(-1.9,.8),type:BodyType.Static);
        w.AddBox(new(.06f,.8f),at:new(-1.0,.4),type:BodyType.Static);
        w.AddCircle(.15f,at:new(-2.8,1.4),type:BodyType.Static);
        const int segments=24;var points=new Vector2D[segments+1];for(int i=0;i<=segments;i++)points[i]=new(-2+1.5*i/segments,3-2.0*i/segments);
        _swing=w.AddRod(new(){Points=points,PinnedParticles=[0],Material=new(5e7f,2e7f,.015f,1000),Damping=.3f,Substeps=8,Iterations=4});Rods2.Add(_swing);
        var weight=w.AddCircle(.18f,at:new(-.5,1),density:20);_swing.Attach(segments,weight);
    }
    internal void Step(int step)
    {
        if(Name=="hair-3d")
        {
            var (sin,cos)=Math64.SinCos(step*Dt*2.0943951023931953);
            _head.MovePosition(new(.15*sin,2+.05*(cos-1),0));
        }
        if(Name=="rope-2d")
        {var (sin,_)=Math64.SinCos(step*Dt*1.5);_swing.SetTarget(0,new(-2+.35*sin,3));}
        if(Spatial!=null)Spatial.Step(Dt);else Planar!.Step(Dt);
    }
    internal ulong Hash()=>Spatial?.ComputeHash()??Planar!.ComputeHash();
    internal RodStepStatistics Statistics=>Spatial?.RodStatistics??Planar!.RodStatistics;
    internal int Tunnelled()
    {
        int count=0;
        if(Name=="rope-whip-3d")
        {
            if(!JointChain)foreach(var rod in Rods3)for(int p=0;p<rod.ParticleCount;p++)if(rod.GetPosition(p).X>.005)count++;
            else foreach(var chain in Chains)foreach(var node in chain)if(node.Position.X>.005)count++;
        }
        else if(Name=="hair-3d"&&!JointChain)
        {
            var center=_head.Position;
            foreach(var rod in Rods3)for(int p=0;p<rod.ParticleCount;p++)if((rod.GetPosition(p)-center).Length()<.34)count++;
        }
        else
        {
            // Bridge and swing: a dynamic body or a rod particle whose centre is below the centre plane of the floor slab has passed through it.
            if(Spatial!=null)
            {
                var bs=Spatial.Core.Bodies;for(int b=0;b<bs.HighWater;b++)if(bs.IsLive(b)&&bs.IsDynamic(b)&&bs.PositionOf(b)[1]<-2)count++;
                foreach(var rod in Rods3)for(int p=0;p<rod.ParticleCount;p++)if(rod.GetPosition(p).Y<-2)count++;
            }
            else
            {
                var bs=Planar!.Core.Bodies;for(int b=0;b<bs.HighWater;b++)if(bs.IsLive(b)&&bs.IsDynamic(b)&&bs.PositionOf(b)[1]<-.5)count++;
                foreach(var rod in Rods2)for(int p=0;p<rod.ParticleCount;p++)if(rod.GetPosition(p).Y<-.5)count++;
            }
        }
        return count;
    }
    internal double BodyMaxSpeed()
    {
        var store=Spatial?.Core.Bodies??Planar!.Core.Bodies;double max=0;
        for(int b=0;b<store.HighWater;b++)if(store.IsLive(b)){var v=store.LinearVelocityOf(b);double sq=(double)v[0]*v[0]+(double)v[1]*v[1]+(v.Length==3?(double)v[2]*v[2]:0);max=Math.Max(max,Math.Sqrt(sq));}
        return max;
    }
    internal double ChainStretch()
    {
        double max=0;for(int i=0;i<Chains.Count;i++)for(int p=0;p<ChainLengths[i].Length;p++)max=Math.Max(max,Math.Abs((Chains[i][p+1].Position-Chains[i][p].Position).Length()/ChainLengths[i][p]-1));return max;
    }
}
