using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Cables;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

// Shared deterministic fixtures for the hash runner, tests and stability measurements of pulley systems: tackle-2d (a four-fall block and tackle plus a friction pulley), belt-2d (a three-wheel belt
// under a driver and a load), chain-drive-3d (a bicycle-like frame with a chain from crank to rear wheel) and crane-3d (a slewing boom with a winch and a two-fall block carrying a crate).
internal sealed class PulleyScene
{
    internal readonly string Name;
    internal readonly P3.PhysicsWorld3D? Spatial;
    internal readonly P2.PhysicsWorld2D? Planar;
    internal readonly List<P3.PulleySystem> Systems3=[];
    internal readonly List<P2.PulleySystem> Systems2=[];
    private readonly List<double> _initialLength=[];
    private readonly List<(P3.Body Body,double Half)> _floor3=[];
    private readonly List<(P2.Body Body,double Half)> _floor2=[];
    private double _floorTop;
    private P3.Body _boom,_crate,_crank,_wheel;private P3.Body _frame;
    private P2.Body _haul,_driver,_loadWheel,_light;
    private P3.PulleySystem _winch;
    internal const float Dt=1f/60;
    internal static double SlewAmplitude=.15;
    private PulleyScene(string name,int threads)
    {
        Name=name;
        if(name=="tackle-2d"||name=="belt-2d")Planar=new(new(){WorldId=880,ThreadCount=threads,SleepEnabled=false});
        else Spatial=new(new(){WorldId=881,ThreadCount=threads,SleepEnabled=false});
    }
    internal static IReadOnlyList<string> Names=>["tackle-2d","belt-2d","chain-drive-3d","crane-3d"];
    internal static PulleyScene Create(string name,int threads)
    {
        var scene=new PulleyScene(name,threads);
        switch(name)
        {
            case "tackle-2d":scene.Tackle();break;
            case "belt-2d":scene.Belt();break;
            case "chain-drive-3d":scene.Chain();break;
            case "crane-3d":scene.Crane();break;
            default:throw new ArgumentException("Unknown pulley fixture.",nameof(name));
        }
        foreach(var s in scene.Systems3)scene._initialLength.Add(s.TotalLength);
        foreach(var s in scene.Systems2)scene._initialLength.Add(s.TotalLength);
        return scene;
    }

    private void Tackle()
    {
        var w=Planar!;_floorTop=0;
        w.AddBox(new(30,1),at:new(0,-.5),type:BodyType.Static);
        const double R=.1,yF=6,bx=2*R;
        var block=w.AddBox(new(.9f,.3f),at:new(bx,3),density:(float)(20/(.9*.3)));_floor2.Add((block,.15));
        _haul=w.AddBox(new(.1f,.1f),at:new(7*R,2.0),density:100);
        P2.PulleyNodeDef Pull(double x,bool fixedPulley,WrapDirection d)=>new(){Kind=CableNodeKind.Wheel,Body=fixedPulley?default:block.RawHandle,Point=fixedPulley?new(x,yF):new(x-bx,0),Radius=(float)R,Direction=d,Inertia=1e-3f};
        Systems2.Add(w.AddPulleySystem(new(){Nodes=[P2.PulleyNodeDef.Anchor(default,new(-R,yF)),Pull(0,false,WrapDirection.CounterClockwise),Pull(2*R,true,WrapDirection.Clockwise),Pull(4*R,false,WrapDirection.CounterClockwise),Pull(6*R,true,WrapDirection.Clockwise),P2.PulleyNodeDef.Anchor(_haul.RawHandle,default)],BreakForce=5000}));
        // a friction pulley carrying two weights that stick and slide in turn (capstan ratio e^(0.3 pi) = 2.57, mass ratio 2.5)
        var heavy=w.AddBox(new(.3f,.3f),at:new(-3-.25,3),density:50);var light=w.AddBox(new(.3f,.3f),at:new(-3+.25,3),density:20);_light=light;_floor2.Add((heavy,.15));_floor2.Add((light,.15));
        Systems2.Add(w.AddRopeOverPulleys(P2.RopeEnd.On(heavy,default),[new P2.Pulley(new(-3,5),.25f,clockwise:true,friction:.3f,inertia:float.PositiveInfinity)],P2.RopeEnd.On(light,default)));
    }

    private void Belt()
    {
        var w=Planar!;_floorTop=-5;
        var ground=w.AddBox(new(30,1),at:new(0,-5.5),type:BodyType.Static);
        const float R=.12f;
        _driver=w.AddCircle(R,at:new(0,0),density:400);_loadWheel=w.AddCircle(R,at:new(.7,0),density:400);var idler=w.AddCircle(.08f,at:new(.35,.5),density:400);
        w.AddHinge(ground,_driver,new(0,0));w.AddHinge(ground,_loadWheel,new(.7,0));w.AddHinge(ground,idler,new(.35,.5));
        Systems2.Add(w.AddBelt([new P2.Pulley(new(0,0),R,_driver),new P2.Pulley(new(0,0),.08f,idler),new P2.Pulley(new(0,0),R,_loadWheel)],friction:.4f,preload:12));
        // the belt's lower run is open: the wheels turn the other way round the loop (the idler sits above)
        var ballast=w.AddBox(new(.3f,.3f),at:new(2,-4.84),density:50);_floor2.Add((ballast,.15));
    }

    private void Chain()
    {
        var w=Spatial!;_floorTop=0;
        w.AddBox(new(60,1,60),at:new(0,-.5,0),type:BodyType.Static);
        var frame=w.AddBox(new(1.1f,.12f,.12f),at:new(0,.45,0),density:300);_frame=frame;
        // wheels and the crank turn about Z: a cylinder's own axis is Y, so stand them on their sides
        var onSide=QuaternionF.CreateFromAxisAngle(new(1,0,0),(float)(Math.PI/2));
        _wheel=w.AddCylinder(.04f,.35f,at:new(-.5,.35,0),rotation:onSide,density:150);var front=w.AddCylinder(.04f,.35f,at:new(.5,.35,0),rotation:onSide,density:150);
        _crank=w.AddCylinder(.03f,.08f,at:new(0,.45,.12),rotation:onSide,density:400);
        w.AddHinge(frame,_wheel,new(-.5,.35,0),new(0,0,1));w.AddHinge(frame,front,new(.5,.35,0),new(0,0,1));w.AddHinge(frame,_crank,new(0,.45,.12),new(0,0,1));
        _floor3.Add((_wheel,.04));
        // chain from the crank chainring (44 teeth) to the rear sprocket (16 teeth); sprockets on the wheel and crank axles
        Systems3.Add(w.AddChainDrive([new P3.Sprocket(_crank,default,44,new(0,1,0)),new P3.Sprocket(_wheel,new(0,.12,0),16,new(0,1,0))],.0127f));
    }

    private void Crane()
    {
        var w=Spatial!;_floorTop=0;
        w.AddBox(new(40,1,40),at:new(0,-.5,0),type:BodyType.Static);
        // the boom slews about the vertical axis through the origin; it is kinematic and carries the winch drum and the boom-tip sheaves
        _boom=w.AddBox(new(.4f,.4f,.4f),at:new(0,6,0),type:BodyType.Kinematic);
        _crate=w.AddBox(new(.6f,.6f,.6f),at:new(4,.3,0),density:500);_floor3.Add((_crate,.3));   // 108 kg on the ground
        const float R=.15f;
        // The rope runs from the drum over the boom-tip sheave, down and under a sheave on the crate, and up to a dead end on the boom: two falls carry the crate.
        var drum=P3.PulleyNodeDef.Drum(_boom.RawHandle,new(-.5,0,0),.25f,new(0,0,1),WrapDirection.Clockwise,2);
        drum.FixedAxle=true;drum.BrakeTorque=3000;
        P3.PulleyNodeDef Sheave(P3.Body carrier,Vector3D at,bool mounted)=>new(){Kind=CableNodeKind.Wheel,Body=carrier.RawHandle,Point=at,Radius=R,Direction=WrapDirection.Clockwise,Inertia=.01f,FixedAxle=mounted,Axis=new(0,0,1)};
        var deadEnd=P3.PulleyNodeDef.Anchor(_boom.RawHandle,new(3.85,0,0));deadEnd.FixedAxle=true;
        Systems3.Add(_winch=w.AddPulleySystem(new(){Nodes=[drum,Sheave(_boom,new(4,0,0),true),Sheave(_crate,new(0,.6,0),false),deadEnd],BreakForce=100000}));
    }

    internal void Step(int step)
    {
        double t=step*Dt;
        switch(Name)
        {
            case "tackle-2d":
            {
                var (_,c)=Math64.SinCos(t*1.2);
                float hold=(float)(20*9.81/4-_haul.Mass*9.81);_haul.ApplyForce(new(0,-hold*(1+.06f*(float)c)));
                var (_,c2)=Math64.SinCos(t*2.4);_light.ApplyForce(new(0,(float)(.1*_light.Mass*9.81*c2)));
                break;
            }
            case "belt-2d":
            {
                var (s,_)=Math64.SinCos(t*2.0);
                _driver.ApplyTorque((float)(6+4*s));_loadWheel.ApplyTorque(-.35f*_loadWheel.AngularVelocity-1.2f);
                break;
            }
            case "chain-drive-3d":
            {
                var (s,_)=Math64.SinCos(t*3.0);
                _crank.ApplyTorque(new(0,0,(float)(-(6+3*s))));
                break;
            }
            case "crane-3d":
            {
                // 0-3.5 s the winch lifts; the boom slews from 1 s on; 3.5-6 s a brake holds the load swinging; then the winch lowers it.
                _boom.MovePosition(new(0,6,0));
                _boom.MoveRotation(QuaternionF.CreateFromAxisAngle(new(0,1,0),(float)(SlewAmplitude*(1-Math64.SinCos(.8*Math.Max(0,t-1)).Cos))));
                if(t<3.5)_winch.SetMotor(0,2.5f,4000);
                else if(t<6){_winch.SetMotor(0,0,0);_winch.SetBrake(0,3000);}
                else{_winch.SetBrake(0,0);_winch.SetMotor(0,-2f,4000);}
                break;
            }
        }
        if(Spatial!=null)Spatial.Step(Dt);else Planar!.Step(Dt);
    }
    internal ulong Hash()=>Spatial?.ComputeHash()??Planar!.ComputeHash();

    /// <summary>The largest overstretch of a loaded system: its total path length minus its rope, over the systems that carry tension. Rope sliding through a wheel does not change it.</summary>
    internal double MaxLoadedStretch()
    {
        double max=0;
        foreach(var s in Systems3)if(s.MaximumTension>0)max=Math.Max(max,s.TotalStretch);
        foreach(var s in Systems2)if(s.MaximumTension>0)max=Math.Max(max,s.TotalStretch);
        return max;
    }
    internal double LengthDrift()
    {
        double max=0;int i=0;
        foreach(var s in Systems3)max=Math.Max(max,Math.Abs(s.TotalLength-_initialLength[i++]));
        foreach(var s in Systems2)max=Math.Max(max,Math.Abs(s.TotalLength-_initialLength[i++]));
        return max;
    }
    internal double MaxSpeed()
    {
        var store=Spatial?.Core.Bodies??Planar!.Core.Bodies;double max=0;
        for(int b=0;b<store.HighWater;b++)if(store.IsLive(b)&&store.IsDynamic(b)){var v=store.LinearVelocityOf(b);double sq=(double)v[0]*v[0]+(double)v[1]*v[1]+(v.Length==3?(double)v[2]*v[2]:0);max=Math.Max(max,Math.Sqrt(sq));}
        return max;
    }
    internal double Penetration()
    {
        double max=0;
        foreach(var (b,h) in _floor3)max=Math.Max(max,_floorTop-(b.Position.Y-h));
        foreach(var (b,h) in _floor2)max=Math.Max(max,_floorTop-(b.Position.Y-h));
        return max;
    }
    internal int Tunnelled()
    {
        int count=0;
        if(Spatial!=null){var bs=Spatial.Core.Bodies;for(int b=0;b<bs.HighWater;b++)if(bs.IsLive(b)&&bs.IsDynamic(b)&&bs.PositionOf(b)[1]<-.5)count++;}
        else{var bs=Planar!.Core.Bodies;for(int b=0;b<bs.HighWater;b++)if(bs.IsLive(b)&&bs.IsDynamic(b)&&bs.PositionOf(b)[1]<(Name=="belt-2d"?-6:-.5))count++;}
        foreach(var s in Systems3)if(s.IsBroken)count++;
        foreach(var s in Systems2)if(s.IsBroken)count++;
        return count;
    }
    /// <summary>The scene's own check of behaviour beyond stability, a value the run prints: crane load height, bicycle distance, belt slip.</summary>
    internal string Extra()
    {
        if(Name=="crane-3d")return $"crateY={_crate.Position.Y:F3} crateX={_crate.Position.X:F3} crateZ={_crate.Position.Z:F3}";
        if(Name=="chain-drive-3d")return $"frameX={_frame.Position.X:F3} wheelSpin={_wheel.AngularVelocity.Z:F3} crankSpin={_crank.AngularVelocity.Z:F3}";
        if(Name=="belt-2d")return $"driver={_driver.AngularVelocity:F3} load={_loadWheel.AngularVelocity:F3}";
        return $"haulY={_haul.Position.Y:F3}";
    }
}
