using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class PulleySystemTests(ITestOutputHelper helper)
{
    private readonly Out output=new(helper);
    private sealed class Out(ITestOutputHelper h)
    {
        internal void WriteLine(string s){h.WriteLine(s);var f=Environment.GetEnvironmentVariable("PULLEY_LOG");if(f!=null)System.IO.File.AppendAllText(f,s+"\n");}
    }
    private const float Dt=1f/60;
    private static P2.PhysicsWorld2D World2(float gravity=-9.81f)=>new(new(){Gravity=new(0,gravity),SleepEnabled=false});

    [Fact]
    public void Atwood2D()
    {
        var w=World2();
        var a=w.AddBox(new(.4f,.4f),at:new(-.25,3),density:12.5f);   // 2 kg
        var b=w.AddBox(new(.4f,.4f),at:new(.25,3.5),density:6.25f);  // 1 kg
        output.WriteLine($"masses {a.Mass} {b.Mass}");
        var rope=w.AddRopeOverPulleys(P2.RopeEnd.On(a,default),[new P2.Pulley(new(0,5),.25f,clockwise:true,inertia:1e-4f)],P2.RopeEnd.On(b,default));
        for(int i=0;i<60;i++)
        {
            w.Step(Dt);
            if(i%10==9)output.WriteLine($"t={(i+1)*Dt:F3} va={a.LinearVelocity.Y:F4} vb={b.LinearVelocity.Y:F4} T0={rope.Tension(0):F3} T1={rope.Tension(1):F3} stretch={rope.Stretch(0):E2},{rope.Stretch(1):E2} total={rope.TotalLength:F6}");
        }
        double g=9.81,expected=(a.Mass-b.Mass)/(a.Mass+b.Mass)*g;
        output.WriteLine($"expected a={expected:F4}, measured={-a.LinearVelocity.Y/(60*Dt):F4}");
    }

    // Euler-Eytelwein: a rope sliding over a locked wheel carries tension ratio e^(mu*theta). A heavy load hangs on the entry side; the exit side is pulled away with a small force.
    [Theory]
    [InlineData(0,0.25,1.5707963267948966)]
    [InlineData(1,0.25,3.141592653589793)]
    [InlineData(2,0.25,6.283185307179586)]
    [InlineData(3,0.25,12.566370614359172)]
    public void CapstanRatioAtTheSlipLimit(int kind,double mu,double theta)
    {
        var w=World2();
        const double r=.25;var centre=new Vector2D(0,5);
        var a=w.AddBox(new(.3f,.3f),at:new(-r,3),density:50);   // 4.5 kg hanging below the entry tangent
        Vector2D exit=kind==0?new(2.5,5+r):kind==1?new(r,3.5):new(-r,7.5);
        var b=w.AddBox(new(.2f,.2f),at:exit,density:10);
        Vector2F away=kind==0?new(1,0):kind==1?new(0,-1):new(0,1);
        int turns=kind==2?1:kind==3?2:0;
        var wheel=new P2.PulleyNodeDef{Kind=CableNodeKind.Wheel,Point=centre,Radius=(float)r,Direction=WrapDirection.Clockwise,Friction=(float)mu,Inertia=float.PositiveInfinity,ExtraTurns=turns};
        var rope=w.AddPulleySystem(new(){Nodes=[P2.PulleyNodeDef.Anchor(a.RawHandle,default),wheel,P2.PulleyNodeDef.Anchor(b.RawHandle,default)]});
        output.WriteLine($"kind {kind} wrap {rope.Wrap(1):F6} expected {theta:F6}");
        double ratio=0;float pull=.4f;
        for(int i=0;i<30;i++)
        {
            b.ApplyForce(new(away.X*pull,away.Y*pull+b.Mass*9.81f));
            w.Step(Dt);
            if(i==29){ratio=rope.Tension(0)/rope.Tension(1);output.WriteLine($"T0={rope.Tension(0):F4} T1={rope.Tension(1):F4} ratio={ratio:F5} expected={Math.Exp(mu*theta):F5} va={a.LinearVelocity.Y:F3} vb={b.LinearVelocity.X:F3},{b.LinearVelocity.Y:F3}");}
        }
    }

    // A block and tackle with n supporting strands: n = 2 or 4. The hauling end is a hanging weight pulled down with force F.
    private static (P2.PhysicsWorld2D World,P2.Body Load,P2.Body Haul,P2.PulleySystem Rope) Tackle2(int n,double loadMass,double haulMass,float friction=0)
    {
        var w=World2();const double R=.1,m=0,yF=6;
        double bx=m+(n==2?0:2*R);var block=w.AddBox(new(.9f,.3f),at:new(bx,3),density:(float)(loadMass/(.9*.3)));
        var haul=w.AddBox(new(.1f,.1f),at:new(m+(n==2?3:7)*R+(n==2?0:0),2.0),density:(float)(haulMass/.01));
        P2.PulleyNodeDef Pull(double x,double y,bool fixedPulley,WrapDirection d)=>new(){Kind=CableNodeKind.Wheel,Body=fixedPulley?default:block.RawHandle,Point=fixedPulley?new(x,y):new(x-bx,0),Radius=(float)R,Direction=d,Inertia=1e-3f,Friction=friction};
        var nodes=new System.Collections.Generic.List<P2.PulleyNodeDef>{P2.PulleyNodeDef.Anchor(default,new(m-R,yF))};
        nodes.Add(Pull(m,3,false,WrapDirection.CounterClockwise));
        nodes.Add(Pull(m+2*R,yF,true,WrapDirection.Clockwise));
        if(n==4){nodes.Add(Pull(m+4*R,3,false,WrapDirection.CounterClockwise));nodes.Add(Pull(m+6*R,yF,true,WrapDirection.Clockwise));}
        nodes.Add(P2.PulleyNodeDef.Anchor(haul.RawHandle,default));
        // the hauling end hangs below the last fixed pulley's descending tangent
        var rope=w.AddPulleySystem(new(){Nodes=nodes.ToArray()});
        return (w,block,haul,rope);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    public void TackleMechanicalAdvance(int n)
    {
        double M=10,mb=.5,g=9.81;
        var (w,load,haul,rope)=Tackle2(n,M,mb);
        output.WriteLine($"n={n} tackle: load {load.Mass} kg, spans {rope.SpanCount}, taut length {rope.TotalLength:F4}");
        // hold: F = M g / n - mb g keeps it still
        float hold=(float)(M*g/n-mb*g);
        for(int i=0;i<240;i++){haul.ApplyForce(new(0,-hold));w.Step(Dt);}
        output.WriteLine($"hold: load vy={load.LinearVelocity.Y:E3} y={load.Position.Y:F5} T={rope.Tension(1):F4} expected T={M*g/n:F4} haul vy={haul.LinearVelocity.Y:E3} stretch={rope.Stretch(1):E2}");
        // pull 5 % harder: a = (F + mb g - M g / n) / (mb n + M / n)
        float f2=hold*1.05f;double a=(f2+mb*g-M*g/n)/(mb*n+M/n);
        double v0=load.LinearVelocity.Y;
        for(int i=0;i<60;i++){haul.ApplyForce(new(0,-f2));w.Step(Dt);}
        output.WriteLine($"pull: measured a={(load.LinearVelocity.Y-v0)/(60*Dt):F5} expected {a:F5}; hauling end speed {-haul.LinearVelocity.Y:F4} = {n} x load speed {n*load.LinearVelocity.Y:F4}");
    }

    [Fact]
    public void WinchLiftsAtRatedSpeedAndHoldsWithTheBrake()
    {
        var w=World2();const double r=.2,g=9.81;
        var load=w.AddBox(new(.4f,.4f),at:new(-r,2),density:125);   // 20 kg
        var winch=w.AddWinch(default,new(0,6),(float)r,P2.RopeEnd.On(load,default),drumInertia:.5f,maxTorque:100);
        winch.SetMotor(0,-5,100);
        double t0=winch.FreeLength(0);
        for(int i=0;i<120;i++)w.Step(Dt);
        double v=load.LinearVelocity.Y,rate=-5*r;
        output.WriteLine($"lift: speed {v:F5} m/s (rated {-rate:F3}), free length {winch.FreeLength(0):F4} (was {t0:F4}), drum torque {winch.WheelTorque(0):F3} (load {load.Mass*g*r:F3}), tension {winch.Tension(0):F3}");
        winch.SetMotor(0,0,0);winch.SetBrake(0,60);
        double y0=load.Position.Y;double vmax=0;
        for(int i=0;i<240;i++){w.Step(Dt);vmax=Math.Max(vmax,Math.Abs(load.LinearVelocity.Y));}
        output.WriteLine($"hold with 60 N m brake (load needs {load.Mass*g*r:F2}): drift {load.Position.Y-y0:E3} m, max speed {vmax:E3}, brake torque {winch.DriveTorque(0):F3}");
        winch.SetBrake(0,20);
        for(int i=0;i<60;i++)w.Step(Dt);
        output.WriteLine($"weak 20 N m brake: speed {load.LinearVelocity.Y:F3} m/s, expected a={(load.Mass*g*r-20)/(load.Mass*r*r+.5)*r:F3}, measured a={load.LinearVelocity.Y/(60*Dt):F3}");
    }

    [Fact]
    public void ChainDriveRunsAtTheToothRatio()
    {
        var w=World2(0);
        var ground=w.AddBox(new(1,1),at:new(0,-5),type:BodyType.Static);
        double pitch=.02;int n1=20,n2=40;double r1=n1*pitch/(2*Math.PI),r2=n2*pitch/(2*Math.PI);
        var a=w.AddCircle((float)r1,at:new(0,0),density:200);var b=w.AddCircle((float)r2,at:new(.5,0),density:200);
        w.AddHinge(ground,a,new(0,0));w.AddHinge(ground,b,new(.5,0));
        var chain=w.AddChainDrive([new P2.Sprocket(a,default,n1),new P2.Sprocket(b,default,n2)],(float)pitch);
        output.WriteLine($"chain: spans {chain.SpanCount}, length {chain.TotalLength:F5} = {chain.TotalLength/pitch:F3} pitches, radii {r1:F5} {r2:F5}");
        double maxErr=0;
        for(int i=0;i<600;i++)
        {
            a.ApplyTorque(.05f);w.Step(Dt);
            if(i>60){double e=Math.Abs(b.AngularVelocity*n2-a.AngularVelocity*n1)/Math.Abs(a.AngularVelocity*n1);maxErr=Math.Max(maxErr,e);}
        }
        output.WriteLine($"w1={a.AngularVelocity:F4} w2={b.AngularVelocity:F4} ratio error max={maxErr:E3}; tensions {chain.Tension(0):F3} {chain.Tension(1):F3}, angle ratio {b.Angle/a.Angle:F6}");
    }
}
