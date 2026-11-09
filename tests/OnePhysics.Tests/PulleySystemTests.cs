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
        var rope=w.AddRopeOverPulleys(P2.RopeEnd.On(a,default),[new P2.Pulley(new(0,5),.25f,clockwise:true,inertia:1e-4f)],P2.RopeEnd.On(b,default));
        double total=rope.TotalLength;
        for(int i=0;i<30;i++)w.Step(Dt);
        double expected=(a.Mass-b.Mass)/(a.Mass+b.Mass)*9.81,measured=-a.LinearVelocity.Y/(30*Dt);
        double tension=2*a.Mass*b.Mass/(a.Mass+b.Mass)*9.81;
        output.WriteLine($"Atwood a={measured:F5} (exact {expected:F5}), T={rope.Tension(0):F4}/{rope.Tension(1):F4} (exact {tension:F4}), length drift {rope.TotalLength-total:E2}");
        Assert.InRange(measured/expected,.99,1.01);Assert.InRange(rope.Tension(0)/tension,.99,1.01);Assert.InRange(rope.TotalLength-total,-1e-9,1e-9);
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
        Assert.InRange(rope.Wrap(1),theta-1e-6,theta+1e-6);
        double ratio=0;
        for(int i=0;i<30;i++)
        {
            b.ApplyForce(new(away.X*.4f,away.Y*.4f+b.Mass*9.81f));
            w.Step(Dt);
            if(i==29)ratio=rope.Tension(0)/rope.Tension(1);
        }
        double expected=Math.Exp(mu*theta);
        output.WriteLine($"capstan wrap {theta:F4} rad: T ratio {ratio:F5}, e^(mu theta) {expected:F5}, error {(ratio/expected-1):E2}, load still sliding at {a.LinearVelocity.Y:F3} m/s");
        Assert.InRange(ratio/expected,.995,1.005);Assert.True(a.LinearVelocity.Y<-.1);
    }

    // A block and tackle with n supporting strands: n = 2 or 4. The hauling end is a hanging weight pulled down with force F.
    private static (P2.PhysicsWorld2D World,P2.Body Load,P2.Body Haul,P2.PulleySystem Rope) Tackle2(int n,double loadMass,double haulMass,float friction=0)
    {
        var w=World2();const double R=.1,m=0,yF=6;
        double bx=m+(n==2?0:2*R);var block=w.AddBox(new(.9f,.3f),at:new(bx,3),density:(float)(loadMass/(.9*.3)));
        var haul=w.AddBox(new(.1f,.1f),at:new(m+(n==2?3:7)*R,2.0),density:(float)(haulMass/.01));
        P2.PulleyNodeDef Pull(double x,double y,bool fixedPulley,WrapDirection d)=>new(){Kind=CableNodeKind.Wheel,Body=fixedPulley?default:block.RawHandle,Point=fixedPulley?new(x,y):new(x-bx,0),Radius=(float)R,Direction=d,Inertia=1e-3f,Friction=friction};
        var nodes=new System.Collections.Generic.List<P2.PulleyNodeDef>{P2.PulleyNodeDef.Anchor(default,new(m-R,yF))};
        nodes.Add(Pull(m,3,false,WrapDirection.CounterClockwise));
        nodes.Add(Pull(m+2*R,yF,true,WrapDirection.Clockwise));
        if(n==4){nodes.Add(Pull(m+4*R,3,false,WrapDirection.CounterClockwise));nodes.Add(Pull(m+6*R,yF,true,WrapDirection.Clockwise));}
        nodes.Add(P2.PulleyNodeDef.Anchor(haul.RawHandle,default));
        var rope=w.AddPulleySystem(new(){Nodes=nodes.ToArray()});
        return (w,block,haul,rope);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    public void TackleMechanicalAdvantage(int n)
    {
        double M=10,mb=.5,g=9.81;
        var (w,load,haul,rope)=Tackle2(n,M,mb);
        float hold=(float)(M*g/n-mb*g);
        for(int i=0;i<240;i++){haul.ApplyForce(new(0,-hold));w.Step(Dt);}
        output.WriteLine($"{n}:1 hold: load v={load.LinearVelocity.Y:E2} m/s, tension {rope.Tension(1):F4} (exact {M*g/n:F4})");
        Assert.InRange(Math.Abs(load.LinearVelocity.Y),0,1e-4);Assert.InRange(rope.Tension(1)/(M*g/n),.999,1.001);
        float f2=hold*1.05f;double a=(f2+mb*g-M*g/n)/(mb*n+M/n);
        double v0=load.LinearVelocity.Y;
        for(int i=0;i<60;i++){haul.ApplyForce(new(0,-f2));w.Step(Dt);}
        double measured=(load.LinearVelocity.Y-v0)/(60*Dt);
        output.WriteLine($"{n}:1 pull: a={measured:F5} (exact {a:F5}); hauling end speed {-haul.LinearVelocity.Y:F4} = {n} x load speed {n*load.LinearVelocity.Y:F4}");
        Assert.InRange(measured/a,.99,1.01);Assert.InRange(-haul.LinearVelocity.Y/(n*load.LinearVelocity.Y),.999,1.001);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    public void TackleLengthStaysConservedOverTenThousandSteps(int n)
    {
        var (w,load,haul,rope)=Tackle2(n,10,.5);
        double total=rope.TotalLength,worst=0,hold=10*9.81/n-.5*9.81;
        for(int i=0;i<10000;i++)
        {
            var (_,c)=Math64.SinCos(i*Dt*2.0);
            haul.ApplyForce(new(0,-(float)(hold*(1+.1*c))));w.Step(Dt);
            for(int k=0;k<rope.SpanCount;k++)worst=Math.Max(worst,Math.Abs(rope.Stretch(k)));
        }
        output.WriteLine($"{n}:1 tackle, 10000 steps: material length drift {rope.TotalLength-total:E2} m, largest span stretch/slack {worst:E2} m, load y {load.Position.Y:F3}");
        Assert.InRange(rope.TotalLength-total,-1e-9,1e-9);Assert.InRange(worst,0,2e-3);
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
        double v=load.LinearVelocity.Y;
        output.WriteLine($"lift: speed {v:F5} m/s (rated {5*r:F3}), free length {winch.FreeLength(0):F4} (was {t0:F4}), drum torque {winch.WheelTorque(0):F3} (load {load.Mass*g*r:F3})");
        Assert.InRange(v,5*r*.99,5*r*1.01);Assert.InRange(t0-winch.FreeLength(0),1.9,2.1);
        winch.SetMotor(0,0,0);winch.SetBrake(0,60);
        for(int i=0;i<90;i++)w.Step(Dt);
        double y0=load.Position.Y,vmax=0;
        for(int i=0;i<240;i++){w.Step(Dt);vmax=Math.Max(vmax,Math.Abs(load.LinearVelocity.Y));}
        output.WriteLine($"hold with 60 N m brake (load needs {load.Mass*g*r:F2}): drift {load.Position.Y-y0:E3} m, max speed {vmax:E3}, brake torque {winch.DriveTorque(0):F3}");
        Assert.InRange(Math.Abs(load.Position.Y-y0),0,1e-3);Assert.InRange(vmax,0,1e-2);
        winch.SetBrake(0,20);
        double v1=load.LinearVelocity.Y;
        for(int i=0;i<60;i++)w.Step(Dt);
        double expected=(load.Mass*g*r-20)/(load.Mass*r*r+.5)*r,measured=-(load.LinearVelocity.Y-v1)/(60*Dt);
        output.WriteLine($"weak 20 N m brake: a={measured:F4}, expected {expected:F4}");
        Assert.InRange(measured/expected,.98,1.02);
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
        double links=chain.TotalLength/pitch;Assert.InRange(links-2*Math.Round(links/2),-1e-4,1e-4);   // an even number of links
        double maxErr=0,total=chain.TotalLength;
        for(int i=0;i<600;i++)
        {
            a.ApplyTorque(.05f);w.Step(Dt);
            if(i>60)maxErr=Math.Max(maxErr,Math.Abs(b.AngularVelocity*n2-a.AngularVelocity*n1)/Math.Abs(a.AngularVelocity*n1));
        }
        output.WriteLine($"chain {n1}:{n2}: w1={a.AngularVelocity:F4} w2={b.AngularVelocity:F4}, speed-ratio error max {maxErr:E2}, tight tension {chain.Tension(1):F3}, slack span tension {chain.Tension(0):F3}, sag {chain.Sag(0):F3} m, length drift {chain.TotalLength-total:E2}");
        Assert.InRange(maxErr,0,1e-5);Assert.Equal(0,chain.Tension(0));Assert.InRange(chain.TotalLength-total,-1e-9,1e-9);Assert.True(chain.Sag(0)>0.01);
    }

    // Capacity of a belt: with its slack side held at the preload T0, the tight side reaches e^(mu*pi) T0 on equal wheels, so a locked wheel takes R (e^(mu*pi) - 1) T0 of torque before the driving wheel slips.
    [Fact]
    public void BeltSlipsAtThePredictedTorque()
    {
        var w=World2(0);
        var ground=w.AddBox(new(1,1),at:new(0,-5),type:BodyType.Static);
        const float R=.1f,mu=.3f,t0=20;
        var a=w.AddCircle(R,at:new(0,0),density:100);var lockedBody=w.AddCircle(R,at:new(.6,0),type:BodyType.Static);
        w.AddHinge(ground,a,new(0,0));
        var belt=w.AddBelt([new P2.Pulley(new(0,0),R,a),new P2.Pulley(new(0,0),R,lockedBody)],friction:mu,preload:t0);
        double e=Math.Exp(mu*Math.PI),capacity=R*(e-1)*t0;
        // first below capacity: the locked wheel holds the driver
        for(int i=0;i<120;i++){a.ApplyTorque((float)(.9*capacity));w.Step(Dt);}
        double held=Math.Abs(a.AngularVelocity);
        for(int i=0;i<60;i++){a.ApplyTorque((float)(1.2*capacity));w.Step(Dt);}
        double slipping=a.AngularVelocity,alpha=(1.2*capacity-capacity)/(a.Mass*R*R/2);
        output.WriteLine($"belt capacity predicted {capacity:F4} N m: at 0.9x speed {held:E2}; at 1.2x torque on A is {belt.WheelTorque(0):F4}, spin {slipping:F3} rad/s after 1 s (a={alpha:F3}), tensions {belt.Tension(0):F3} {belt.Tension(1):F3}");
        Assert.InRange(held,0,1e-3);Assert.InRange(Math.Abs(belt.WheelTorque(0))/capacity,.99,1.01);
    }

    [Fact]
    public void RingDown()
    {
        var w=World2();
        var load=w.AddBox(new(.4f,.4f),at:new(0,3),density:625);   // 100 kg
        var rope=w.AddRopeOverPulleys(P2.RopeEnd.At(new(0,6)),[],P2.RopeEnd.On(load,default));
        rope.AdjustLength(-.05);
        string line="";
        for(int i=0;i<120;i++){w.Step(Dt);if(i%4==0)line+=$"{load.Position.Y-(3):F4} ";}
        output.WriteLine("ring-down y-offsets every 4 steps: "+line);
    }
}
