using System;
using RecursiveAI.OneMath;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodTwistTests(ITestOutputHelper output)
{
    [Fact]
    public void OverTwistDevelopsSpatialWrithe()
    {
        const int n=32;var points=new Vector3D[n+1];
        for(int i=0;i<=n;i++){double t=(double)i/n;points[i]=new(t,.08*Math.Sin(t*Math.PI),i==0||i==n?0:1e-4*Math.Sin(t*2*Math.PI));}
        var world=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});
        var rod=world.AddRod(new(){Points=points,Material=new(1e6f,4e5f,.01f,1000),PinnedParticles=[0,n],ClampedSegments=[0,n-1],Damping=5,Substeps=8,Iterations=4});
        rod.SetTarget(n,new(.6,0,0));
        var rest=rod.Particles.FrameData[n-1];
        for(int step=0;step<720;step++)
        {
            double angle=Math.Min(step,360)*(6*Math.PI/360);var (sin,cos)=Math64.SinCos(angle*.5);
            rod.SetFrameTarget(n-1,rest*new QuaternionF((float)sin,0,0,(float)cos));world.Step(1f/120);
        }
        double writhe=Writhe(rod);double z=0;for(int i=0;i<=n;i++)z=Math.Max(z,Math.Abs(rod.GetPosition(i).Z));
        output.WriteLine($"over-twist writhe={writhe:R} maxZ={z:R} stretch={rod.MaximumStretch:R}");
        Assert.True(Math.Abs(writhe)>.1,$"No measured writhe: {writhe:R}");
    }
    internal static double Writhe(P3.Rod rod)
    {
        double total=0;
        for(int i=0;i<rod.ParticleCount-1;i++)
        {
            var a=rod.GetPosition(i);var b=rod.GetPosition(i+1);var tangent=b-a;
            for(int j=i+2;j<rod.ParticleCount-1;j++)
            {
                var c=rod.GetPosition(j);var d=rod.GetPosition(j+1);var delta=(a+b-c-d)*.5;double length=delta.Length();
                if(length>1e-8)total+=Vector3D.Dot(Vector3D.Cross(tangent,d-c),delta)/(length*length*length);
            }
        }
        return total/(2*Math.PI);
    }
    [Fact]
    public void HeldStraightRodRetainsMaterialTwistAtRest()
    {
        const int n=16;const double target=Math.PI/2;
        var points=new Vector3D[n+1];for(int i=0;i<=n;i++)points[i]=new((double)i/n,0,0);
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});var rod=world.AddRod(new(){Points=points,PinnedParticles=[0,n],ClampedSegments=[0,n-1],Damping=2});
        for(int i=0;i<n;i++){var (sin,cos)=Math64.SinCos(.5*target*i/(n-1));rod.Particles.Frames[i]=new((float)sin,0,0,(float)cos);}
        for(int step=0;step<300;step++)world.Step(1f/120);
        double twist=0;
        for(int i=0;i<n-1;i++)
        { var a=rod.Particles.Frames[i];var b=rod.Particles.Frames[i+1];var q=new QuaternionF(-a.X,-a.Y,-a.Z,a.W)*b;twist+=2*Math.Atan2(q.X,q.W); }
        output.WriteLine($"held twist={twist:R} rad expected={target:R} maxStretch={rod.MaximumStretch:R}");Assert.InRange(Math.Abs(twist-target),0,1e-4);
    }

}
