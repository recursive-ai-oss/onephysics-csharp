using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    RodWitness IRodHost.Distance(int shape,Vector3D a,Vector3D b,float radius,float time)
    {
        int owner=Core.Shapes.Owner[shape];Rods.BodyCollisionPose(owner,time,out var position,out var rotation);
        var lc=new Vector3F(LocalCenter[2*owner],LocalCenter[2*owner+1],0);
        var origin=position-RodGeometry.Double(RodParticles.Rotate(rotation,lc));
        var inv=RodParticles.Conjugate(rotation);
        var p=RodParticles.Rotate(inv,RodGeometry.Float(a-origin));var q=RodParticles.Rotate(inv,RodGeometry.Float(b-origin));
        var capsule=RodGeometry.Length(b-a)<1e-5?Shape2D.CreateCircle(new(p.X,p.Y),radius):Shape2D.CreateCapsule(new(p.X,p.Y),new(q.X,q.Y),radius);
        SimplexCache2D cache=default;
        var distance=Distance2D.Compute(Geom[shape],capsule,Transform2.Identity,ref cache);
        var result=new RodWitness{Separation=distance.Distance,Convex=Geom[shape].Kind is ShapeKind2D.Circle or ShapeKind2D.Capsule or ShapeKind2D.Segment or ShapeKind2D.Polygon};
        if(distance.Distance>1e-7f)
        {
            var delta=distance.PointB-distance.PointA;float l=(float)Math.Sqrt(delta.X*delta.X+delta.Y*delta.Y);
            if(l>0)result.Normal=RodParticles.Rotate(rotation,new(delta.X/l,delta.Y/l,0));
            result.Surface=origin+RodGeometry.Double(RodParticles.Rotate(rotation,new(distance.PointA.X,distance.PointA.Y,0)));
            var core=origin+RodGeometry.Double(RodParticles.Rotate(rotation,new(distance.PointB.X,distance.PointB.Y,0)))+RodGeometry.Double(result.Normal*radius);
            result.Fraction=RodGeometry.Parameter(a,b,core);return result;
        }
        var manifold=Collide2D.Collide(Geom[shape],capsule,Transform2.Identity,0.001f);
        if(manifold.PointCount==0){result.Separation=float.PositiveInfinity;return result;}
        int best=0;if(manifold.PointCount>1&&manifold.Point1.Separation<manifold.Point0.Separation)best=1;
        var contact=manifold[best];result.Separation=contact.Separation;
        result.Normal=RodParticles.Rotate(rotation,new(manifold.Normal.X,manifold.Normal.Y,0));
        var midpoint=origin+RodGeometry.Double(RodParticles.Rotate(rotation,new(contact.Point.X,contact.Point.Y,0)));
        result.Surface=midpoint-RodGeometry.Double(result.Normal*(contact.Separation*.5f));
        result.Fraction=RodGeometry.Parameter(a,b,midpoint+RodGeometry.Double(result.Normal*(radius+contact.Separation*.5f)));
        return result;
    }
}
