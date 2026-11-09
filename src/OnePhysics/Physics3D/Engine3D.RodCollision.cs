using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Rod sweeps run on pool threads, so the scratch objects belong to the thread, not the world.
    [ThreadStatic] private static ManifoldGenerator3D? t_rodManifolds;
    [ThreadStatic] private static TriangleShapeBuffer3D? t_rodTriangle;
    RodWitness IRodHost.Distance(int shape,Vector3D a,Vector3D b,float radius,float time)
    {
        int owner=Core.Shapes.Owner[shape];Rods.BodyCollisionPose(owner,time,out var body,out var bodyRotation);
        var local=Local[shape];var rotation=bodyRotation*local.Rotation;
        var origin=body+RodGeometry.Double(RodParticles.Rotate(bodyRotation,local.Translation-LocalCenterOf(owner)));
        var g=Geoms[GeomOf[shape]]!;
        // A sphere is far the commonest shape a rod meets (heads, balls, pulleys), and while apart its distance to a segment is closed-form: the segment point nearest the centre.
        if(g.IsConvex&&g.Convex.Kind==ConvexShapeKind3D.Sphere)
        {
            var axis=b-a;double axisSquared=RodGeometry.Dot(axis,axis);
            double along=axisSquared>1e-24?Math.Clamp(RodGeometry.Dot(origin-a,axis)/axisSquared,0,1):0;
            var offset=a+axis*along-origin;double distance=RodGeometry.Length(offset);
            double gap=distance-g.Convex.ConvexRadius-radius;
            if(gap>1e-7&&distance>1e-12)
            {
                var unit=offset/distance;var normal=RodGeometry.Float(unit);
                return new RodWitness{Separation=(float)gap,Normal=normal,Surface=origin+unit*g.Convex.ConvexRadius,Fraction=(float)along,Convex=true};
            }
        }
        var delta=b-a;float length=(float)RodGeometry.Length(delta);
        var tangent=length>1e-10f?RodGeometry.Float(delta/length):new Vector3F(0,1,0);
        var capsuleRotation=tangent.Y<-.999999f?new QuaternionF(1,0,0,0):RodParticles.Unit(new(tangent.Z,0,-tangent.X,1+tangent.Y));
        var capsule=length>1e-10f?ConvexShape3D.Capsule(length*.5f,radius):ConvexShape3D.Sphere(radius);
        var center=(a+b)*.5;
        RodWitness best=new(){Separation=float.PositiveInfinity};
        if(g.IsConvex){var single=RodLeaf(g.Convex,origin,rotation,capsule,center,capsuleRotation,a,b,radius,0);single.Convex=true;return single;}
        if(g.Compound!=null)
        {
            var compound=g.Compound;
            for(int i=0;i<compound.SlotCount;i++)
            {
                if(!compound.IsLive(i))continue;var child=compound.GetChild(i);
                var hit=RodLeaf(child.Shape,origin+RodGeometry.Double(RodParticles.Rotate(rotation,child.Local.Translation)),rotation*child.Local.Rotation,capsule,center,capsuleRotation,a,b,radius,i);
                if(hit.Separation<best.Separation)best=hit;
            }
        }
        else if(g.Soup!=null)
        {
            t_rodTriangle??=new TriangleShapeBuffer3D();
            // Triangle ids are stable and include height-field holes. Broad rejection precedes expensive convex distance.
            var inv=RodParticles.Conjugate(rotation);var p=RodParticles.Rotate(inv,RodGeometry.Float(a-origin));var q=RodParticles.Rotate(inv,RodGeometry.Float(b-origin));
            float limit=float.PositiveInfinity;
            for(int i=0;i<g.Soup.TriangleIdCount;i++)
            {
                if(!g.Soup.TryGetTriangle(i,out var t))continue;
                float dx=Math.Max(0,Math.Max(Math.Min(t.V0.X,Math.Min(t.V1.X,t.V2.X))-Math.Max(p.X,q.X),Math.Min(p.X,q.X)-Math.Max(t.V0.X,Math.Max(t.V1.X,t.V2.X))));
                float dy=Math.Max(0,Math.Max(Math.Min(t.V0.Y,Math.Min(t.V1.Y,t.V2.Y))-Math.Max(p.Y,q.Y),Math.Min(p.Y,q.Y)-Math.Max(t.V0.Y,Math.Max(t.V1.Y,t.V2.Y))));
                float dz=Math.Max(0,Math.Max(Math.Min(t.V0.Z,Math.Min(t.V1.Z,t.V2.Z))-Math.Max(p.Z,q.Z),Math.Min(p.Z,q.Z)-Math.Max(t.V0.Z,Math.Max(t.V1.Z,t.V2.Z))));
                if(dx*dx+dy*dy+dz*dz>limit*limit)continue;
                var hit=RodLeaf(t_rodTriangle.Set(t.V0,t.V1,t.V2),origin,rotation,capsule,center,capsuleRotation,a,b,radius,i);
                if(hit.Separation<best.Separation){best=hit;limit=Math.Max(0,hit.Separation)+2*radius;}
            }
        }
        return best;
    }
    private static RodWitness RodLeaf(in ConvexShape3D leaf,Vector3D origin,QuaternionF rotation,in ConvexShape3D capsule,Vector3D center,QuaternionF capsuleRotation,Vector3D a,Vector3D b,float radius,int subshape)
    {
        var inv=RodParticles.Conjugate(rotation);var transform=new RigidTransformF(inv*capsuleRotation,RodParticles.Rotate(inv,RodGeometry.Float(center-origin)));
        var distance=Gjk3D.Distance(leaf,capsule,transform);
        var result=new RodWitness{Separation=distance.Distance,Subshape=subshape};
        if(!distance.Overlapping&&distance.Distance>1e-7f)
        {
            result.Normal=RodParticles.Rotate(rotation,distance.Normal);
            result.Surface=origin+RodGeometry.Double(RodParticles.Rotate(rotation,distance.PointA));
            var core=origin+RodGeometry.Double(RodParticles.Rotate(rotation,distance.PointB))+RodGeometry.Double(result.Normal*radius);
            result.Fraction=RodGeometry.Parameter(a,b,core);return result;
        }
        t_rodManifolds??=new ManifoldGenerator3D();var manifold=t_rodManifolds.Collide(leaf,capsule,transform,0.001f);
        if(manifold.PointCount==0){result.Separation=float.PositiveInfinity;return result;}
        int best=0;for(int i=1;i<manifold.PointCount;i++)if(manifold[i].Separation<manifold[best].Separation)best=i;
        var contact=manifold[best];result.Separation=contact.Separation;result.Normal=RodParticles.Rotate(rotation,manifold.Normal);
        var midpoint=origin+RodGeometry.Double(RodParticles.Rotate(rotation,contact.Point));
        result.Surface=midpoint-RodGeometry.Double(result.Normal*(contact.Separation*.5f));
        result.Fraction=RodGeometry.Parameter(a,b,midpoint+RodGeometry.Double(result.Normal*(radius+contact.Separation*.5f)));
        return result;
    }
}
