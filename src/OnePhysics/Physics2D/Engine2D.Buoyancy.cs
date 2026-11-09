using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private AreaCutter2D? _areaCutter;
    internal float ApplyFieldBuoyancy(int b,FieldDefinition d,float dt)
    {
        var bodies=Core.Bodies;var pos=bodies.PositionOf(b);var rot=bodies.RotationOf(b);
        float c=rot[0],s=rot[1],lx=LocalCenter[2*b],ly=LocalCenter[2*b+1];
        var localNormal=new Vector2F(c*d.Axis.X+s*d.Axis.Y,-s*d.Axis.X+c*d.Axis.Y);
        double ox=pos[0]-(c*lx-s*ly),oy=pos[1]-(s*lx+c*ly);
        float offset=(float)(d.SurfaceHeight-d.Axis.X*ox-d.Axis.Y*oy);
        float totalArea=0,torque=0,dragBudget=Mass[b]/dt;
        Vector3F force=default;var v=bodies.LinearVelocityOf(b);float omega=bodies.AngularVelocityOf(b)[0];
        float vx=d.Flow.X-v[0],vy=d.Flow.Y-v[1],speed=float.Sqrt(vx*vx+vy*vy);
        for(int shape=ShapeHead[b];shape>=0;shape=NextShape[shape])
        {
            if(IsSensor(shape))continue;
            var cut=(_areaCutter??=new AreaCutter2D()).Cut(in Geom[shape],localNormal,offset);
            if(!(cut.Area>0))continue;
            totalArea+=cut.Area;
            float ax=c*(cut.Centroid.X-lx)-s*(cut.Centroid.Y-ly),ay=s*(cut.Centroid.X-lx)+c*(cut.Centroid.Y-ly);
            float massDisplaced=d.Density*cut.Area,factor=Fields.BuoyancyFactor(b);
            var buoyancy=new Vector3F(-Opt.Gravity.X*massDisplaced*factor,-Opt.Gravity.Y*massDisplaced*factor,0);
            torque+=ax*buoyancy.Y-ay*buoyancy.X;
            float drag=Math.Min(d.Drag*massDisplaced+0.5f*d.Density*d.QuadraticDrag*float.Sqrt(cut.Area)*speed,dragBudget);
            dragBudget-=drag;
            force+=buoyancy+new Vector3F(vx,vy,0)*drag;
        }
        if(totalArea>0)
        {
            float angularRate=d.AngularDrag/(1+dt*d.AngularDrag),invI=bodies.InverseInertia[b];
            if(invI>0)torque-=omega*angularRate/invI;
            FieldDefinition.CheckVector(force,"buoyancy force");FieldDefinition.Check(torque,"buoyancy torque");
            if(Fields.Wake(b,force*bodies.InverseMass[b],new(0,0,torque*invI),dt))
            {Force[2*b]+=force.X;Force[2*b+1]+=force.Y;Torque[b]+=torque;}
        }
        return totalArea;
    }
}
