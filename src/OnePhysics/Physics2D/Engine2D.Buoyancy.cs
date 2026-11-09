using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private AreaCutter2D? _areaCutter;
    /// <summary>How far ahead of the step's start the buoyancy force is evaluated: the mean extra travel of a substep scheme, (n-1)/(2n) of a step.</summary>
    internal float BuoyancyLead(float dt){int n=Math.Max(Opt.Substeps,1);return dt*(n-1)/(2f*n);}
    internal float ApplyFieldBuoyancy(int b,FieldDefinition d,float dt)
    {
        var bodies=Core.Bodies;var pos=bodies.PositionOf(b);var rot=bodies.RotationOf(b);
        var v=bodies.LinearVelocityOf(b);float omega=bodies.AngularVelocityOf(b)[0];
        // The force is held fixed while the substeps move the body, which is explicit in position and adds energy to a
        // floating body without damping. Cutting the hull where the substeps will have carried it by mid-step removes that.
        float lead=BuoyancyLead(dt);
        float c=rot[0]-rot[1]*omega*lead,s=rot[1]+rot[0]*omega*lead,norm=float.Sqrt(c*c+s*s);
        if(norm>0&&float.IsFinite(norm)){c/=norm;s/=norm;}else{c=rot[0];s=rot[1];}
        float lx=LocalCenter[2*b],ly=LocalCenter[2*b+1];
        var localNormal=new Vector2F(c*d.Axis.X+s*d.Axis.Y,-s*d.Axis.X+c*d.Axis.Y);
        double ox=pos[0]+(double)v[0]*lead-(c*lx-s*ly),oy=pos[1]+(double)v[1]*lead-(s*lx+c*ly);
        float offset=(float)(d.SurfaceHeight-d.Axis.X*ox-d.Axis.Y*oy);
        // Restored state may be finite but not a rotation: keep the plane normal exactly unit, and a plane that cannot be formed cuts nothing.
        float nl=localNormal.X*localNormal.X+localNormal.Y*localNormal.Y;
        if(!float.IsFinite(offset)||!(nl>0)||!float.IsFinite(nl))return 0;
        if(Math.Abs(nl-1)>1e-6f){float inv=1/float.Sqrt(nl);localNormal=new Vector2F(localNormal.X*inv,localNormal.Y*inv);}
        float totalArea=0,torque=0,dragBudget=Mass[b]/dt;
        Vector3F force=default;
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
            // A boat's hull has its own drag and heave damping, so the field's linear drag is left out for it.
            float drag=_actuators!=null&&_actuators.OwnsHullDrag(b)?0f:Math.Min(d.Drag*massDisplaced+0.5f*d.Density*d.QuadraticDrag*float.Sqrt(cut.Area)*speed,dragBudget);
            dragBudget-=drag;
            force+=buoyancy+new Vector3F(vx,vy,0)*drag;
        }
        if(totalArea>0)
        {
            _actuators?.AddSubmerged(b,totalArea);
            float angularRate=d.AngularDrag/(1+dt*d.AngularDrag),invI=bodies.InverseInertia[b];
            if(invI>0)torque-=omega*angularRate/invI;
            // A heavy, fast-spinning body legitimately produces a large torque, so only non-finite values (state that is already broken) are refused, and by doing nothing rather than by throwing mid-step.
            if(!float.IsFinite(force.X)||!float.IsFinite(force.Y)||!float.IsFinite(torque))return totalArea;
            if(Fields.Wake(b,force*bodies.InverseMass[b],new(0,0,torque*invI),dt))
            {Force[2*b]+=force.X;Force[2*b+1]+=force.Y;Torque[b]+=torque;}
        }
        return totalArea;
    }
}
