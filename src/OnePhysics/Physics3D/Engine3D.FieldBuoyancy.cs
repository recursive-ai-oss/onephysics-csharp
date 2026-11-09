using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    private readonly FluidDef _fieldFluid=new();
    private void ApplyFieldBuoyancy(int b,FieldDefinition d,float dt)
    {
        var fluid=_fieldFluid;
        fluid.SurfaceNormal=d.Axis;fluid.SurfaceOffset=d.SurfaceHeight;fluid.RegionMin=d.Minimum;fluid.RegionMax=d.Maximum;
        fluid.Density=d.Density;fluid.Flow=d.Flow;fluid.ViscousDamping=d.Drag;fluid.LinearDrag=d.QuadraticDrag;fluid.AngularDrag=d.AngularDrag;
        var beforeForce=new Vector3F(Force[3*b],Force[3*b+1],Force[3*b+2]);
        var beforeTorque=new Vector3F(Torque[3*b],Torque[3*b+1],Torque[3*b+2]);
        ApplyFluidToBody(b,fluid,dt);
        if(!Core.Bodies.IsAwake(b))
        {
            var force=new Vector3F(Force[3*b],Force[3*b+1],Force[3*b+2])-beforeForce;
            var torque=new Vector3F(Torque[3*b],Torque[3*b+1],Torque[3*b+2])-beforeTorque;
            RefreshWorldInertia(b);
            if(!Fields.Wake(b,force*Core.Bodies.InverseMass[b],ApplyWorldInertia(b,torque),dt))
            {
                Force[3*b]=beforeForce.X;Force[3*b+1]=beforeForce.Y;Force[3*b+2]=beforeForce.Z;
                Torque[3*b]=beforeTorque.X;Torque[3*b+1]=beforeTorque.Y;Torque[3*b+2]=beforeTorque.Z;
            }
        }
    }
    /// <summary>World-frame torque of an angular acceleration: R · I_local · Rᵀ · alpha, built from rotations so no matrix object is allocated.</summary>
    internal Vector3F FieldAngularTorque(int b,Vector3F alpha)
    {
        var local=Inertia.AsSpan(6*b,6);
        var q=RotationOfBody(b);
        var a=q.Conjugated().Rotate(alpha);
        var t=new Vector3F(local[0]*a.X+local[1]*a.Y+local[2]*a.Z,local[1]*a.X+local[3]*a.Y+local[4]*a.Z,local[2]*a.X+local[4]*a.Y+local[5]*a.Z);
        return q.Rotate(t);
    }
}
