using System;
using RecursiveAI.OneMath.Physics;
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
    internal Vector3F FieldAngularTorque(int b,Vector3F alpha)
    {
        var local=Inertia.AsSpan(6*b,6);
        var tensor=InertiaF.Rotate(new Matrix3x3F(local[0],local[1],local[2],local[1],local[3],local[4],local[2],local[4],local[5]),RotationOfBody(b));
        return new(tensor.M11*alpha.X+tensor.M12*alpha.Y+tensor.M13*alpha.Z,tensor.M21*alpha.X+tensor.M22*alpha.Y+tensor.M23*alpha.Z,tensor.M31*alpha.X+tensor.M32*alpha.Y+tensor.M33*alpha.Z);
    }
}
