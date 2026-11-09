using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class VehicleState
{
    internal VehicleState CloneForFork()
    {
        var c = (VehicleState)MemberwiseClone();
        c.Sprung = (float[])Sprung.Clone();
        c.Omega = (float[])Omega.Clone();
        c.Spin = (float[])Spin.Clone();
        c.Steer = (float[])Steer.Clone();
        c.Length = (float[])Length.Clone();
        c.SpringForce = (float[])SpringForce.Clone();
        c.SlipRatio = (float[])SlipRatio.Clone();
        c.SlipAngle = (float[])SlipAngle.Clone();
        c.Fx = (float[])Fx.Clone();
        c.Fy = (float[])Fy.Clone();
        c.Nx = (float[])Nx.Clone();
        c.Ny = (float[])Ny.Clone();
        c.Nz = (float[])Nz.Clone();
        c.Cx = (double[])Cx.Clone();
        c.Cy = (double[])Cy.Clone();
        c.Cz = (double[])Cz.Clone();
        c.Contact = (byte[])Contact.Clone();
        c.GroundBody = (int[])GroundBody.Clone();
        c.GroundGen = (int[])GroundGen.Clone();
        c.Curve = (float[])Curve.Clone();
        c.Ratios = (float[])Ratios.Clone();
        c.CastShapes = (ConvexShape3D[])CastShapes.Clone();
        c.GroundShape = (int[])GroundShape.Clone();
        c.Torque = (float[])Torque.Clone();
        c.Coef = (float[])Coef.Clone();
        c.Compression = (float[])Compression.Clone();
        c.Stop = (float[])Stop.Clone();
        c.ExtraTorque = (float[])ExtraTorque.Clone();
        c.Def = Def.Clone();
        c.Track = Track?.Clone();
        c.Bike = Bike?.Clone();
        return c;
    }
}
