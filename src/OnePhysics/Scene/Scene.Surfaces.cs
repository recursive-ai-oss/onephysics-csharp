using System;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Scene;

public sealed partial class Scene
{
    private static SurfaceProperties ReadSurface(string[] fields)
    {
        float F(int i) => float.Parse(fields[i], NumberStyles.Float, CultureInfo.InvariantCulture);
        return new SurfaceProperties { Velocity = new(F(1), F(2), F(3)), OneWayDirection = new(F(4), F(5), F(6)), FrictionAxis = new(F(7), F(8), F(9)), LongitudinalFriction = F(10), TransverseFriction = F(11), SpinningFriction = F(12) };
    }
    private static void WriteSurface(StringBuilder builder, SurfaceProperties surface)
    {
        if (surface.IsDefault) return;
        builder.Append("surface");
        void V(Vector3F vector) { builder.Append(' ').Append(F(vector.X)).Append(' ').Append(F(vector.Y)).Append(' ').Append(F(vector.Z)); }
        V(surface.Velocity); V(surface.OneWayDirection); V(surface.FrictionAxis);
        builder.Append(' ').Append(F(surface.LongitudinalFriction)).Append(' ').Append(F(surface.TransverseFriction)).Append(' ').Append(F(surface.SpinningFriction)).Append('\n');
    }
}
