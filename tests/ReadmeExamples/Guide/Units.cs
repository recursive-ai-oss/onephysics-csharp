using System;
using RecursiveAI.OnePhysics;            // LengthUnit
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, WorldOptions2D
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D, WorldOptions3D

namespace GuideExamples;

/// <summary>The code block of the length-unit section of docs/guide/units.md.</summary>
public static class Units
{
    public static (float Gravity3D, float Gravity2D) Run()
    {
        var world = new PhysicsWorld3D(new WorldOptions3D { LengthUnit = LengthUnit.Millimetres });
        var pixels = new PhysicsWorld2D(new WorldOptions2D { LengthUnit = LengthUnit.Pixels(32) });   // 32 pixels to the metre
        return (world.Options.Gravity.Y, pixels.Options.Gravity.Y);
    }
}
