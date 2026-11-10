#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType, CharacterWaterState
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Character, CharacterDef

namespace GuideExamples;

/// <summary>The code block of docs/guide/water.md about swimming.</summary>
public static class Swimming
{
    public static (bool Swimming, CharacterWaterState State, int Events, ulong Hash) Run()
    {
        int events = 0;
        float dt = 1f / 60f;
        var world = new PhysicsWorld3D();
        world.AddWater(2.0);
        var stroke = new Vector3F(1f, 0f, 0f);
        var walkAndGravity = new Vector3F(1f, -0.2f, 0f);
        var def = new CharacterDef { SwimDensity = 900f, CanClimb = true };      // a person floats; try 1100 for one that sinks
        var swimmer = world.AddCharacter(new Vector3D(0, 2, 5), def);
        world.CharacterWaterChanged += e => events++;                            // EnteredWater, StartedSwimming, StoppedSwimming, LeftWater

        for (int i = 0; i < 300; i++)
        {
            if (swimmer.IsSwimming)  swimmer.Move(stroke * dt, dt);                  // while swimming, Move takes a stroke relative to the water, not a step
            else                     swimmer.Move(walkAndGravity * dt, dt);          // wading slows the walk by the depth; gravity is still yours
            world.Step(dt);
        }

        return (swimmer.IsSwimming, swimmer.WaterState, events, world.ComputeHash());
    }
}
