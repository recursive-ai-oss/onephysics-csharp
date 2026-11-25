using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Interest messages: whatever bytes arrive after a valid snapshot are either refused with a typed error naming the field, or applied; the client then hashes and steps without failing.
/// </summary>
internal static class FuzzInterest
{
    private static byte[]? _first2, _first3;

    public static void Run(byte[] bytes)
    {
        int dimension = bytes.Length >= 12 ? BitConverter.ToInt32(bytes, 8) : 3;
        try
        {
            if (dimension == 2)
            {
                var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 41 });
                var client = world.CreateInterestClient();
                client.Apply(_first2 ??= Messages(2, 1)[0]);
                client.Apply(bytes, 1 << 22);
                client.ComputeSharedHash();
                for (int i = 0; i < 3; i++) { world.Step(1f / 60f); }
            }
            else
            {
                var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 41 });
                var client = world.CreateInterestClient();
                client.Apply(_first3 ??= Messages(3, 1)[0]);
                client.Apply(bytes, 1 << 22);
                client.ComputeSharedHash();
                for (int i = 0; i < 3; i++) { world.Step(1f / 60f); }
            }
        }
        catch (PhysicsException) { }
        catch (ArgumentException) { }
    }

    /// <summary>Messages 0..count of a small authority world: a stack, a ragdoll and a joint chain with the region's edge through them; with <paramref name="entities"/> also a car and a character.</summary>
    public static byte[][] Messages(int dimension, int count, bool entities = false)
    {
        var result = new byte[count + 1][];
        if (dimension == 2)
        {
            var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 40 });
            world.AddBox(new(40, 1), at: new(0, -0.5), type: BodyType.Static);
            for (int i = 0; i < 4; i++) { world.AddBox(new(1, 1), at: new(0.02 * i, 0.5 + 1.001 * i)); }
            world.AddRagdoll(RecursiveAI.OnePhysics.Physics2D.RagdollDef.Humanoid, new Vector2D(5, 1.2), 1.5f);
            if (entities) { world.AddCar(new Vector2D(-6, 0)); world.AddCharacter(new Vector2D(1, 1)); }
            var pivot = world.AddBox(new(0.2f, 0.2f), at: new(-4, 3), type: BodyType.Static);
            var prev = pivot;
            for (int i = 0; i < 4; i++)
            {
                var link = world.AddBox(new(1, 0.2f), at: new(-3.5 + i, 3));
                world.CreateJoint(new RevoluteJointDef { BodyA = prev, BodyB = link, LocalAnchorA = new Vector2F(i == 0 ? 0f : 0.5f, 0), LocalAnchorB = new Vector2F(-0.5f, 0) });
                prev = link;
            }

            var writer = world.CreateInterestWriter();
            for (int m = 0; m <= count; m++)
            {
                world.Step(1f / 60f);
                result[m] = writer.Write(new InterestRegion().AddBox(new Vector2D(-8 + m, -2), new Vector2D(2 + m, 20)));
            }
        }
        else
        {
            var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 40 });
            world.AddBox(new(40, 1, 40), at: new(0, -0.5, 0), type: BodyType.Static);
            for (int i = 0; i < 4; i++) { world.AddBox(new(1, 1, 1), at: new(0.02 * i, 0.5 + 1.001 * i, 0)); }
            world.AddRagdoll(RecursiveAI.OnePhysics.Physics3D.RagdollDef.Humanoid, new Vector3D(5, 1.2, 0));
            if (entities)
            {
                world.AddVehicle(world.AddBox(new(1.8f, 0.6f, 4.2f), at: new(-6, 1.0, 4), density: 357f), RecursiveAI.OnePhysics.Physics3D.VehicleDef.Car);
                world.AddCharacter(new Vector3D(1, 1, 2));
            }
            var pivot = world.AddBox(new(0.2f, 0.2f, 0.2f), at: new(-4, 3, 0), type: BodyType.Static);
            var prev = pivot;
            for (int i = 0; i < 4; i++)
            {
                var link = world.AddBox(new(1, 0.2f, 0.2f), at: new(-3.5 + i, 3, 0));
                world.CreateJoint(new PointJointDef { BodyA = prev, BodyB = link, LocalFrameA = new JointFrame3D(new Vector3F(i == 0 ? 0f : 0.5f, 0, 0), QuaternionF.Identity), LocalFrameB = new JointFrame3D(new Vector3F(-0.5f, 0, 0), QuaternionF.Identity) });
                prev = link;
            }

            var writer = world.CreateInterestWriter();
            for (int m = 0; m <= count; m++)
            {
                world.Step(1f / 60f);
                result[m] = writer.Write(new InterestRegion().AddBox(new Vector3D(-8 + m, -2, -10), new Vector3D(2 + m, 20, 10)));
            }
        }

        return result;
    }
}
