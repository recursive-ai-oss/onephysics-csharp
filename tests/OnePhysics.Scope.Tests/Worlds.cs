using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using B2 = RecursiveAI.OnePhysics.Physics2D.Body;
using B3 = RecursiveAI.OnePhysics.Physics3D.Body;

namespace RecursiveAI.OnePhysics.Scope.Tests;

/// <summary>Scenes with contacts and joints, run under the library's own recorder, with the library's own hash after every step.</summary>
internal static class Worlds
{
    public const float Dt = 1f / 60f;

    /// <summary>A recorded run: the replay bytes and the world hash before the first step and after each step.</summary>
    public sealed record Run(int Dimension, byte[] Replay, ulong[] Hashes);

    public static Run Record(int dimension, int steps, int preSteps = 0) => dimension == 3 ? Record3D(steps, preSteps) : Record2D(steps, preSteps);

    private static Run Record3D(int steps, int preSteps)
    {
        var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7 });
        world.AddBox(new Vector3F(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++)
        {
            world.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 0.5 + i * 1.02, 0));
        }

        for (int i = 0; i < 6; i++)
        {
            world.AddSphere(0.4f, new Vector3D(3 + (i % 3) * 0.9, 1 + (i / 3) * 0.9, 1.5));
        }

        B3 anchor = world.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(-5, 8, 0), type: BodyType.Static);
        B3 previous = anchor;
        for (int i = 0; i < 4; i++)
        {
            B3 link = world.AddBox(new Vector3F(0.5f, 0.9f, 0.5f), new Vector3D(-5, 7.0 - i * 1.0, 0));
            world.AddPoint(previous, link, new Vector3D(-5, 7.5 - i * 1.0, 0));
            previous = link;
        }

        for (int i = 0; i < preSteps; i++)
        {
            world.Step(Dt);
        }

        using var stream = new MemoryStream();
        var hashes = new ulong[steps + 1];
        using (world.StartRecording(stream))
        {
            hashes[0] = world.ComputeHash();
            for (int i = 0; i < steps; i++)
            {
                if (i == steps / 3)
                {
                    world.AddSphere(0.5f, new Vector3D(0.3, 7, 0.2));
                }

                if (i == steps / 2)
                {
                    foreach (B3 b in world.Bodies)
                    {
                        if (b.Type == BodyType.Dynamic)
                        {
                            b.ApplyImpulse(new Vector3F(2, 3, 0));
                            break;
                        }
                    }
                }

                world.Step(Dt);
                hashes[i + 1] = world.ComputeHash();
            }
        }

        return new Run(3, stream.ToArray(), hashes);
    }

    private static Run Record2D(int steps, int preSteps)
    {
        var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7 });
        world.AddBox(new Vector2F(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 5; i++)
        {
            world.AddBox(new Vector2F(1, 1), new Vector2D(0, 0.5 + i * 1.02));
        }

        for (int i = 0; i < 6; i++)
        {
            world.AddCircle(0.4f, new Vector2D(3 + (i % 3) * 0.9, 1 + (i / 3) * 0.9));
        }

        B2 anchor = world.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(-5, 8), type: BodyType.Static);
        B2 previous = anchor;
        for (int i = 0; i < 4; i++)
        {
            B2 link = world.AddBox(new Vector2F(0.5f, 0.9f), new Vector2D(-5, 7.0 - i * 1.0));
            world.AddHinge(previous, link, new Vector2D(-5, 7.5 - i * 1.0));
            previous = link;
        }

        for (int i = 0; i < preSteps; i++)
        {
            world.Step(Dt);
        }

        using var stream = new MemoryStream();
        var hashes = new ulong[steps + 1];
        using (world.StartRecording(stream))
        {
            hashes[0] = world.ComputeHash();
            for (int i = 0; i < steps; i++)
            {
                if (i == steps / 3)
                {
                    world.AddCircle(0.5f, new Vector2D(0.3, 7));
                }

                if (i == steps / 2)
                {
                    foreach (B2 b in world.Bodies)
                    {
                        if (b.Type == BodyType.Dynamic)
                        {
                            b.ApplyImpulse(new Vector2F(2, 3));
                            break;
                        }
                    }
                }

                world.Step(Dt);
                hashes[i + 1] = world.ComputeHash();
            }
        }

        return new Run(2, stream.ToArray(), hashes);
    }

    /// <summary>A spatial world with resting contacts and one limited hinge, slider and cone joint.</summary>
    public static PhysicsWorld3D LimitScene3D()
    {
        var world = new PhysicsWorld3D(new WorldOptions3D { WorldId = 11 });
        B3 floor = world.AddBox(new Vector3F(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        world.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 0.5, 0));
        B3 post = world.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(-5, 6, 0), type: BodyType.Static);
        B3 arm = world.AddBox(new Vector3F(0.5f, 1.5f, 0.5f), new Vector3D(-5, 4.5, 0));
        var hinge = world.AddHinge(post, arm, new Vector3D(-5, 5.8, 0), new Vector3F(0, 0, 1));
        hinge.SetLimits(-0.5f, 0.8f);
        hinge.EnableLimit = true;
        B3 rail = world.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(5, 4, 0), type: BodyType.Static);
        B3 cart = world.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(5, 4, 0));
        var slider = world.AddSlider(rail, cart, new Vector3D(5, 4, 0), new Vector3F(1, 0, 0));
        slider.SetLimits(-0.5f, 1.5f);
        slider.EnableLimit = true;
        B3 socket = world.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), new Vector3D(0, 8, 5), type: BodyType.Static);
        B3 limb = world.AddBox(new Vector3F(0.3f, 1.2f, 0.3f), new Vector3D(0, 7, 5));
        world.AddCone(socket, limb, new Vector3D(0, 7.8, 5), new Vector3F(0, -1, 0), 0.6f);
        _ = floor;
        return world;
    }
}
