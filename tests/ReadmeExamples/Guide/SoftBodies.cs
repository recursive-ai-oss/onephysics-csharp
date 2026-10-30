#pragma warning disable OPHYS0002
using System;
using RecursiveAI.OneMath;               // Vector3F, Vector3D
using RecursiveAI.OnePhysics;            // BodyType
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D, SoftBody, SoftBodyDef, SoftBodyMesh, SoftSkin

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/soft-bodies.md.</summary>
public static class SoftBodies
{
    public static (double ClothY, double CubeVolumeRatio, ulong Hash) Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        world.AddSphere(0.6f, at: new Vector3D(0, 0.6, 0), type: BodyType.Static);

        // Happy path: a cloth, a jelly cube and a rubber ball, each in one line.
        SoftBody cloth = world.AddCloth(width: 2f, depth: 2f, segments: 16, at: new Vector3D(0, 1.8, 0));
        SoftBody cube = world.AddSoftBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(2, 1, 0), SoftMaterial.Jelly);
        SoftBody ball = world.AddSoftBall(0.3f, at: new Vector3D(-2, 1, 0), SoftMaterial.Rubber);
        var crate = world.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(2, 3, 0), density: 40f);   // a rigid box lands on the cube: two-way coupling

        for (int i = 0; i < 180; i++) world.Step(1f / 60);

        Vector3D middle = cloth.GetPosition(8 * 17 + 8);
        Console.WriteLine($"cloth centre {middle}, cube volume {cube.Volume / cube.RestVolume:P1} of rest, ball asleep {ball.IsAsleep}, crate {crate.Position}");

        // Middle tier: a definition from your own tetrahedral mesh, with every parameter.
        var mesh = SoftBodyMesh.FromTetrahedra(
            [new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1)], [0, 1, 2, 3]);
        var def = SoftBodyDef.Rubber(mesh, at: new Vector3D(0, 5, 0));
        def.Mass = 2f;
        def.EdgeCompliance = 1e-5f;      // metres per newton; 0 is as stiff as the solver can make it
        def.PinnedParticles = [0];       // particle 0 does not move
        SoftBody custom = world.AddSoftBody(def);
        world.Step(1f / 60);

        // Skinning: a cloth that follows a bone and may stray at most 5 cm from where the bone carries it; its first row is kinematic.
        var sheet = SoftBodyMesh.Cloth(1f, 1f, 4, 4);
        int n = sheet.VertexCount;
        var joints = new int[4 * n];        // four joint numbers per particle (all joint 0 here)
        var weights = new float[4 * n];     // and four weights
        var maxDistance = new float[n];
        for (int i = 0; i < n; i++) { weights[4 * i] = 1f; maxDistance[i] = i < 5 ? 0f : 0.05f; }
        var skin = SoftSkin.Create(sheet, [-1], [new Vector3F(0, 0, 0)], joints, weights, maxDistance);
        SoftBody banner = world.AddSoftBody(new SoftBodyDef { Mesh = sheet, Skin = skin, Position = new Vector3D(0, 4, 3) });
        banner.SetSkeletonPose([QuaternionF.Identity], at: new Vector3D(0.5, 4, 3));   // move the bone; the targets follow over the next step
        world.Step(1f / 60);
        Console.WriteLine($"banner corner {banner.GetPosition(0)}, skinned {banner.GetSkinnedPosition(0)}");
        return (middle.Y, cube.Volume / cube.RestVolume, world.ComputeHash() + (ulong)custom.ParticleCount);
    }
}
