using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F, QuaternionF
using RecursiveAI.OnePhysics;                    // BodyType, BodySetPacket, BodySetOptions, BoundaryPolicy
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Body, RagdollDef

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/transfer.md.</summary>
public static class Transfer
{
    public static (int Bodies, int Contacts, int Bytes, double SpeedAfter, bool SameHash, ulong Hash) Run()
    {
        var level = new PhysicsWorld3D();
        level.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = level.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.6, 0));
        level.AddRagdoll(RagdollDef.Humanoid, new Vector3D(6, 1.2, 0));
        for (int i = 0; i < 60; i++) level.Step(1f / 60f);

        BodySetPacket packet = level.ExtractBodies(new[] { crate });             // moves the crate out of `level`
        byte[] wire = packet.ToArray();                                          // send these bytes to the other world

        var other = new PhysicsWorld3D();
        other.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);   // the same ground
        BodySetHandleMap map = other.InsertBodies(BodySetPacket.Read(wire));
        Body arrived = other.GetBody(map.NewBodies[0]);

        // A portal: the body comes out turned a quarter about y, a hundred metres away, with its velocity turned too.
        var portalWorld = new PhysicsWorld3D();
        var ball = portalWorld.AddSphere(0.5f, at: new Vector3D(0, 2, 0));
        ball.LinearVelocity = new Vector3F(3, 0, 0);
        BodySetPacket thrown = portalWorld.ExtractBodies(new[] { ball }, new BodySetOptions { Move = false });
        var far = new PhysicsWorld3D();
        Body through = far.GetBody(far.InsertBodies(thrown, at: new Vector3D(100, 0, 0), rotation: QuaternionF.CreateFromAxisAngle(new Vector3F(0, 1, 0), 1.5707964f)).NewBodies[0]);

        var again = new PhysicsWorld3D();
        again.AddBox(new Vector3F(60, 1, 60), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        again.InsertBodies(packet);
        return (packet.BodyCount, packet.ContactCount, wire.Length, Math.Abs(through.LinearVelocity.Z), again.ComputeHash() == other.ComputeHash(), other.ComputeHash());
    }
}
