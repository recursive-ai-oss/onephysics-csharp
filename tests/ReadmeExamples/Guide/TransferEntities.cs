using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType, BodySetPacket, BodySetOptions
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Body, Character, Vehicle, VehicleDef

namespace GuideExamples;

/// <summary>The code block of docs/guide/transfer.md that moves a car and its driver.</summary>
public static class TransferEntities
{
    public static (int Characters, int Vehicles, bool Seated, double Gap) Run()
    {
        var north = new PhysicsWorld3D();
        north.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        Body chassis = north.AddBox(new Vector3F(1.8f, 0.6f, 4.2f), at: new Vector3D(0, 1.0, 0), density: 357f);
        Vehicle car = north.AddVehicle(chassis, VehicleDef.Car);
        Character driver = north.AddCharacter(new Vector3D(0, 2, 0));
        driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        for (int i = 0; i < 90; i++) { car.Drive(throttle: 0.5f); north.Step(1f / 60f); }

        // The car and its driver cross to the world that owns the next stretch of road.
        BodySetPacket packet = north.ExtractBodies(new[] { chassis, driver.Body });
        var south = new PhysicsWorld3D();
        south.AddBox(new Vector3F(400, 1, 400), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);   // the same road
        BodySetHandleMap map = south.InsertBodies(BodySetPacket.Read(packet.ToArray()));
        Vehicle arrived = south.GetVehicle(map.NewVehicles[0]);
        Character passenger = south.GetCharacter(map.NewCharacters[0]);

        var before = arrived.Chassis.Position;
        for (int i = 0; i < 60; i++) { arrived.Drive(throttle: 0.5f); south.Step(1f / 60f); }
        var after = arrived.Chassis.Position;
        return (packet.CharacterCount, packet.VehicleCount, passenger.Mode == CharacterMode.Seated, after.Z - before.Z);
    }
}
