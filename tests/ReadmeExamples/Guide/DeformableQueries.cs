using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // DeformableKind
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, DeformableHit3D

namespace GuideExamples;

/// <summary>The code block of docs/guide/water.md about querying deformables.</summary>
public static class DeformableQueries
{
    public static (DeformableKind Kind, int Slot, int Element, int Contacts, ulong Hash) Run()
    {
        DeformableKind kind = default; int slot = -1, element = -1, contacts = 0;
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        world.AddCloth(2f, 2f, 8, new Vector3D(0, 2, 0));
        var origin = new Vector3D(0.1, 5, 0.1);
        var direction = new Vector3F(0, -1, 0);
        if (world.RaycastDeformables(origin, direction, out var hit))
        {
            kind = hit.Kind; slot = hit.Slot; element = hit.Index;                // SoftBody, Cloth or Rod; GetSoftBody(slot) / GetRod(slot); the triangle or segment
        }
        world.TrackDeformableContacts = true;                                    // opt in: soft body, cloth and rod contact begin/end with rigid bodies
        world.DeformableContactChanged += e => contacts++;

        for (int i = 0; i < 240; i++) { world.Step(1f / 60f); }
        return (kind, slot, element, contacts, world.ComputeHash());
    }
}
