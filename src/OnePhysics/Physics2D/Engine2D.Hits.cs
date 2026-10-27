using System;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>
    /// Hit events: after the last substep, a contact whose fastest approach at the start of the step exceeded <see cref="WorldOptions2D.HitSpeedThreshold"/> and that pushed with a positive impulse reports it.
    /// Each chunk of constraints writes to its own event slot, so the delivered order does not depend on threads.
    /// </summary>
    public void EmitHits(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot)
    {
        var events = world.Events;
        if (!events.Wants(EventKind.ContactHit)) { return; }
        var cs = world.Constraints;
        float threshold = Opt.HitSpeedThreshold;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) != ConstraintKind.Contact) { continue; }
            var pl = cs.Payload(c);
            int count = cs.PointCount[c];
            float best = 0f;
            for (int i = 0; i < count; i++)
            {
                if (!(pl[ManifoldLayout.MaxNormalImpulse(i)] > 0f)) { continue; }
                int slot = SlotOf[c];
                float approach = 0f - Wide[(slot >> 3) * W2D.GroupFloats + (W2D.Point0 + i * W2D.PointStride + W2D.RelativeVelocity) * 8 + (slot & 7)];
                if (approach > best) { best = approach; }
            }

            if (!(best > threshold)) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            events.Emit(eventSlot, new PhysicsEvent(EventKind.ContactHit, world.HandleOfBody(a), world.HandleOfBody(b),
                new ShapeHandle(world.Id, cs.ShapeA[c], cs.ShapeGenerationA(c)), new ShapeHandle(world.Id, cs.ShapeB[c], cs.ShapeGenerationB(c)), cs.SubShapeA[c], cs.SubShapeB[c], default, best));
        }
    }
}
