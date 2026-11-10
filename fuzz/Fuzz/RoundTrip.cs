using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

// The round-trip property, checked after every step a fuzz target makes: a snapshot the library wrote restores into a fresh world without error and re-serializes to the same bytes.
// The one documented exception is a world that has just diverged (a body not finite or outside the supported range): the next step raises a typed error, and a restore of its snapshot may refuse it by saying so.
internal static class RoundTrip
{
    public static void StepChecked(this PhysicsWorld3D w, float dt)
    {
        // A snapshot restored into a fresh world has no host delegates; a custom gravity field then refuses the step before changing anything (documented), and there is nothing to round-trip.
        try { w.Step(dt); } catch (FieldCallbackException) { return; }
        byte[] written;
        // A diverged world refuses to be saved (by contract); the next step reports the body, so there is nothing to round-trip yet.
        try { written = w.Snapshot(); } catch (NonFiniteStateException) { return; }
        Check(written, () => { var f = new PhysicsWorld3D(w.Options); f.Restore(written); return f.Snapshot(); });
    }

    public static void StepChecked(this PhysicsWorld2D w, float dt)
    {
        // A snapshot restored into a fresh world has no host delegates; a custom gravity field then refuses the step before changing anything (documented), and there is nothing to round-trip.
        try { w.Step(dt); } catch (FieldCallbackException) { return; }
        byte[] written;
        // A diverged world refuses to be saved (by contract); the next step reports the body, so there is nothing to round-trip yet.
        try { written = w.Snapshot(); } catch (NonFiniteStateException) { return; }
        Check(written, () => { var f = new PhysicsWorld2D(w.Options); f.Restore(written); return f.Snapshot(); });
    }

    private static void Check(byte[] written, Func<byte[]> restoreAndWriteAgain)
    {
        byte[] again;
        try { again = restoreAndWriteAgain(); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (!again.AsSpan().SequenceEqual(written)) { throw new InvalidOperationException("a snapshot the library wrote restores into a world that re-serializes differently"); }
    }
}
