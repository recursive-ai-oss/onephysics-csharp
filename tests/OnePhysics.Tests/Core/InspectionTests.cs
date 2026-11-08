using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

[Collection("Allocation")]
public class InspectionTests
{
    private const float Dt = 1f / 60f;

    private static P2.PhysicsWorld2D Scene2(int threads, int width)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { DetailedInspection = true, ThreadCount = threads, SleepEnabled = false });
        w.Engine.KernelWidth = width;
        w.AddBox(new Vector2F(20, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector2F(1, 1), new Vector2D((i % 4) * 1.2, 0.5 + i / 4)); }
        return w;
    }

    private static P3.PhysicsWorld3D Scene3(int threads, int width)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { DetailedInspection = true, ThreadCount = threads, SleepEnabled = false });
        w.Engine.KernelWidth = width;
        w.AddBox(new Vector3F(20, 1, 20), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector3F(1, 1, 1), new Vector3D((i % 4) * 1.2, 0.5 + i / 4, 0)); }
        return w;
    }

    [Fact]
    public void DetailOptionIsObservationalAndRequiredForImpulseReadback()
    {
        var a = Scene2(1, 1); var b = Scene2(1, 1);
        var c = Scene3(1, 1); var d = Scene3(1, 1);
        b.Options.DetailedInspection = false; d.Options.DetailedInspection = false;
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt); c.Step(Dt); d.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash()); Assert.Equal(c.ComputeHash(), d.ComputeHash());
            EqualCounters(a.Counters, b.Counters); EqualCounters(c.Counters, d.Counters);
        }
        Assert.Equal(a.Snapshot(), b.Snapshot()); Assert.Equal(c.Snapshot(), d.Snapshot());
        var error = Assert.Throws<DetailedInspectionDisabledException>(() => { _ = b.Core.BodyReadback.Impulses.Length; });
        Assert.Contains("Options.DetailedInspection", error.Message);
        Assert.Throws<DetailedInspectionDisabledException>(() => { _ = d.Core.JointReadback.Rows.Length; });
        b.Options.DetailedInspection = true; d.Options.DetailedInspection = true;
        b.Step(Dt); d.Step(Dt);
        Assert.True(b.Core.BodyReadback.Impulses.Length > 0 && d.Core.BodyReadback.Impulses.Length > 0);
        // Restore keeps the host's diagnostic setting, including the reused spare-options path.
        b.Restore(a.Snapshot()); d.Restore(c.Snapshot());
        b.Restore(a.Snapshot()); d.Restore(c.Snapshot());
        Assert.True(b.Options.DetailedInspection && d.Options.DetailedInspection);
        b.Options.DetailedInspection = false; d.Options.DetailedInspection = false;
        b.Step(Dt); d.Step(Dt);
        Assert.Throws<DetailedInspectionDisabledException>(() => { _ = b.Core.BodyReadback.Impulses.Length; });
        Assert.Throws<DetailedInspectionDisabledException>(() => { _ = d.Core.JointReadback.Impulses.Length; });
    }

    private static void EqualCounters(StepCounters a, StepCounters b)
    {
        Assert.Equal(a.NarrowphasePairs, b.NarrowphasePairs);
        Assert.Equal(a.NewManifolds, b.NewManifolds);
        Assert.Equal(a.PersistedManifolds, b.PersistedManifolds);
        Assert.Equal(a.NewContactPoints, b.NewContactPoints);
        Assert.Equal(a.PersistedContactPoints, b.PersistedContactPoints);
        Assert.Equal(a.RecycledContactPoints, b.RecycledContactPoints);
        Assert.Equal(a.WorldWarmStartPasses, b.WorldWarmStartPasses);
        Assert.Equal(a.WorldRestitutionPasses, b.WorldRestitutionPasses);
        Assert.Equal(a.ContactWarmStartPasses, b.ContactWarmStartPasses);
        Assert.Equal(a.ContactRestitutionPasses, b.ContactRestitutionPasses);
        Assert.Equal(a.CcdBodies, b.CcdBodies);
        Assert.Equal(a.CcdHits, b.CcdHits);
        Assert.Equal(a.BulletBodies, b.BulletBodies);
        Assert.Equal(a.SoftParticles, b.SoftParticles);
        Assert.Equal(a.SoftBlocks, b.SoftBlocks);
        Assert.Equal(a.Characters, b.Characters);
        Assert.Equal(a.Vehicles, b.Vehicles);
        Assert.Equal(a.AwakeBodies, b.AwakeBodies);
        Assert.Equal(a.AwakeIslands, b.AwakeIslands);
        Assert.Equal(a.AsleepIslands, b.AsleepIslands);
        Assert.Equal(a.BroadphasePairs, b.BroadphasePairs);
        Assert.Equal(a.UpdatedPairs, b.UpdatedPairs);
        Assert.Equal(a.RecycledManifolds, b.RecycledManifolds);
        Assert.Equal(a.Manifolds, b.Manifolds);
        Assert.Equal(a.ContactPoints, b.ContactPoints);
        Assert.Equal(a.Joints, b.Joints);
        Assert.True(a.ConstraintsPerColor.SequenceEqual(b.ConstraintsPerColor));
        Assert.Equal(a.SerialOverflow, b.SerialOverflow);
        Assert.Equal(a.WorldSubsteps, b.WorldSubsteps);
        Assert.Equal(a.WorldSolvePasses, b.WorldSolvePasses);
        Assert.Equal(a.WorldRelaxPasses, b.WorldRelaxPasses);
        Assert.Equal(a.ContactSubsteps, b.ContactSubsteps);
        Assert.Equal(a.ContactSolvePasses, b.ContactSolvePasses);
        Assert.Equal(a.ContactRelaxPasses, b.ContactRelaxPasses);
        Assert.Equal(a.SleepTransitions, b.SleepTransitions);
        Assert.Equal(a.WakeTransitions, b.WakeTransitions);
    }

    [Theory]
    [InlineData(4)]
    [InlineData(8)]
    public void CountersAndHashesAgreeAcrossThreadsWidthsAndProfiling(int width)
    {
        var a = Scene2(1, 1); var b = Scene2(8, width);
        var c = Scene3(1, 1); var d = Scene3(8, width);
        long tick2 = 0, tick3 = 0;
        b.Profile.TimestampSource = () => ++tick2;
        d.Profile.TimestampSource = () => ++tick3;
        for (int i = 0; i < 120; i++)
        {
            a.Step(Dt); b.Step(Dt); c.Step(Dt); d.Step(Dt);
            EqualCounters(a.Counters, b.Counters); EqualCounters(c.Counters, d.Counters);
            Assert.True(a.Core.BodyReadback.Impulses.SequenceEqual(b.Core.BodyReadback.Impulses));
            Assert.True(c.Core.BodyReadback.Impulses.SequenceEqual(d.Core.BodyReadback.Impulses));
            foreach (var counts in new[] { a.Counters, c.Counters })
            {
                Assert.Equal(counts.Manifolds, counts.NewManifolds + counts.PersistedManifolds + counts.RecycledManifolds);
                Assert.Equal(counts.ContactPoints, counts.NewContactPoints + counts.PersistedContactPoints + counts.RecycledContactPoints);
                Assert.True(counts.PersistedManifolds >= 0 && counts.PersistedContactPoints >= 0 && counts.NarrowphasePairs >= 0);
            }
            Assert.Equal(a.ComputeHash(), b.ComputeHash()); Assert.Equal(c.ComputeHash(), d.ComputeHash());
        }
        Assert.True(tick2 > 0 && tick3 > 0);
        Assert.True(b.Profile.Durations[(int)StepStage.Solve] > 0);
        Assert.True(d.Profile.Durations[(int)StepStage.Narrowphase] > 0);
        foreach (long t in a.Profile.Durations) { Assert.Equal(0, t); }
        Assert.Equal(0, b.Profile.InvalidTimestampCount);
        long after = tick2;
        b.Engine.Bp.UpdatePairs(new AcceptAllPairs());
        Assert.Equal(after, tick2);
    }

    [Fact]
    public void TinyRestingBoxesExposeKnownCountsAndReadOnlyContactImpulses()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { DetailedInspection = true, SleepEnabled = false });
        var floor = w.AddBox(new Vector2F(10, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var box = w.AddBox(new Vector2F(1, 1), new Vector2D(0, 0.5));
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.Equal(1, w.Counters.AwakeBodies);
        Assert.Equal(1, w.Counters.AwakeIslands);
        Assert.Equal(0, w.Counters.AsleepIslands);
        Assert.Equal(1, w.Counters.Manifolds);
        Assert.Equal(2, w.Counters.ContactPoints);
        Assert.Equal(0, w.Counters.Joints);
        Assert.Equal(1, box.Inspect().ConstraintCount);
        Assert.Equal(1, floor.Inspect().ConstraintCount);
        Assert.True(box.Inspect().IsAwake);
        Assert.True(box.Inspect().SleepTime >= 0);
        float impulse = 0;
        foreach (var contact in box.Inspect().Constraints)
        {
            Assert.True(contact.IsContact);
            Assert.Equal(2, contact.Normal.Length);
            Assert.True(contact.Color < 24);
            for (int p = 0; p < contact.PointCount; p++) { impulse += contact.Point(p).NormalImpulse; Assert.Equal(0, contact.Point(p).Tangent2Impulse); }
        }
        float force = impulse * w.Options.Substeps / Dt;
        Assert.InRange(force, box.Mass * 9.81f * 0.98f, box.Mass * 9.81f * 1.02f);
        var view = box.Inspect();
        box.Destroy();
        Assert.Throws<StaleHandleException>(() => view.ConstraintCount);
        Assert.Throws<InvalidHandleException>(() => default(BodyInspection).Island);
        Assert.Throws<InvalidHandleException>(() => default(BodyConstraintEnumerable).GetEnumerator());
        Assert.Throws<InvalidOperationException>(() => { var c = default(ConstraintInspection); _ = c.PointCount; });
        Assert.Throws<InvalidOperationException>(() => { var p = default(ContactPointInspection); _ = p.NormalImpulse; });
    }

    [Fact]
    public void SpatialRestingBoxAndStackSupportWeight()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { DetailedInspection = true, SleepEnabled = false });
        var floor = w.AddBox(new Vector3F(10, 1, 10), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var lower = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 0.5, 0));
        var upper = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 1.5, 0));
        for (int i = 0; i < 180; i++) { w.Step(Dt); }
        Assert.Equal(2, w.Counters.AwakeBodies);
        Assert.Equal(1, w.Counters.AwakeIslands);
        Assert.Equal(2, w.Counters.Manifolds);
        Assert.Equal(8, w.Counters.ContactPoints);
        Assert.Equal(2, lower.Inspect().ConstraintCount);
        Assert.Equal(1, upper.Inspect().ConstraintCount);
        float impulse = 0;
        foreach (var contact in floor.Inspect().Constraints)
        {
            Assert.Equal(3, contact.Normal.Length);
            for (int p = 0; p < contact.PointCount; p++) { impulse += contact.Point(p).NormalImpulse; }
        }
        float force = impulse * w.Counters.ContactSubsteps / Dt;
        // An unsplit program uses the world schedule, even if its bodies are contact-only.
        if (w.Counters.ContactSubsteps == 0) { force = impulse * w.Counters.WorldSubsteps / Dt; }
        float weight = (lower.Mass + upper.Mass) * 9.81f;
        Assert.InRange(force, weight * 0.97f, weight * 1.03f);
    }

    [Fact]
    public void DiagnosticsDoNotEnterSnapshotAndRestoreDoesNotRestoreThem()
    {
        var a = Scene2(1, 1); var b = Scene2(1, 1);
        long ticks = 0;
        b.Profile.TimestampSource = () => ++ticks;
        for (int i = 0; i < 20; i++) { a.Step(Dt); b.Step(Dt); }
        Assert.Equal(a.Snapshot(), b.Snapshot());
        byte[] saved = b.Snapshot();
        for (int i = 0; i < 10; i++) { b.Step(Dt); }
        b.Restore(saved);
        Assert.Equal(saved, b.Snapshot());
        Assert.NotNull(b.Profile.TimestampSource);
        a.Step(Dt); b.Step(Dt);
        Assert.Equal(a.Snapshot(), b.Snapshot());
        EqualCounters(a.Counters, b.Counters);
    }

    [Fact]
    public void TimestampSourcesArePerWorldAndCanBeDisabledOrDecrease()
    {
        var a = Scene2(1, 1); var b = Scene2(1, 1);
        long tick = 1000;
        a.Profile.TimestampSource = () => --tick;
        a.Step(Dt); b.Step(Dt);
        Assert.True(a.Profile.InvalidTimestampCount > 0);
        Assert.Equal(0, b.Profile.InvalidTimestampCount);
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        a.Profile.TimestampSource = null;
        a.Step(Dt);
        Assert.Equal(0, a.Profile.InvalidTimestampCount);
        foreach (long duration in a.Profile.Durations) { Assert.Equal(0, duration); }
    }

    [Fact]
    public void CountersProfileAndBodyInspectionAllocateNothingAfterWarmup()
    {
        var w = Scene2(1, 1);
        w.Profile.TimestampSource = static () => 42;
        for (int i = 0; i < 40; i++) { w.Step(Dt); }
        void Read()
        {
            foreach (var body in w.Bodies)
            {
                var view = body.Inspect();
                _ = view.IsAwake; _ = view.SleepTime; _ = view.Island;
                foreach (var constraint in view.Constraints)
                {
                    for (int p = 0; p < constraint.PointCount; p++) { _ = constraint.Point(p).NormalImpulse; }
                }
            }
        }
        Read();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 20; i++) { w.Step(Dt); Read(); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
