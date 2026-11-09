using System;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Pulley systems on one and three threads agree, and a snapshot restored into a fresh world continues identically, in both dimensions.
internal static class PulleySmoke
{
    internal static bool Run(out ulong digest)
    {
        digest = 0;
        foreach (var name in new[] { "tackle-2d", "belt-2d", "chain-drive-3d", "crane-3d" })
        {
            var a = PulleyScene.Create(name, 1); var b = PulleyScene.Create(name, 3);
            for (int i = 0; i < 60; i++)
            {
                a.Step(i); b.Step(i);
                if (a.Hash() != b.Hash()) return false;
            }
            if (a.Planar != null)
            {
                var snapshot = a.Planar.Snapshot();
                var copy = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 880, ThreadCount = 2, SleepEnabled = false }); copy.Restore(snapshot);
                if (copy.ComputeHash() != a.Planar.ComputeHash() || copy.PulleySystemCount != a.Planar.PulleySystemCount) return false;
                for (int i = 0; i < 30; i++) { a.Planar.Step(1f / 60); copy.Step(1f / 60); if (a.Planar.ComputeHash() != copy.ComputeHash()) return false; }
            }
            else
            {
                var snapshot = a.Spatial!.Snapshot();
                var copy = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 881, ThreadCount = 2, SleepEnabled = false }); copy.Restore(snapshot);
                if (copy.ComputeHash() != a.Spatial.ComputeHash() || copy.PulleySystemCount != a.Spatial.PulleySystemCount) return false;
                for (int i = 0; i < 30; i++) { a.Spatial.Step(1f / 60); copy.Step(1f / 60); if (a.Spatial.ComputeHash() != copy.ComputeHash()) return false; }
            }
            digest = digest * 31 + a.Hash();
        }
        return true;
    }
}
