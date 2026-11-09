namespace Bar;

// Prints one stability row per 2D soft-body scene (see docs/spec/scenes.md). Exit code 1 when a limit is broken.
internal static class SoftStability2D
{
    private static readonly (string Scene, double Speed, double Pen, double Area, double Overlap, double Final)[] Limits =
    {
        ("soft-pile-2d", 25.0, 0.08, 0.05, 0.15, 1.5),
        ("soft-rigid-2d", 15.0, 0.05, 0.02, 0.10, 1.5),
        ("balloon-2d", 20.0, 0.06, 0.02, 0.05, 1.0),
    };

    internal static int Run()
    {
        int bad = 0;
        foreach (var l in Limits)
        {
            var m = SoftBodies2DScene.Measure(l.Scene);
            bool ok = m.NaN == 0 && m.Escapes == 0 && m.MaxSpeed <= l.Speed && m.MaxPenetration <= l.Pen && m.AreaError <= l.Area && m.MaxSoftOverlap <= l.Overlap && m.FinalSpeed99 <= l.Final;
            Console.WriteLine(SoftBodies2DScene.Format(m) + (ok ? " STABLE" : " UNSTABLE"));
            if (!ok) { bad++; }
        }

        return bad == 0 ? 0 : 1;
    }
}
