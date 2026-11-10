using System.Globalization;

namespace Bar;

// cloth-stability: the fabric scenes (cloth-flag-3d, cloth-character-3d, cloth-throw-3d, cloth-garment-3d, strip-2d) and the soft
// fracture scenes (cloth-fold-3d, tear-3d) with the limits the test suite and docs/spec/scenes.md already state. Own thresholds:
// no engine runs these scenes. Prints one line per scene ending in PASS or FAIL.
internal static class ClothStability
{
    static readonly (string Scene, double Speed, double Pen, double Strain)[] Fabric =
    [
        ("cloth-flag-3d", 20, 0, 0.2),
        ("cloth-character-3d", 7, 0.02, 0.09),
        ("cloth-throw-3d", 40, 0, 0.11),
        ("cloth-garment-3d", 3.5, 0.02, 0.06),
    ];
    static readonly (string Scene, double Speed, int MinTears)[] Fracture = [("cloth-fold-3d", 20, 0), ("tear-3d", 20, 50)];

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int bad = 0;
        foreach (var l in Fabric)
        {
            if (only != "" && only != l.Scene) continue;
            var m = Cloth3DScene.Measure(l.Scene, 600, threads);
            bool ok = m.NaN == 0 && m.Escapes == 0 && m.MaxSpeed <= l.Speed && m.MaxPenetration <= l.Pen && m.MaxStrain <= l.Strain && m.MaxIntersections == 0
                && (l.Scene != "cloth-garment-3d" || m.MaxSeamGap <= 0.09);
            Console.WriteLine(Cloth3DScene.Format(m) + $" threads={threads}" + (ok ? " PASS" : " FAIL"));
            if (!ok) bad++;
        }
        if (only == "" || only == "strip-2d")
        {
            var m = Strip2DScene.Measure(600, threads);
            bool ok = m.NaN == 0 && m.Escapes == 0 && m.MaxSpeed <= 16 && m.MaxStrain <= 0.11;
            Console.WriteLine(Strip2DScene.Format(m) + $" threads={threads}" + (ok ? " PASS" : " FAIL"));
            if (!ok) bad++;
        }
        foreach (var l in Fracture)
        {
            if (only != "" && only != l.Scene) continue;
            var m = SoftFracture3DScene.Measure(l.Scene, 600, threads);
            bool ok = m.NaN == 0 && m.Escapes == 0 && m.MaxSpeed <= l.Speed && m.Tears >= l.MinTears && m.MaxIntersections == 0 && m.Asleep == m.Bodies;
            Console.WriteLine(SoftFracture3DScene.Format(m) + $" threads={threads}" + (ok ? " PASS" : " FAIL"));
            if (!ok) bad++;
        }
        return bad == 0 ? 0 : 1;
    }
}
