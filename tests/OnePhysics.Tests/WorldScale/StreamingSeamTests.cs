using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.WorldScale;

public sealed class StreamingSeamTests
{
    private static string Dbg = "";

    private static float Plane(int x, int z) => -0.12f * x;

    // Rolls a sphere down a plane across the seams of a row of tiles and returns the largest change of vertical speed in one step within half a metre of a seam
    // (the sphere's radius is 0.4, so every contact in that band is at the seam).
    private static double RollAcrossSeams(bool edges, bool unsplit, out int seamsCrossed)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 1350, Gravity = new(0, -9.81f, 0) });
        int cells = WorldScaleScenes.TileCells, tiles = 4;
        if (unsplit)
        {
            int nx = tiles * cells + 1, nz = cells + 1;
            var h = new float[nx * nz];
            for (int j = 0; j < nz; j++) { for (int i = 0; i < nx; i++) { h[j * nx + i] = Plane(i, j); } }
            w.AddHeightField(h, nx, nz, new Vector3F(1, 1, 1));
        }
        else
        {
            for (int tx = 0; tx < tiles; tx++) { w.LoadTile(new TileKey(tx, 0), WorldScaleScenes.Tile3D(tx, 0, edges, cells, Plane)); }
        }

        var ball = w.AddSphere(0.4f, new Vector3D(2, Plane(2, 16) + 0.5, 16));
        ball.LinearVelocity = new Vector3F(4f, 0, 0);
        double worst = 0, last = 0;
        seamsCrossed = 0;
        double previousX = 2;
        for (int s = 0; s < 3000; s++)
        {
            w.Step(WorldScaleScenes.Dt);
            double vy = ball.LinearVelocity.Y, x = ball.Position.X;
            if (s > 30 && x > 3 && x < tiles * cells - 3)
            {
                double toSeam = Math.Abs(x - Math.Round(x / cells) * cells);
                if (toSeam < 0.5) { worst = Math.Max(worst, Math.Abs(vy - last)); if (Math.Abs(vy - last) > 0.05) { Dbg += $" x={x:F2} z={ball.Position.Z:F2} dv={vy - last:F3} v={ball.LinearVelocity.X:F2}"; } }
                if (Math.Floor(x / cells) != Math.Floor(previousX / cells)) { seamsCrossed++; }
            }

            previousX = x; last = vy;
            if (x > tiles * cells - 2) { break; }
        }

        return worst;
    }

    [Fact]
    public void TilesWithNeighbourHeightsRollLikeOneField()
    {
        double unsplit = RollAcrossSeams(false, true, out int c0), tiled = RollAcrossSeams(true, false, out int c1), ghosty = RollAcrossSeams(false, false, out int c2);
        Assert.True(c0 >= 2 && c1 >= 2 && c2 >= 2, "the sphere must cross the seams");
        Assert.True(tiled <= unsplit + 0.02, $"seam dvy: unsplit {unsplit:G4}, tiled with edges {tiled:G4}, tiled without {ghosty:G4}" + Dbg);
        Assert.True(ghosty > 0.2, $"without neighbour heights the seam must show its ghost: {ghosty:G4}");
    }
}
