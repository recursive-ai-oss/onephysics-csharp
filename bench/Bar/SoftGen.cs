namespace Bar;

/// <summary>A soft body of a scene: a generator kind with its parameters (docs/spec/scenes.md, "soft" records). Every engine builds the same mesh from <see cref="SoftGen"/>.</summary>
public sealed class SoftDef
{
    public string Preset = "jelly";   // cloth, rubber, jelly: each engine's own starting values for that kind of material
    public string Kind = "box";       // cloth, box, ball
    public double X, Y, Z, Qx, Qy, Qz, Qw = 1;
    public double Mass = 1;
    public double[] P = [];           // cloth: width depth n pinMode (0 none, 1 four corners); box: sx sy sz nx ny nz; ball: radius n
}

/// <summary>A mesh in the body's own frame: float32 vertices (x y z per vertex), surface triangles (cloth and the boundary of volumes) and tetrahedra (volumes, positive orientation).</summary>
public sealed class SoftMeshData
{
    public float[] V = [];
    public int[] Tris = [];
    public int[] Tets = [];
    public int[] Pins = [];
    public int Count => V.Length / 3;
}

/// <summary>Deterministic generators for the soft-body scenes. Only IEEE-exact arithmetic, no trigonometry.</summary>
public static class SoftGen
{
    public static SoftMeshData Build(SoftDef d) => d.Kind switch
    {
        "cloth" => Cloth(d.P[0], d.P[1], (int)d.P[2], (int)d.P[3]),
        "box" => Box(d.P[0], d.P[1], d.P[2], (int)d.P[3], (int)d.P[4], (int)d.P[5]),
        "ball" => Ball(d.P[0], (int)d.P[1]),
        "sleeve" => SleeveRig.Mesh(d.P[0], d.P[1], (int)d.P[2], (int)d.P[3]),
        _ => throw new InvalidDataException("unknown soft kind " + d.Kind),
    };

    static SoftMeshData Cloth(double w, double dp, int n, int pinMode)
    {
        var m = new SoftMeshData { V = new float[3 * (n + 1) * (n + 1)] };
        int k = 0;
        for (int j = 0; j <= n; j++)
            for (int i = 0; i <= n; i++) { m.V[k++] = (float)(w * (i / (double)n - 0.5)); m.V[k++] = 0f; m.V[k++] = (float)(dp * (j / (double)n - 0.5)); }
        var tris = new List<int>();
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                int a = j * (n + 1) + i, b = a + 1, c = a + n + 1, e = c + 1;
                tris.AddRange([a, c, b, b, c, e]);
            }
        m.Tris = tris.ToArray();
        if (pinMode == 1) m.Pins = [0, n, n * (n + 1), n * (n + 1) + n];
        return m;
    }

    static SoftMeshData Lattice(int nx, int ny, int nz, Func<double, double, double, (double, double, double)> place)
    {
        var m = new SoftMeshData { V = new float[3 * (nx + 1) * (ny + 1) * (nz + 1)] };
        int Id(int i, int j, int k) => (k * (ny + 1) + j) * (nx + 1) + i;
        for (int k = 0; k <= nz; k++)
            for (int j = 0; j <= ny; j++)
                for (int i = 0; i <= nx; i++)
                {
                    var (x, y, z) = place(i / (double)nx, j / (double)ny, k / (double)nz);
                    int o = 3 * Id(i, j, k); m.V[o] = (float)x; m.V[o + 1] = (float)y; m.V[o + 2] = (float)z;
                }
        int[][] perms = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
        var tets = new List<int>();
        for (int k = 0; k < nz; k++)
            for (int j = 0; j < ny; j++)
                for (int i = 0; i < nx; i++)
                    foreach (var p in perms)
                    {
                        var c = new[] { i, j, k }; int v0 = Id(c[0], c[1], c[2]);
                        c[p[0]]++; int v1 = Id(c[0], c[1], c[2]);
                        c[p[1]]++; int v2 = Id(c[0], c[1], c[2]);
                        c[p[2]]++; int v3 = Id(c[0], c[1], c[2]);
                        if (Det(m.V, v0, v1, v2, v3) < 0) (v2, v3) = (v3, v2);
                        tets.AddRange([v0, v1, v2, v3]);
                    }
        m.Tets = tets.ToArray();
        // Boundary: faces of the lattice cells on the outside, two triangles each, outward.
        var tris = new List<int>();
        void Face(int a, int b, int c, int dd, int ox, int oy, int oz)
        {
            // quad a b c d around the face; orient by the outward direction (ox, oy, oz)
            double ux = m.V[3 * b] - m.V[3 * a], uy = m.V[3 * b + 1] - m.V[3 * a + 1], uz = m.V[3 * b + 2] - m.V[3 * a + 2];
            double vx = m.V[3 * c] - m.V[3 * a], vy = m.V[3 * c + 1] - m.V[3 * a + 1], vz = m.V[3 * c + 2] - m.V[3 * a + 2];
            double nxx = uy * vz - uz * vy, nyy = uz * vx - ux * vz, nzz = ux * vy - uy * vx;
            if (nxx * ox + nyy * oy + nzz * oz < 0) { (b, dd) = (dd, b); }
            tris.AddRange([a, b, c, a, c, dd]);
        }
        for (int j = 0; j < ny; j++)
            for (int k = 0; k < nz; k++)
            {
                Face(Id(0, j, k), Id(0, j + 1, k), Id(0, j + 1, k + 1), Id(0, j, k + 1), -1, 0, 0);
                Face(Id(nx, j, k), Id(nx, j + 1, k), Id(nx, j + 1, k + 1), Id(nx, j, k + 1), 1, 0, 0);
            }
        for (int i = 0; i < nx; i++)
            for (int k = 0; k < nz; k++)
            {
                Face(Id(i, 0, k), Id(i + 1, 0, k), Id(i + 1, 0, k + 1), Id(i, 0, k + 1), 0, -1, 0);
                Face(Id(i, ny, k), Id(i + 1, ny, k), Id(i + 1, ny, k + 1), Id(i, ny, k + 1), 0, 1, 0);
            }
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++)
            {
                Face(Id(i, j, 0), Id(i + 1, j, 0), Id(i + 1, j + 1, 0), Id(i, j + 1, 0), 0, 0, -1);
                Face(Id(i, j, nz), Id(i + 1, j, nz), Id(i + 1, j + 1, nz), Id(i, j + 1, nz), 0, 0, 1);
            }
        m.Tris = tris.ToArray();
        return m;
    }

    static double Det(float[] v, int a, int b, int c, int d)
    {
        double bx = v[3 * b] - v[3 * a], by = v[3 * b + 1] - v[3 * a + 1], bz = v[3 * b + 2] - v[3 * a + 2];
        double cx = v[3 * c] - v[3 * a], cy = v[3 * c + 1] - v[3 * a + 1], cz = v[3 * c + 2] - v[3 * a + 2];
        double dx = v[3 * d] - v[3 * a], dy = v[3 * d + 1] - v[3 * a + 1], dz = v[3 * d + 2] - v[3 * a + 2];
        return bx * (cy * dz - cz * dy) - by * (cx * dz - cz * dx) + bz * (cx * dy - cy * dx);
    }

    static SoftMeshData Box(double sx, double sy, double sz, int nx, int ny, int nz) =>
        Lattice(nx, ny, nz, (u, v, w) => (sx * (u - 0.5), sy * (v - 0.5), sz * (w - 0.5)));

    // A ball is the cube lattice with each point pulled onto the sphere by the ratio of its max-norm to its length.
    static SoftMeshData Ball(double r, int n) => Lattice(n, n, n, (u, v, w) =>
    {
        double x = 2 * u - 1, y = 2 * v - 1, z = 2 * w - 1;
        double inf = Math.Max(Math.Abs(x), Math.Max(Math.Abs(y), Math.Abs(z))), len = Math.Sqrt(x * x + y * y + z * z);
        double k = len > 0 ? inf / len : 0;
        return (r * x * k, r * y * k, r * z * k);
    });

    /// <summary>The distinct edges of the surface triangles and tetrahedra, as pairs sorted by (low, high).</summary>
    public static List<(int A, int B)> Edges(SoftMeshData m)
    {
        var set = new SortedSet<(int, int)>();
        void E(int a, int b) { set.Add(a < b ? (a, b) : (b, a)); }
        for (int t = 0; t < m.Tris.Length; t += 3) { E(m.Tris[t], m.Tris[t + 1]); E(m.Tris[t + 1], m.Tris[t + 2]); E(m.Tris[t + 2], m.Tris[t]); }
        for (int t = 0; t < m.Tets.Length; t += 4)
            for (int a = 0; a < 4; a++) for (int b = a + 1; b < 4; b++) E(m.Tets[t + a], m.Tets[t + b]);
        return set.ToList();
    }
}
