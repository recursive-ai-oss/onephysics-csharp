using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Water;

/// <summary>What happens at one side of a water grid.</summary>
internal enum WaterEdge : byte
{
    /// <summary>A closed wall: no flow through it, the water pushes on it.</summary>
    Wall = 0,

    /// <summary>Open: waves leave, nothing is held back (zero gradient).</summary>
    Open = 1,

    /// <summary>Inflow at a fixed speed.</summary>
    Inflow = 2,

    /// <summary>A held water level.</summary>
    Level = 3,
}

/// <summary>A source (positive rate) or drain (negative rate) of water at a point of the grid, in cubic metres per second (square metres per second in a planar world).</summary>
internal struct WaterSourceSpec
{
    public float X, Z;      // metres from the grid's minimum corner
    public float Radius;    // metres; 0 acts on the one cell holding the point
    public float Rate;
}

/// <summary>One body that currently displaces water here: the volume it displaced as of the last step and the footprint it was spread over.</summary>
internal struct WaterBodyRecord
{
    public int Slot;                // body slot; records are kept sorted by it
    public int Generation;
    public float Volume;            // the volume added to the water for this body: its submerged volume times the grid's displacement factor
    public float Submerged;         // the body's submerged volume as of the last step
    public float X0, X1, Z0, Z1;    // footprint, metres from the grid's minimum corner
}

/// <summary>
/// One body of shallow water on a regular grid: the depth and the depth-averaged momentum of every cell over a bed that is constant in each cell.
/// The scheme is a second-order finite volume method for the Saint-Venant equations: central-upwind fluxes (Kurganov and Petrova 2007, "A second-order well-balanced positivity preserving central-upwind scheme for the Saint-Venant system")
/// on states made hydrostatic at each face (Audusse, Bouchut, Bristeau, Klein and Perthame 2004, "A fast and stable well-balanced scheme with hydrostatic reconstruction for shallow water flows"),
/// which keeps a lake at rest exactly at rest over any bed, wet and dry cells included, stepped with two-stage strong-stability-preserving Runge-Kutta.
/// A planar grid (one row) is the one-dimensional scheme. Everything is float32 with IEEE-exact operations in a fixed order, so the answer does not depend on thread count or vector width.
/// </summary>
internal sealed partial class WaterGrid
{
    /// <summary>Depth below which a cell counts as dry: its momentum is zero and it holds no waves.</summary>
    public const float DryDepth = 1e-3f;
    private const float Eps4 = DryDepth * DryDepth * DryDepth * DryDepth;
    private const float Theta = 1.3f;   // generalised minmod limiter parameter, 1 (most diffusive) to 2 (most compressive)
    public const int MaxCellsPerGrid = 1 << 22;
    public const int MaxSources = 64;

    /// <summary>The deepest water a cell may hold, metres, and the fastest it may flow, m/s: a state beyond them (a step stretched past what the wave speed allows, a huge added volume) is cut back so that it stays finite and saveable.</summary>
    public const float MaxDepth = 4194304f, MaxSpeed = 1000f;

    public bool Planar;
    public int Nx, Nz;
    public float Dx;
    public double OriginX, OriginZ, BaseY;
    public float Density = 1000f, LinearDrag = 0.5f, AngularDrag = 0.5f, ViscousDamping = 1f, BottomFriction, Damping = 0.1f, Displacement = 0.5f;
    public float Cfl = 0.25f;
    public int MaxSubsteps = 16;
    public WaterEdge[] Edges = new WaterEdge[4];                        // west (-x), east (+x), south (-z), north (+z)
    public float[] EdgeLevel = new float[4], EdgeSpeed = new float[4];  // inflow surface level above BaseY, and inflow speed into the grid
    public WaterSourceSpec[] Sources = Array.Empty<WaterSourceSpec>();

    public float[] Bed = Array.Empty<float>();   // Nx*Nz bed heights at cell centres, metres above BaseY
    public float[] H = Array.Empty<float>(), Hu = Array.Empty<float>(), Hv = Array.Empty<float>();   // Nx*Nz cells
    public WaterBodyRecord[] Records = new WaterBodyRecord[8];
    public int RecordCount;

    // The volume that soft bodies, cloth and rods currently displace, per cell (cell index ascending; state): what the last step added for them, taken out again when it puts the new amounts in.
    public int[] HoldCells = Array.Empty<int>();
    public float[] HoldVolume = Array.Empty<float>();
    public int HoldCount;

    /// <summary>Water pushed below zero by a coupling edit or a step that was too long for the substep cap; a measure of lost volume in cubic metres.</summary>
    public double LostVolume;

    public int Cells => Nx * Nz;
    public float CellArea => Planar ? Dx : Dx * Dx;
    public double Width => Nx * (double)Dx;
    public double Depth => Planar ? 1.0 : Nz * (double)Dx;

    // Scratch (never saved): slopes, face fluxes, the stage-start state.
    private float[] _sxw = Array.Empty<float>(), _sxn = Array.Empty<float>(), _sxt = Array.Empty<float>();
    private float[] _szw = Array.Empty<float>(), _szn = Array.Empty<float>(), _szt = Array.Empty<float>();
    // Face fluxes: mass, normal momentum as the cell on the low side and the cell on the high side take it (they differ by the hydrostatic correction), momentum along the face.
    private float[] _fxh = Array.Empty<float>(), _fxl = Array.Empty<float>(), _fxu = Array.Empty<float>(), _fxr = Array.Empty<float>();
    private float[] _fzh = Array.Empty<float>(), _fzl = Array.Empty<float>(), _fzu = Array.Empty<float>(), _fzr = Array.Empty<float>();
    private float[] _h0 = Array.Empty<float>(), _u0 = Array.Empty<float>(), _v0 = Array.Empty<float>();
    private float[] _amaxX = Array.Empty<float>(), _amaxZ = Array.Empty<float>();

    public void Allocate()
    {
        int cells = Nx * Nz;
        H = new float[cells]; Hu = new float[cells]; Hv = new float[cells];
        Bed = new float[cells];
    }

    /// <summary>Allocates the scratch arrays (never saved). Call after the sizes are set.</summary>
    public void Derive()
    {
        int cells = Nx * Nz;
        _sxw = new float[cells]; _sxn = new float[cells]; _sxt = new float[cells];
        _szw = new float[cells]; _szn = new float[cells]; _szt = new float[cells];
        int xf = (Nx + 1) * Nz, zf = Nx * (Nz + 1);
        _fxh = new float[xf]; _fxl = new float[xf]; _fxu = new float[xf]; _fxr = new float[xf];
        _fzh = new float[zf]; _fzl = new float[zf]; _fzu = new float[zf]; _fzr = new float[zf];
        _h0 = new float[cells]; _u0 = new float[cells]; _v0 = new float[cells];
        int blocks = Blocks;
        _amaxX = new float[blocks]; _amaxZ = new float[blocks];
    }

    /// <summary>A structural copy: every array is copied and the derived data rebuilt (scratch is not shared).</summary>
    public WaterGrid Clone()
    {
        var c = new WaterGrid
        {
            Planar = Planar, Nx = Nx, Nz = Nz, Dx = Dx, OriginX = OriginX, OriginZ = OriginZ, BaseY = BaseY,
            Density = Density, LinearDrag = LinearDrag, AngularDrag = AngularDrag, ViscousDamping = ViscousDamping, BottomFriction = BottomFriction, Damping = Damping, Displacement = Displacement,
            Cfl = Cfl, MaxSubsteps = MaxSubsteps, LostVolume = LostVolume, RecordCount = RecordCount,
            Sources = (WaterSourceSpec[])Sources.Clone(), Bed = (float[])Bed.Clone(), H = (float[])H.Clone(), Hu = (float[])Hu.Clone(), Hv = (float[])Hv.Clone(),
            Records = (WaterBodyRecord[])Records.Clone(),
            HoldCells = (int[])HoldCells.Clone(), HoldVolume = (float[])HoldVolume.Clone(), HoldCount = HoldCount,
        };
        Array.Copy(Edges, c.Edges, 4); Array.Copy(EdgeLevel, c.EdgeLevel, 4); Array.Copy(EdgeSpeed, c.EdgeSpeed, 4);
        c.Derive();
        return c;
    }

    /// <summary>Rows per parallel task: a fixed number, so the split depends on the grid alone.</summary>
    private const int RowsPerTask = 4;

    private int Blocks => (Nz + RowsPerTask) / RowsPerTask;

    /// <summary>The surface height of cell (i, k) above BaseY: the depth over the bed at the cell's centre.</summary>
    public float SurfaceOfCell(int i, int k) => H[k * Nx + i] + Bed[k * Nx + i];

    public float BedOfCell(int i, int k) => Bed[k * Nx + i];

    /// <summary>Total water volume in cubic metres, summed in cell order in double.</summary>
    public double Volume()
    {
        double v = 0;
        for (int c = 0; c < H.Length; c++) { v += H[c]; }
        return v * CellArea;
    }

    /// <summary>Largest |velocity| plus wave speed over the cells, from the current state.</summary>
    public float MaxWaveSpeed(float g)
    {
        float m = 0f;
        for (int c = 0; c < H.Length; c++)
        {
            float h = H[c];
            if (!(h > DryDepth)) { continue; }
            float u = Hu[c] / h, v = Hv[c] / h;
            float s = float.Sqrt(u * u + v * v) + float.Sqrt(g * h);
            if (s > m) { m = s; }
        }

        return m;
    }

    // ---------------------------------------------------------------- sampling

    /// <summary>Locates a point in cell coordinates (cell centres at integer values) clamped to the grid; returns false when the point lies outside the grid's footprint.</summary>
    public bool Locate(double x, double z, out float fx, out float fz)
    {
        double lx = (x - OriginX) / Dx, lz = Planar ? 0.5 : (z - OriginZ) / Dx;
        bool inside = lx >= 0 && lx <= Nx && lz >= 0 && lz <= Nz;
        double cx = lx - 0.5, cz = lz - 0.5;
        if (cx < 0) { cx = 0; } else if (cx > Nx - 1) { cx = Nx - 1; }
        if (cz < 0) { cz = 0; } else if (cz > Nz - 1) { cz = Nz - 1; }
        fx = (float)cx; fz = (float)cz;
        return inside;
    }

    /// <summary>Bilinear sample of the surface height (above BaseY), the depth and the velocity at a point given in cell coordinates from <see cref="Locate"/>.</summary>
    public void Sample(float fx, float fz, out float surface, out float depth, out float u, out float v)
    {
        int i0 = (int)fx, k0 = (int)fz;
        if (i0 > Nx - 2) { i0 = Math.Max(Nx - 2, 0); }
        if (k0 > Nz - 2) { k0 = Math.Max(Nz - 2, 0); }
        int i1 = Math.Min(i0 + 1, Nx - 1), k1 = Math.Min(k0 + 1, Nz - 1);
        float tx = fx - i0, tz = fz - k0;
        if (i1 == i0) { tx = 0f; }
        if (k1 == k0) { tz = 0f; }
        float w00 = (1f - tx) * (1f - tz), w10 = tx * (1f - tz), w01 = (1f - tx) * tz, w11 = tx * tz;
        int c00 = k0 * Nx + i0, c10 = k0 * Nx + i1, c01 = k1 * Nx + i0, c11 = k1 * Nx + i1;
        surface = w00 * (H[c00] + Bed[c00]) + w10 * (H[c10] + Bed[c10]) + w01 * (H[c01] + Bed[c01]) + w11 * (H[c11] + Bed[c11]);
        depth = w00 * H[c00] + w10 * H[c10] + w01 * H[c01] + w11 * H[c11];
        u = w00 * Vel(H[c00], Hu[c00]) + w10 * Vel(H[c10], Hu[c10]) + w01 * Vel(H[c01], Hu[c01]) + w11 * Vel(H[c11], Hu[c11]);
        v = w00 * Vel(H[c00], Hv[c00]) + w10 * Vel(H[c10], Hv[c10]) + w01 * Vel(H[c01], Hv[c01]) + w11 * Vel(H[c11], Hv[c11]);
    }

    /// <summary>The bed height under a point (the cell's, since the bed is constant in a cell), above BaseY; clamped to the grid.</summary>
    public float BedAt(double x, double z)
    {
        double lx = (x - OriginX) / Dx, lz = Planar ? 0.0 : (z - OriginZ) / Dx;
        int i = (int)Math.Min(Math.Max(lx, 0.0), Nx - 1), k = (int)Math.Min(Math.Max(lz, 0.0), Nz - 1);
        return Bed[k * Nx + i];
    }

    // ---------------------------------------------------------------- edits

    /// <summary>
    /// Adds <paramref name="volume"/> (negative removes) spread over the cells a footprint overlaps, in proportion to the overlap. Removal never takes a cell below zero depth: what cannot be taken is counted in <see cref="LostVolume"/>.
    /// </summary>
    public void AddVolume(float x0, float x1, float z0, float z1, float volume)
    {
        if (volume == 0f) { return; }
        float inv = 1f / Dx;
        float a0 = x0 * inv, a1 = x1 * inv, b0 = Planar ? 0f : z0 * inv, b1 = Planar ? 1f : z1 * inv;
        a0 = Math.Max(a0, 0f); a1 = Math.Min(a1, Nx); b0 = Math.Max(b0, 0f); b1 = Math.Min(b1, Nz);
        // A footprint wholly outside the grid adds nothing.
        if (!(a1 >= a0) || !(b1 >= b0)) { return; }

        int i0 = Math.Min((int)a0, Nx - 1), i1 = Math.Min((int)a1, Nx - 1), k0 = Math.Min((int)b0, Nz - 1), k1 = Math.Min((int)b1, Nz - 1);
        double total = 0;
        for (int k = k0; k <= k1; k++)
        {
            float oz = Math.Min(b1, k + 1) - Math.Max(b0, k);
            for (int i = i0; i <= i1; i++)
            {
                float ox = Math.Min(a1, i + 1) - Math.Max(a0, i);
                total += (double)(ox > 0f ? ox : 0f) * (oz > 0f ? oz : 0f);
            }
        }

        if (!(total > 0))
        {
            // Zero-area footprint: put everything on the cell holding it.
            int c = k0 * Nx + i0;
            Apply(c, volume / CellArea);
            return;
        }

        float perArea = (float)(volume / (total * CellArea));
        for (int k = k0; k <= k1; k++)
        {
            float oz = Math.Min(b1, k + 1) - Math.Max(b0, k);
            for (int i = i0; i <= i1; i++)
            {
                float ox = Math.Min(a1, i + 1) - Math.Max(a0, i);
                if (ox > 0f && oz > 0f) { Apply(k * Nx + i, perArea * ox * oz); }
            }
        }
    }

    // The running total of lost water, kept finite and non-negative whatever was lost.
    private static double AddLost(double total, double lost)
    {
        double t = total + lost;
        return t >= 0.0 && t < 1e30 ? t : double.IsNaN(t) || t < 0.0 ? total : 1e30;
    }

    /// <summary>Adds (or removes, if negative) a volume of water to one cell; removal never takes the cell below zero depth.</summary>
    public void AddToCell(int cell, float volume) => Apply(cell, volume / CellArea);

    private void Apply(int c, float dh)
    {
        float h = H[c] + dh;
        if (h < 0f) { LostVolume = AddLost(LostVolume, -(double)h * CellArea); h = 0f; }
        else if (!(h <= MaxDepth)) { h = MaxDepth; }
        H[c] = h;
        if (!(h > DryDepth)) { Hu[c] = 0f; Hv[c] = 0f; }
    }

    /// <summary>Fills the cells with water up to a level above BaseY (nothing where the bed is higher); momentum is zeroed.</summary>
    public void Fill(float level)
    {
        for (int c = 0; c < H.Length; c++)
        {
            float d = level - Bed[c];
            H[c] = d > 0f ? d : 0f; Hu[c] = 0f; Hv[c] = 0f;
        }
    }

    // ---------------------------------------------------------------- scheme pieces

    /// <summary>Velocity from depth and momentum with the desingularisation that keeps it finite in nearly dry cells (Kurganov and Petrova 2007, eq. 2.17).</summary>
    public static float Vel(float h, float q)
    {
        float h4 = h * h * h * h;
        float m = h4 > Eps4 ? h4 : Eps4;
        return 1.41421356f * h * q / float.Sqrt(h4 + m);
    }

    /// <summary>The hydrostatic pressure force per unit width of water of depth h, divided by the density: g h² / 2.</summary>
    private static float Pressure(float h, float g) => 0.5f * g * h * h;

    private static float MinMod3(float a, float b, float c)
    {
        if (a > 0f && b > 0f && c > 0f) { return Math.Min(a, Math.Min(b, c)); }
        if (a < 0f && b < 0f && c < 0f) { return Math.Max(a, Math.Max(b, c)); }
        return 0f;
    }

    private static float Slope(float l, float m, float r)
    {
        float a = Theta * (m - l), c = Theta * (r - m), b = 0.5f * (r - l);
        return MinMod3(a, b, c);
    }

    /// <summary>
    /// The central-upwind numerical flux across a face between a left and a right state (depth, velocity normal to the face, velocity along it)
    /// and the largest signal speed it saw. Kurganov and Petrova 2007, eq. 2.9, with the hydrostatic pressure in the normal momentum.
    /// </summary>
    private static void CentralUpwind(float hl, float ul, float vl, float hr, float ur, float vr, float g, out float fh, out float fq, out float fr, out float amax)
    {
        float cl = float.Sqrt(g * hl), cr = float.Sqrt(g * hr);
        if (hl == hr && ul == ur && vl == vr)
        {
            // Equal states: the flux is the physical flux itself, computed exactly the way the pressure correction is, so a lake at rest cancels to the last bit.
            amax = Math.Abs(ul) + cl;
            fh = hl * ul; fq = hl * ul * ul + Pressure(hl, g); fr = hl * ul * vl;
            return;
        }

        float ap = Math.Max(Math.Max(ul + cl, ur + cr), 0f);
        float am = Math.Min(Math.Min(ul - cl, ur - cr), 0f);
        amax = Math.Max(ap, -am);
        float d = ap - am;
        if (!(d > 1e-7f)) { fh = 0f; fq = 0f; fr = 0f; return; }
        float k = ap * am / d;
        float ql = hl * ul, qr = hr * ur, rl = ql * vl, rr = qr * vr;
        fh = (ap * ql - am * qr) / d + k * (hr - hl);
        fq = (ap * (ql * ul + 0.5f * g * hl * hl) - am * (qr * ur + 0.5f * g * hr * hr)) / d + k * (qr - ql);
        fr = (ap * rl - am * rr) / d + k * (hr * vr - hl * vl);
    }
}
