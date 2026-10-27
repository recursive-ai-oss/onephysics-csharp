using System;
using System.Numerics;

namespace RecursiveAI.OnePhysics.Experiments.Batching;

/// <summary>Scene and step options for <see cref="Pile"/>.</summary>
internal readonly record struct PileOptions(int Side, int Layers, int Substeps, int Colors)
{
    /// <summary>4096 spheres (16 x 16 x 16) in a walled box. Ten colours is deliberately fewer than the pile needs, so every run exercises the overflow set.</summary>
    public static PileOptions Large => new(16, 16, 4, 10);

    /// <summary>512 spheres (8 x 8 x 8), for slow platforms.</summary>
    public static PileOptions Small => new(8, 8, 4, 10);
}

/// <summary>
/// A pile of unit-mass spheres in a box, stepped with a soft-step sequential-impulse contact solver (Catto, "Solver2D" and
/// "Soft Step" 2024; sequential impulses, GDC 2005/2006). A prototype for the width-independence test, not the engine's solver.
/// <list type="bullet">
/// <item>All state is float32. Positions are relative to the box corner; the double-position rule is not exercised here.</item>
/// <item>Contacts: sphere-sphere and sphere-plane, found with a uniform grid and listed in key order (lower body, then higher
/// body or plane), so a contact keeps its place in the list from step to step and warm starting is a merge.</item>
/// <item>Colouring: contacts in list order take the lowest colour (below <c>Colors</c>) not yet used by either dynamic body;
/// a colour's contacts fill batches of <see cref="L"/> slots in list order. Contacts that find no colour form the overflow set,
/// solved serially. Neither the colouring nor the batches know the SIMD width or the thread count.</item>
/// <item>Solver: per substep integrate velocities, warm start, solve with soft bias, integrate positions, relax. Friction is
/// solved before the normal in two tangent directions. No restitution, no rolling resistance, and the anchors of a contact do
/// not rotate within a step (exact for spheres to first order).</item>
/// </list>
/// </summary>
internal sealed class Pile : IDisposable
{
    /// <summary>Fixed logical lane count of a batch, chosen once and independent of the hardware.</summary>
    public const int L = 16;

    // Body record (BodyStride floats per body): velocity, angular velocity, position change this step, masses, position.
    public const int BodyStride = 16;
    public const int VX = 0, VY = 1, VZ = 2, WX = 3, WY = 4, WZ = 5, DX = 6, DY = 7, DZ = 8, IM = 9, II = 10, PX = 11, PY = 12, PZ = 13;

    // Solver slot fields: field f of slot s is Data[f * Stride + s], so consecutive slots are consecutive floats.
    public const int NX = 0, T1X = 3, T2X = 6, RAN = 9, RBN = 12, RAT1 = 15, RBT1 = 18, RAT2 = 21, RBT2 = 24;
    public const int Sep0 = 27, FRIC = 28, MASSN = 29, MASST1 = 30, MASST2 = 31, LAMN = 32, LAMT1 = 33, LAMT2 = 34, FieldCount = 35;

    private const float Radius = 0.5f;
    private const float SpeculativeDistance = 0.04f;
    private const float Friction = 0.6f;
    private const float Gravity = 9.81f;
    private const int PlaneCount = 5;

    private struct Contact
    {
        public long Key;
        public int A, B;
        public float Nx, Ny, Nz, Sep;
        public float Rax, Ray, Raz, Rbx, Rby, Rbz;
        public float LamN, Px, Py, Pz; // accumulated normal impulse and friction impulse as a world-space vector
        public int Slot;
    }

    private readonly PileOptions _options;
    private readonly float _dt;
    private readonly float _h;
    private readonly float _width;
    private readonly float[] _planeNormal = new float[PlaneCount * 3];
    private readonly float[] _planeOffset = new float[PlaneCount];

    private Contact[] _contacts = new Contact[1024];
    private Contact[] _previous = new Contact[1024];
    private int _contactCount;
    private int _previousCount;

    // Grid.
    private readonly int _gx, _gy, _gz;
    private readonly int[] _cellStart;
    private readonly int[] _cellItems;
    private readonly int[] _cellOf;

    // Colouring.
    private readonly ulong[] _usedColors;
    private readonly int[] _colorCount;
    private readonly int[] _colorStart; // slot where each colour begins, then the overflow start
    private int _overflowCount;
    private int _slotCount;

    private readonly SpinPool? _pool;
    private readonly int _threads;
    private readonly object?[] _jobs = new object?[5];

    public Pile(PileOptions options, int threads)
    {
        _options = options;
        _threads = threads;
        _dt = 1f / 60f;
        _h = _dt / options.Substeps;
        N = options.Side * options.Side * options.Layers;
        _width = options.Side * 1.0f + 0.6f;
        Body = new float[(N + 1) * BodyStride];
        IA = Array.Empty<int>();
        IB = Array.Empty<int>();
        Data = Array.Empty<float>();
        Stride = 0;

        // Planes: floor, then four walls. dist = dot(n, p) - offset; normals point into the box.
        SetPlane(0, 0, 1, 0, 0f);
        SetPlane(1, 1, 0, 0, 0f);
        SetPlane(2, -1, 0, 0, -_width);
        SetPlane(3, 0, 0, 1, 0f);
        SetPlane(4, 0, 0, -1, -_width);

        // A jittered lattice with horizontal slack, so the spheres fall into a packed heap. The jitter is a fixed linear
        // congruential sequence (integer arithmetic, exact everywhere).
        uint state = 12345;
        int i = 0;
        for (int y = 0; y < options.Layers; y++)
        {
            for (int z = 0; z < options.Side; z++)
            {
                for (int x = 0; x < options.Side; x++, i++)
                {
                    float jx = Jitter(ref state), jy = Jitter(ref state), jz = Jitter(ref state);
                    int o = i * BodyStride;
                    Body[o + PX] = 0.55f + x * 1.0f + (y & 1) * 0.3f + jx * 0.1f;
                    Body[o + PY] = 0.6f + y * 1.04f + jy * 0.02f;
                    Body[o + PZ] = 0.55f + z * 1.0f + ((y >> 1) & 1) * 0.3f + jz * 0.1f;
                    Body[o + IM] = 1f;
                    Body[o + II] = 1f / (0.4f * Radius * Radius); // solid sphere, unit mass
                }
            }
        }
        // Slot N is the static body that plane contacts and padding lanes refer to: zero inverse masses.

        _gx = (int)(_width + 1f) + 1;
        _gz = _gx;
        _gy = (int)(options.Layers * 1.04f + 2f) + 1;
        _cellStart = new int[_gx * _gy * _gz + 1];
        _cellItems = new int[N];
        _cellOf = new int[N];
        _usedColors = new ulong[N + 1];
        _colorCount = new int[options.Colors];
        _colorStart = new int[options.Colors + 2];
        if (threads > 1)
        {
            _pool = new SpinPool(threads);
        }
    }

    public int N { get; }
    public float[] Body { get; }
    public int[] IA { get; private set; }
    public int[] IB { get; private set; }
    public float[] Data { get; private set; }
    public int Stride { get; private set; }
    public int Colors => _options.Colors;
    public int ContactCount => _contactCount;
    public int OverflowCount => _overflowCount;
    public int SlotCount => _slotCount;
    public int ColorsUsed { get { int u = 0; for (int c = 0; c < _colorCount.Length; c++) { if (_colorCount[c] > 0) { u = c + 1; } } return u; } }
    public int ColorStart(int color) => _colorStart[color];
    public int OverflowStart => _colorStart[_options.Colors];

    private static float Jitter(ref uint state)
    {
        state = unchecked(state * 1664525u + 1013904223u);
        return ((state >> 8) & 0xFFFF) / 65536f - 0.5f;
    }

    private void SetPlane(int k, float nx, float ny, float nz, float offset)
    {
        _planeNormal[k * 3] = nx;
        _planeNormal[k * 3 + 1] = ny;
        _planeNormal[k * 3 + 2] = nz;
        _planeOffset[k] = offset;
    }

    /// <summary>Advances the pile by one fixed step with kernels of the given width.</summary>
    public void Step(Width width)
    {
        Collide();
        SolveSubsteps(width);
        StoreImpulses();
    }

    // ---- Narrowphase ------------------------------------------------------------------------------------------------

    /// <summary>Finds contacts, warm starts them from the previous step, colours them and fills the solver slots.</summary>
    public void Collide()
    {
        FindContacts();
        WarmStartFromPrevious();
        Colour();
        Prepare();
    }

    private static int CellIndex(float p, int max)
    {
        int c = (int)p; // positions inside the box are non-negative, so truncation is floor
        return c < 0 ? 0 : (c >= max ? max - 1 : c);
    }

    private void FindContacts()
    {
        (_previous, _contacts) = (_contacts, _previous);
        _previousCount = _contactCount;
        _contactCount = 0;

        // Counting sort of spheres into grid cells; within a cell, spheres stay in index order.
        Array.Clear(_cellStart);
        for (int i = 0; i < N; i++)
        {
            int o = i * BodyStride;
            int c = CellIndex(Body[o + PX], _gx) + _gx * (CellIndex(Body[o + PY], _gy) + _gy * CellIndex(Body[o + PZ], _gz));
            _cellOf[i] = c;
            _cellStart[c + 1]++;
        }
        for (int c = 0; c < _cellStart.Length - 1; c++)
        {
            _cellStart[c + 1] += _cellStart[c];
        }
        var cursor = new int[_cellStart.Length - 1];
        for (int i = 0; i < N; i++)
        {
            int c = _cellOf[i];
            _cellItems[_cellStart[c] + cursor[c]++] = i;
        }

        float reach = 2 * Radius + SpeculativeDistance;
        Span<int> near = stackalloc int[64];
        for (int i = 0; i < N; i++)
        {
            int oi = i * BodyStride;
            float xi = Body[oi + PX], yi = Body[oi + PY], zi = Body[oi + PZ];
            int cx = CellIndex(xi, _gx), cy = CellIndex(yi, _gy), cz = CellIndex(zi, _gz);
            int count = 0;
            for (int dz = -1; dz <= 1; dz++)
            {
                int z = cz + dz;
                if (z < 0 || z >= _gz) { continue; }
                for (int dy = -1; dy <= 1; dy++)
                {
                    int y = cy + dy;
                    if (y < 0 || y >= _gy) { continue; }
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int x = cx + dx;
                        if (x < 0 || x >= _gx) { continue; }
                        int c = x + _gx * (y + _gy * z);
                        for (int k = _cellStart[c]; k < _cellStart[c + 1]; k++)
                        {
                            int j = _cellItems[k];
                            if (j <= i) { continue; }
                            int oj = j * BodyStride;
                            float ex = Body[oj + PX] - xi, ey = Body[oj + PY] - yi, ez = Body[oj + PZ] - zi;
                            if (ex * ex + ey * ey + ez * ez < reach * reach)
                            {
                                if (count == near.Length) { throw new InvalidOperationException("Too many neighbours for one sphere."); }
                                near[count++] = j;
                            }
                        }
                    }
                }
            }
            // Ascending partner index: insertion sort, so the list order never depends on the grid traversal.
            for (int a = 1; a < count; a++)
            {
                int v = near[a], b = a - 1;
                while (b >= 0 && near[b] > v) { near[b + 1] = near[b]; b--; }
                near[b + 1] = v;
            }
            for (int a = 0; a < count; a++)
            {
                AddSphereContact(i, near[a]);
            }
            for (int k = 0; k < PlaneCount; k++)
            {
                AddPlaneContact(i, k);
            }
        }
    }

    private ref Contact NewContact()
    {
        if (_contactCount == _contacts.Length)
        {
            Array.Resize(ref _contacts, _contacts.Length * 2);
        }
        ref Contact c = ref _contacts[_contactCount++];
        c = default;
        return ref c;
    }

    private void AddSphereContact(int i, int j)
    {
        int oi = i * BodyStride, oj = j * BodyStride;
        float ex = Body[oj + PX] - Body[oi + PX], ey = Body[oj + PY] - Body[oi + PY], ez = Body[oj + PZ] - Body[oi + PZ];
        float dist = MathF.Sqrt(ex * ex + ey * ey + ez * ez);
        float nx = 0f, ny = 1f, nz = 0f;
        if (dist > 0f)
        {
            nx = ex / dist; ny = ey / dist; nz = ez / dist;
        }
        float sep = dist - 2 * Radius;
        float arm = Radius + 0.5f * sep; // the contact point is midway between the surfaces
        ref Contact c = ref NewContact();
        c.Key = ((long)i << 32) | (uint)j;
        c.A = i; c.B = j;
        c.Nx = nx; c.Ny = ny; c.Nz = nz; c.Sep = sep;
        c.Rax = nx * arm; c.Ray = ny * arm; c.Raz = nz * arm;
        c.Rbx = -c.Rax; c.Rby = -c.Ray; c.Rbz = -c.Raz;
    }

    private void AddPlaneContact(int i, int k)
    {
        int oi = i * BodyStride;
        float px = _planeNormal[k * 3], py = _planeNormal[k * 3 + 1], pz = _planeNormal[k * 3 + 2];
        float sep = (px * Body[oi + PX] + py * Body[oi + PY] + pz * Body[oi + PZ]) - _planeOffset[k] - Radius;
        if (sep >= SpeculativeDistance)
        {
            return;
        }
        ref Contact c = ref NewContact();
        c.Key = ((long)i << 32) | (uint)(N + 1 + k);
        c.A = i; c.B = N; // body N is the static body
        c.Nx = -px; c.Ny = -py; c.Nz = -pz; c.Sep = sep;
        c.Rax = c.Nx * Radius; c.Ray = c.Ny * Radius; c.Raz = c.Nz * Radius;
    }

    private void WarmStartFromPrevious()
    {
        // Both lists are in ascending key order, so matching is a merge.
        int p = 0;
        for (int c = 0; c < _contactCount; c++)
        {
            ref Contact cur = ref _contacts[c];
            while (p < _previousCount && _previous[p].Key < cur.Key) { p++; }
            if (p < _previousCount && _previous[p].Key == cur.Key)
            {
                cur.LamN = _previous[p].LamN;
                cur.Px = _previous[p].Px; cur.Py = _previous[p].Py; cur.Pz = _previous[p].Pz;
            }
        }
    }

    // ---- Colouring and batches -------------------------------------------------------------------------------------

    private void Colour()
    {
        int colors = _options.Colors;
        Array.Clear(_usedColors);
        Array.Clear(_colorCount);
        _overflowCount = 0;
        for (int c = 0; c < _contactCount; c++)
        {
            ref Contact k = ref _contacts[c];
            ulong used = 0;
            if (k.A < N) { used |= _usedColors[k.A]; }
            if (k.B < N) { used |= _usedColors[k.B]; }
            int color = BitOperations.TrailingZeroCount(~used);
            if (color >= colors)
            {
                k.Slot = -1; // overflow
                _overflowCount++;
                continue;
            }
            if (k.A < N) { _usedColors[k.A] |= 1UL << color; }
            if (k.B < N) { _usedColors[k.B] |= 1UL << color; }
            k.Slot = color; // the colour for now; the slot is assigned below
            _colorCount[color]++;
        }
        int slot = 0;
        for (int color = 0; color < colors; color++)
        {
            _colorStart[color] = slot;
            slot += (_colorCount[color] + L - 1) / L * L;
        }
        _colorStart[colors] = slot; // overflow starts here
        _slotCount = slot + _overflowCount;
        _colorStart[colors + 1] = _slotCount;

        Span<int> next = stackalloc int[64];
        for (int color = 0; color < colors; color++) { next[color] = _colorStart[color]; }
        int overflow = slot;
        for (int c = 0; c < _contactCount; c++)
        {
            ref Contact k = ref _contacts[c];
            k.Slot = k.Slot < 0 ? overflow++ : next[k.Slot]++;
        }
    }

    private void Prepare()
    {
        int stride = _slotCount + L;
        if (Stride < stride)
        {
            Stride = stride + stride / 4;
            Data = new float[FieldCount * Stride];
            IA = new int[Stride];
            IB = new int[Stride];
        }
        else
        {
            Array.Clear(Data);
        }
        Array.Fill(IA, N);
        Array.Fill(IB, N);
        float[] d = Data;
        int s = Stride;

        // Soft contact coefficients (Catto, soft constraints): stiffness from a frequency and damping ratio, per substep.
        for (int c = 0; c < _contactCount; c++)
        {
            ref Contact k = ref _contacts[c];
            int slot = k.Slot;
            IA[slot] = k.A; IB[slot] = k.B;
            float imA = Body[k.A * BodyStride + IM], iiA = Body[k.A * BodyStride + II];
            float imB = Body[k.B * BodyStride + IM], iiB = Body[k.B * BodyStride + II];

            // Tangent basis from the normal: a fixed rule, so the same normal always gives the same basis.
            float t1x, t1y, t1z;
            if ((k.Nx < 0f ? -k.Nx : k.Nx) >= 0.57735f)
            {
                t1x = k.Ny; t1y = -k.Nx; t1z = 0f;
            }
            else
            {
                t1x = 0f; t1y = k.Nz; t1z = -k.Ny;
            }
            float tl = MathF.Sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
            t1x /= tl; t1y /= tl; t1z /= tl;
            float t2x = k.Ny * t1z - k.Nz * t1y, t2y = k.Nz * t1x - k.Nx * t1z, t2z = k.Nx * t1y - k.Ny * t1x;

            Put3(d, NX, slot, s, k.Nx, k.Ny, k.Nz);
            Put3(d, T1X, slot, s, t1x, t1y, t1z);
            Put3(d, T2X, slot, s, t2x, t2y, t2z);
            float massN = Arm(d, RAN, RBN, slot, s, k, k.Nx, k.Ny, k.Nz, imA, iiA, imB, iiB);
            float massT1 = Arm(d, RAT1, RBT1, slot, s, k, t1x, t1y, t1z, imA, iiA, imB, iiB);
            float massT2 = Arm(d, RAT2, RBT2, slot, s, k, t2x, t2y, t2z, imA, iiA, imB, iiB);
            d[Sep0 * s + slot] = k.Sep;
            d[FRIC * s + slot] = Friction;
            d[MASSN * s + slot] = massN;
            d[MASST1 * s + slot] = massT1;
            d[MASST2 * s + slot] = massT2;
            d[LAMN * s + slot] = k.LamN;
            // The stored friction impulse is a world vector; project it onto this step's basis.
            d[LAMT1 * s + slot] = k.Px * t1x + k.Py * t1y + k.Pz * t1z;
            d[LAMT2 * s + slot] = k.Px * t2x + k.Py * t2y + k.Pz * t2z;
        }
    }

    private static void Put3(float[] d, int field, int slot, int stride, float x, float y, float z)
    {
        d[field * stride + slot] = x;
        d[(field + 1) * stride + slot] = y;
        d[(field + 2) * stride + slot] = z;
    }

    // Stores rA x dir and rB x dir and returns the effective mass along dir.
    private static float Arm(float[] d, int fa, int fb, int slot, int stride, in Contact k, float x, float y, float z, float imA, float iiA, float imB, float iiB)
    {
        float ax = k.Ray * z - k.Raz * y, ay = k.Raz * x - k.Rax * z, az = k.Rax * y - k.Ray * x;
        float bx = k.Rby * z - k.Rbz * y, by = k.Rbz * x - k.Rbx * z, bz = k.Rbx * y - k.Rby * x;
        Put3(d, fa, slot, stride, ax, ay, az);
        Put3(d, fb, slot, stride, bx, by, bz);
        float kk = imA + imB + iiA * (ax * ax + ay * ay + az * az) + iiB * (bx * bx + by * by + bz * bz);
        return kk > 0f ? 1f / kk : 0f;
    }

    // ---- Solve -------------------------------------------------------------------------------------------------------

    /// <summary>The substep loop: gravity, warm start, biased solve, move, relax.</summary>
    public void SolveSubsteps(Width width)
    {
        float h = _h;
        float hertz = Math.Min(30f, 0.25f / h);
        float omega = 6.2831855f * hertz;
        const float zeta = 10f;
        float a1 = 2f * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1f / (1f + a2);
        var soft = new SoftParams(omega / a1, a2 * a3, a3, 1f / h, 3f);

        for (int i = 0; i < N; i++)
        {
            int o = i * BodyStride;
            Body[o + DX] = 0f; Body[o + DY] = 0f; Body[o + DZ] = 0f;
        }
        for (int sub = 0; sub < _options.Substeps; sub++)
        {
            for (int i = 0; i < N; i++)
            {
                Body[i * BodyStride + VY] -= Gravity * h;
            }
            Phase(width, KernelMode.WarmStart, soft);
            Phase(width, KernelMode.Solve, soft);
            for (int i = 0; i < N; i++)
            {
                int o = i * BodyStride;
                float mx = Body[o + VX] * h, my = Body[o + VY] * h, mz = Body[o + VZ] * h;
                Body[o + PX] += mx; Body[o + PY] += my; Body[o + PZ] += mz;
                Body[o + DX] += mx; Body[o + DY] += my; Body[o + DZ] += mz;
            }
            Phase(width, KernelMode.Relax, soft);
        }
    }

    private void Phase(Width width, KernelMode mode, SoftParams soft)
    {
        switch (width)
        {
            case Width.Scalar: Phase<Lanes1>(0, mode, soft); break;
            case Width.V128: Phase<Lanes128>(1, mode, soft); break;
            case Width.V256: Phase<Lanes256>(2, mode, soft); break;
            case Width.V512: Phase<Lanes512>(3, mode, soft); break;
            case Width.VectorT: Phase<LanesT>(4, mode, soft); break;
            default: throw new ArgumentOutOfRangeException(nameof(width));
        }
    }

    private void Phase<T>(int jobIndex, KernelMode mode, SoftParams soft) where T : struct, ILane<T>
    {
        // Overflow constraints first, one lane at a time and in list order.
        if (_overflowCount > 0)
        {
            Kernels.Run<Lanes1>(this, OverflowStart, _slotCount, mode, soft);
        }
        var job = (ColorJob<T>)(_jobs[jobIndex] ??= new ColorJob<T>(this));
        job.Mode = mode;
        job.Soft = soft;
        for (int c = 0; c < _options.Colors; c++)
        {
            int begin = _colorStart[c], end = _colorStart[c + 1]; // the entry after the last colour is the overflow start
            if (end == begin) { continue; }
            int batches = (end - begin) / L;
            if (_pool is null || batches < 2)
            {
                Kernels.Run<T>(this, begin, end, mode, soft);
            }
            else
            {
                job.Begin = begin;
                job.Batches = batches;
                _pool.Run(job);
            }
        }
    }

    // Splits a colour's batches into contiguous chunks, one per thread. Batches of a colour touch disjoint bodies, so the
    // split cannot change a result.
    private sealed class ColorJob<T> : IPartJob where T : struct, ILane<T>
    {
        private readonly Pile _pile;
        public int Begin, Batches;
        public KernelMode Mode;
        public SoftParams Soft;

        public ColorJob(Pile pile) => _pile = pile;

        public void Execute(int part)
        {
            int parts = _pile._threads;
            int lo = Batches * part / parts, hi = Batches * (part + 1) / parts;
            if (hi > lo)
            {
                Kernels.Run<T>(_pile, Begin + lo * L, Begin + hi * L, Mode, Soft);
            }
        }
    }

    // ---- Results -----------------------------------------------------------------------------------------------------

    public void StoreImpulses()
    {
        float[] d = Data;
        int s = Stride;
        for (int c = 0; c < _contactCount; c++)
        {
            ref Contact k = ref _contacts[c];
            int slot = k.Slot;
            k.LamN = d[LAMN * s + slot];
            float l1 = d[LAMT1 * s + slot], l2 = d[LAMT2 * s + slot];
            k.Px = l1 * d[T1X * s + slot] + l2 * d[T2X * s + slot];
            k.Py = l1 * d[(T1X + 1) * s + slot] + l2 * d[(T2X + 1) * s + slot];
            k.Pz = l1 * d[(T1X + 2) * s + slot] + l2 * d[(T2X + 2) * s + slot];
        }
    }

    /// <summary>
    /// Folds the whole state (bodies, then contact impulses in list order) into two 64-bit words with a multiply-xor hash.
    /// Every 32-bit word changes the result, so any differing bit is caught; it is a check, not a security hash.
    /// </summary>
    public (long, long) StateDigest()
    {
        ulong h1 = 0xcbf29ce484222325UL, h2 = 0x9e3779b97f4a7c15UL;
        for (int i = 0; i < N; i++)
        {
            int o = i * BodyStride;
            for (int f = 0; f < 6; f++) { Fold(ref h1, ref h2, Body[o + f]); }
            for (int f = PX; f <= PZ; f++) { Fold(ref h1, ref h2, Body[o + f]); }
        }
        for (int c = 0; c < _contactCount; c++)
        {
            Fold(ref h1, ref h2, _contacts[c].LamN);
            Fold(ref h1, ref h2, _contacts[c].Px);
            Fold(ref h1, ref h2, _contacts[c].Py);
            Fold(ref h1, ref h2, _contacts[c].Pz);
        }
        return ((long)h1, (long)h2);
    }

    private static void Fold(ref ulong h1, ref ulong h2, float v)
    {
        uint bits = float.IsNaN(v) ? 0x7fc00000u : BitConverter.SingleToUInt32Bits(v);
        h1 = (h1 ^ bits) * 0x100000001b3UL;
        h2 = (h2 ^ bits) * 0xff51afd7ed558ccdUL;
        h2 ^= h2 >> 29;
    }

    /// <summary>Kinetic energy, the deepest penetration and the highest speed of the current state (contacts as of the last collide).</summary>
    public (float KineticEnergy, float MaxPenetration, float MaxSpeed) Metrics()
    {
        float ke = 0f, speed = 0f, pen = 0f;
        for (int i = 0; i < N; i++)
        {
            int o = i * BodyStride;
            float v2 = Body[o + VX] * Body[o + VX] + Body[o + VY] * Body[o + VY] + Body[o + VZ] * Body[o + VZ];
            float w2 = Body[o + WX] * Body[o + WX] + Body[o + WY] * Body[o + WY] + Body[o + WZ] * Body[o + WZ];
            ke += 0.5f * v2 + 0.5f * w2 / Body[o + II];
            speed = Math.Max(speed, MathF.Sqrt(v2));
        }
        for (int c = 0; c < _contactCount; c++)
        {
            pen = Math.Max(pen, -_contacts[c].Sep);
        }
        return (ke, pen, speed);
    }

    /// <summary>True when every body value is finite.</summary>
    public bool IsFinite()
    {
        foreach (float f in Body)
        {
            if (!float.IsFinite(f)) { return false; }
        }
        return true;
    }

    public void Dispose() => _pool?.Dispose();
}
