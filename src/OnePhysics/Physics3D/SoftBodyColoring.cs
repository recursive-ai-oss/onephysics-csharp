using System;
using System.Numerics;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Constraint colouring of a soft body. Constraints of one colour share no particle, so they can be solved in any order, on any number of threads or in vector lanes, with the same result;
/// the solver runs the colours one after another (Gauss–Seidel between colours), as in parallel position-based solvers (Fratarcangeli, Tibaldo and Pellacini 2016, Vivace; Macklin et al. 2014).
/// The colouring is greedy over the constraints in index order, from the mesh alone, so it depends on neither the thread count nor the vector width.
/// </summary>
internal sealed partial class SoftBodyData
{
    /// <summary>Colours per constraint kind; a constraint that finds all of them taken goes to one extra last colour that is always solved by a single task in index order (a hub particle with hundreds of constraints).</summary>
    public const int MaxColours = 62;

    // Solver copies of the constraints, grouped by colour (ascending constraint index within a colour). The public raw arrays keep their documented order.
    public int[] SEdgeA = Array.Empty<int>(), SEdgeB = Array.Empty<int>();
    public float[] SEdgeRest = Array.Empty<float>();
    public int[] SBendIdx = Array.Empty<int>();       // 4 per constraint
    public float[] SBendK = Array.Empty<float>();      // 4 per constraint
    public float[] SBendS = Array.Empty<float>(), SBendM0 = Array.Empty<float>();
    public int[] STets = Array.Empty<int>();          // 4 per tetrahedron
    public float[] STetRest = Array.Empty<float>();

    /// <summary>First constraint of each colour plus the end, per kind (one entry when the kind is empty).</summary>
    public int[] EdgeColour = new int[1], BendColour = new int[1], TetColour = new int[1];
    public bool EdgeOverflow, BendOverflow, TetOverflow;

    public int EdgeColours => EdgeColour.Length - 1;
    public int BendColours => BendColour.Length - 1;
    public int TetColours => TetColour.Length - 1;
    /// <summary>Whether the pressure constraint is part of a pass: a closed surface with a positive internal pressure (the pressure can be changed through the handle between steps).</summary>
    public bool HasPressure => PressureRest > 0f && Def.InternalPressure > 0f;

    /// <summary>Number of solver phases of one pass: every colour of the edges, then of the bends, then of the tetrahedra, then the pressure constraint.</summary>
    public int Phases => EdgeColours + BendColours + TetColours + (HasPressure ? 1 : 0);

    /// <summary>Kind (0 edge, 1 bend, 2 tetrahedron, 3 pressure) and constraint range of a phase.</summary>
    public void Phase(int phase, out int kind, out int first, out int end, out bool serial)
    {
        serial = false;
        if (phase < EdgeColours) { kind = 0; first = EdgeColour[phase]; end = EdgeColour[phase + 1]; serial = EdgeOverflow && phase == EdgeColours - 1; return; }
        phase -= EdgeColours;
        if (phase < BendColours) { kind = 1; first = BendColour[phase]; end = BendColour[phase + 1]; serial = BendOverflow && phase == BendColours - 1; return; }
        phase -= BendColours;
        if (phase < TetColours) { kind = 2; first = TetColour[phase]; end = TetColour[phase + 1]; serial = TetOverflow && phase == TetColours - 1; return; }
        kind = 3; first = 0; end = 1; serial = true;
    }

    private void ColourConstraints()
    {
        int ne = EdgeA.Length, nb = BendS.Length, nt = Tets.Length / 4;
        var interleaved = new int[2 * ne];
        for (int i = 0; i < ne; i++) { interleaved[2 * i] = EdgeA[i]; interleaved[2 * i + 1] = EdgeB[i]; }
        Colour(ne, 2, interleaved, N, out var eo, out EdgeColour, out EdgeOverflow);
        SEdgeA = new int[ne]; SEdgeB = new int[ne]; SEdgeRest = new float[ne];
        for (int i = 0; i < ne; i++) { int s = eo[i]; SEdgeA[i] = EdgeA[s]; SEdgeB[i] = EdgeB[s]; SEdgeRest[i] = EdgeRest[s]; }

        Colour(nb, 4, BendIdx, N, out var bo, out BendColour, out BendOverflow);
        SBendIdx = new int[4 * nb]; SBendK = new float[4 * nb]; SBendS = new float[nb]; SBendM0 = new float[nb];
        for (int i = 0; i < nb; i++)
        {
            int s = bo[i];
            for (int k = 0; k < 4; k++) { SBendIdx[4 * i + k] = BendIdx[4 * s + k]; SBendK[4 * i + k] = BendK[4 * s + k]; }
            SBendS[i] = BendS[s]; SBendM0[i] = BendM0[s];
        }

        Colour(nt, 4, Tets, N, out var to, out TetColour, out TetOverflow);
        STets = new int[4 * nt]; STetRest = new float[nt];
        for (int i = 0; i < nt; i++)
        {
            int s = to[i];
            for (int k = 0; k < 4; k++) { STets[4 * i + k] = Tets[4 * s + k]; }
            STetRest[i] = TetRest[s];
        }
    }

    // Greedy colouring: the first colour not used by any particle of the constraint. Returns the constraint order grouped by colour (a stable counting sort) and the start of each colour.
    private static void Colour(int n, int m, int[] idx, int particles, out int[] order, out int[] start, out bool overflow)
    {
        var used = new ulong[particles];
        var colour = new byte[n];
        int[] count = new int[MaxColours + 1];
        for (int i = 0; i < n; i++)
        {
            ulong u = 0;
            for (int k = 0; k < m; k++) { u |= used[idx[m * i + k]]; }
            int c = BitOperations.TrailingZeroCount(~u);
            if (c >= MaxColours) { c = MaxColours; }
            else { for (int k = 0; k < m; k++) { used[idx[m * i + k]] |= 1UL << c; } }
            colour[i] = (byte)c;
            count[c]++;
        }

        // Colours 0.. are used contiguously (a colour is taken only when every lower one is blocked); the overflow set, when present, is the last.
        int colours = 0;
        while (colours < MaxColours && count[colours] > 0) { colours++; }
        overflow = count[MaxColours] > 0;
        int total = colours + (overflow ? 1 : 0);
        start = new int[Math.Max(total, 0) + 1];
        for (int c = 0; c < total; c++) { start[c + 1] = start[c] + count[c < colours ? c : MaxColours]; }
        order = new int[n];
        var fill = new int[total + 1];
        for (int i = 0; i < n; i++)
        {
            int c = colour[i] == MaxColours ? colours : colour[i];
            order[start[c] + fill[c]++] = i;
        }
    }
}
