using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Bake;

// What a world gives the recorder. One adapter per dimension; everything the recorder stores comes through here, so the
// recorder itself is the same code for both. Positions are doubles, rotations floats: a unit quaternion (x, y, z, w) in 3D
// and (cosine, sine) in 2D, in R0..R3.

internal struct PoseSample
{
    internal int Slot, Gen;
    internal double X, Y, Z;
    internal float R0, R1, R2, R3;
}

/// <summary>The particles of one soft body, cloth or rod this frame; positions interleaved by dimension.</summary>
internal sealed class PointSet
{
    internal int Slot, Gen, Kind, N;
    internal double[] P = System.Array.Empty<double>();
}

internal struct ParticleSample
{
    internal int Id, Fluid;
    internal double X, Y, Z;
}

internal struct EventSample
{
    internal byte Kind;
    internal int ASlot, AGen, BSlot, BGen;
    internal uint ValueBits;
}

internal sealed class GeometrySample
{
    internal int Slot, Gen;
    internal float[] Vertices = System.Array.Empty<float>(); // body frame, interleaved by dimension
    internal int[] Triangles = System.Array.Empty<int>();    // indices, three per triangle (3D only)
}

internal sealed class FractureSample
{
    internal byte Kind;
    internal int SourceSlot, SourceGen;
    internal float Impulse;
    internal double PX, PY, PZ;
    internal (int Slot, int Gen)[] Fragments = System.Array.Empty<(int, int)>();
    internal GeometrySample[] Geometry = System.Array.Empty<GeometrySample>();
}

internal interface IBakeSource
{
    int Dimension { get; }
    double MetresPerUnit { get; }
    long StepCount { get; }
    ulong Hash();
    void ReadBodies(List<PoseSample> into);
    void ReadSoft(List<PointSet> into);
    int[] SoftTriangles(int slot);
    void ReadRods(List<PointSet> into);
    void ReadParticles(List<ParticleSample> into);
    void ReadEvents(List<EventSample> into);
    void ReadFractures(List<FractureSample> into);
}
