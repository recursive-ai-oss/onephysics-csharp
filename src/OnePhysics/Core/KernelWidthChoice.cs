using System.Runtime.Intrinsics;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Which lane width the wide solver kernels run at by default. Batches are always eight lanes and every lane computes the scalar twin's operations in the same order,
/// so the width reaches no result: it only decides how fast a batch runs. Eight lanes want a 256-bit vector; where that is not a hardware type (arm64 and browser
/// WebAssembly have 128-bit vectors) a 256-bit vector is emulated with several instructions per operation, so those platforms run the same batch as two four-lane halves.
/// </summary>
internal static class KernelWidthChoice
{
    /// <summary>8 where <see cref="Vector256.IsHardwareAccelerated"/> is true, otherwise 4.</summary>
    public static int Default => Vector256.IsHardwareAccelerated ? 8 : 4;
}
