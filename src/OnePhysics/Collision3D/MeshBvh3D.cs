using System;
using System.Collections.Generic;
using System.Buffers.Binary;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>A node of the mesh BVH: bounds, and either a leaf range or the right child (the left child is always the next node).</summary>
internal struct BvhNode
{
    public float MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

    /// <summary>For a leaf, the first position in the triangle order; for an inner node, the index of the right child.</summary>
    public int Offset;

    /// <summary>The number of triangles of a leaf; 0 for an inner node.</summary>
    public int Count;
}

/// <summary>
/// A static bounding volume hierarchy over triangle bounds, built top-down with binned surface-area-heuristic splits (Wald 2007, "On fast construction of SAH-based bounding volume hierarchies";
/// Ericson, Real-Time Collision Detection, ch. 6). The result depends only on the input: bins and sweeps are fixed, ties go to the lowest axis and bin, partitions are in place with a fixed rule,
/// and the fallback median split orders by (centroid, triangle index). Nodes are laid out depth first with the left child directly after its parent.
/// </summary>
internal sealed class MeshBvh3D
{
    internal const int Bins = 16;
    internal const int MaxLeaf = 4;
    internal const int MaxSahDepth = 32;
    private const double TraversalCost = 1.0, TriangleCost = 1.0;

    internal readonly BvhNode[] Nodes;
    internal readonly int[] Order;

    private MeshBvh3D(BvhNode[] nodes, int[] order)
    {
        Nodes = nodes;
        Order = order;
    }

    internal int NodeCount => Nodes.Length;

    private static double Area(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        double x = x1 - x0, y = y1 - y0, z = z1 - z0;
        return 2.0 * (((x * y) + (y * z)) + (z * x));
    }

    private sealed class CentroidComparer : IComparer<int>
    {
        private readonly float[] _b;
        private readonly int _axis;

        public CentroidComparer(float[] bounds, int axis)
        {
            _b = bounds;
            _axis = axis;
        }

        public int Compare(int x, int y)
        {
            double cx = (double)_b[(6 * x) + _axis] + _b[(6 * x) + 3 + _axis];
            double cy = (double)_b[(6 * y) + _axis] + _b[(6 * y) + 3 + _axis];
            return cx != cy ? cx.CompareTo(cy) : x.CompareTo(y);
        }
    }

    private struct Frame
    {
        public int Start, Count, Parent, Depth;
    }

    /// <summary>Builds the tree over <paramref name="count"/> triangles whose bounds are <c>b[6 * i .. 6 * i + 5]</c> (min xyz, max xyz).</summary>
    internal static MeshBvh3D Build(float[] b, int count)
    {
        var order = new int[count];
        for (int i = 0; i < count; i++)
        {
            order[i] = i;
        }

        var nodes = new BvhNode[(2 * count) - 1];
        int nodeCount = 0;
        var stack = new Frame[160];
        int sp = 0;
        stack[sp++] = new Frame { Start = 0, Count = count, Parent = -1, Depth = 0 };
        Span<int> binCount = stackalloc int[Bins];
        Span<float> binBox = stackalloc float[Bins * 6];
        Span<double> rightArea = stackalloc double[Bins];
        Span<int> rightCount = stackalloc int[Bins];
        Span<double> cmin = stackalloc double[3];
        Span<double> cmax = stackalloc double[3];
        while (sp > 0)
        {
            Frame f = stack[--sp];
            int ni = nodeCount++;
            if (f.Parent >= 0)
            {
                nodes[f.Parent].Offset = ni;
            }

            int end = f.Start + f.Count;
            float x0 = float.PositiveInfinity, y0 = x0, z0 = x0, x1 = float.NegativeInfinity, y1 = x1, z1 = x1;
            double cx0 = double.PositiveInfinity, cy0 = cx0, cz0 = cx0, cx1 = double.NegativeInfinity, cy1 = cx1, cz1 = cx1;
            for (int k = f.Start; k < end; k++)
            {
                int t = order[k] * 6;
                x0 = Math.Min(x0, b[t]);
                y0 = Math.Min(y0, b[t + 1]);
                z0 = Math.Min(z0, b[t + 2]);
                x1 = Math.Max(x1, b[t + 3]);
                y1 = Math.Max(y1, b[t + 4]);
                z1 = Math.Max(z1, b[t + 5]);
                double cx = (double)b[t] + b[t + 3], cy = (double)b[t + 1] + b[t + 4], cz = (double)b[t + 2] + b[t + 5];
                cx0 = Math.Min(cx0, cx);
                cy0 = Math.Min(cy0, cy);
                cz0 = Math.Min(cz0, cz);
                cx1 = Math.Max(cx1, cx);
                cy1 = Math.Max(cy1, cy);
                cz1 = Math.Max(cz1, cz);
            }

            nodes[ni] = new BvhNode { MinX = x0, MinY = y0, MinZ = z0, MaxX = x1, MaxY = y1, MaxZ = z1, Offset = f.Start, Count = f.Count };
            if (f.Count == 1)
            {
                continue;
            }

            // Centroid sums are twice the centroids; only ratios matter.
            cmin[0] = cx0;
            cmin[1] = cy0;
            cmin[2] = cz0;
            cmax[0] = cx1;
            cmax[1] = cy1;
            cmax[2] = cz1;
            int bestAxis = -1, bestBin = 0;
            double bestCost = double.PositiveInfinity;
            if (f.Depth < MaxSahDepth)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    double ext = cmax[axis] - cmin[axis];
                    double k = Bins / ext;
                    if (!(ext > 0.0) || !double.IsFinite(k))
                    {
                        continue;
                    }

                    binCount.Clear();
                    for (int i = 0; i < Bins; i++)
                    {
                        binBox[(6 * i)] = float.PositiveInfinity;
                        binBox[(6 * i) + 1] = float.PositiveInfinity;
                        binBox[(6 * i) + 2] = float.PositiveInfinity;
                        binBox[(6 * i) + 3] = float.NegativeInfinity;
                        binBox[(6 * i) + 4] = float.NegativeInfinity;
                        binBox[(6 * i) + 5] = float.NegativeInfinity;
                    }

                    for (int q = f.Start; q < end; q++)
                    {
                        int t = order[q] * 6;
                        int bin = BinOf(b, t, axis, cmin[axis], k);
                        binCount[bin]++;
                        for (int a = 0; a < 3; a++)
                        {
                            binBox[(6 * bin) + a] = Math.Min(binBox[(6 * bin) + a], b[t + a]);
                            binBox[(6 * bin) + 3 + a] = Math.Max(binBox[(6 * bin) + 3 + a], b[t + 3 + a]);
                        }
                    }

                    // Suffix sweep: boxes of bins s+1..Bins-1.
                    float rx0 = float.PositiveInfinity, ry0 = rx0, rz0 = rx0, rx1 = float.NegativeInfinity, ry1 = rx1, rz1 = rx1;
                    int rc = 0;
                    for (int s = Bins - 1; s >= 1; s--)
                    {
                        if (binCount[s] > 0)
                        {
                            rx0 = Math.Min(rx0, binBox[6 * s]);
                            ry0 = Math.Min(ry0, binBox[(6 * s) + 1]);
                            rz0 = Math.Min(rz0, binBox[(6 * s) + 2]);
                            rx1 = Math.Max(rx1, binBox[(6 * s) + 3]);
                            ry1 = Math.Max(ry1, binBox[(6 * s) + 4]);
                            rz1 = Math.Max(rz1, binBox[(6 * s) + 5]);
                            rc += binCount[s];
                        }

                        rightCount[s - 1] = rc;
                        rightArea[s - 1] = rc > 0 ? Area(rx0, ry0, rz0, rx1, ry1, rz1) : 0.0;
                    }

                    float lx0 = float.PositiveInfinity, ly0 = lx0, lz0 = lx0, lx1 = float.NegativeInfinity, ly1 = lx1, lz1 = lx1;
                    int lc = 0;
                    for (int s = 0; s < Bins - 1; s++)
                    {
                        if (binCount[s] > 0)
                        {
                            lx0 = Math.Min(lx0, binBox[6 * s]);
                            ly0 = Math.Min(ly0, binBox[(6 * s) + 1]);
                            lz0 = Math.Min(lz0, binBox[(6 * s) + 2]);
                            lx1 = Math.Max(lx1, binBox[(6 * s) + 3]);
                            ly1 = Math.Max(ly1, binBox[(6 * s) + 4]);
                            lz1 = Math.Max(lz1, binBox[(6 * s) + 5]);
                            lc += binCount[s];
                        }

                        if (lc == 0 || rightCount[s] == 0)
                        {
                            continue;
                        }

                        double cost = (Area(lx0, ly0, lz0, lx1, ly1, lz1) * lc) + (rightArea[s] * rightCount[s]);
                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestAxis = axis;
                            bestBin = s;
                        }
                    }
                }
            }

            int mid;
            if (bestAxis >= 0)
            {
                double parentArea = Area(x0, y0, z0, x1, y1, z1);
                double splitCost = TraversalCost + (parentArea > 0.0 ? (bestCost / parentArea) * TriangleCost : f.Count * TriangleCost);
                if (f.Count <= MaxLeaf && splitCost >= f.Count * TriangleCost)
                {
                    continue; // a leaf is cheaper
                }

                double kk = Bins / (cmax[bestAxis] - cmin[bestAxis]);
                int i2 = f.Start, j2 = end - 1;
                while (i2 <= j2)
                {
                    if (BinOf(b, order[i2] * 6, bestAxis, cmin[bestAxis], kk) <= bestBin)
                    {
                        i2++;
                    }
                    else
                    {
                        (order[i2], order[j2]) = (order[j2], order[i2]);
                        j2--;
                    }
                }

                mid = i2;
            }
            else
            {
                if (f.Count <= MaxLeaf)
                {
                    continue;
                }

                // All centroids coincide, or the depth limit is reached: split the range in half, by centroid order on the widest axis when there is one.
                int axis = 0;
                double widest = cmax[0] - cmin[0];
                if (cmax[1] - cmin[1] > widest)
                {
                    axis = 1;
                    widest = cmax[1] - cmin[1];
                }

                if (cmax[2] - cmin[2] > widest)
                {
                    axis = 2;
                    widest = cmax[2] - cmin[2];
                }

                if (widest > 0.0)
                {
                    Array.Sort(order, f.Start, f.Count, new CentroidComparer(b, axis));
                }

                mid = f.Start + (f.Count / 2);
            }

            if (mid <= f.Start || mid >= end)
            {
                mid = f.Start + (f.Count / 2);
            }

            nodes[ni].Count = 0;
            nodes[ni].Offset = 0;
            stack[sp++] = new Frame { Start = mid, Count = end - mid, Parent = ni, Depth = f.Depth + 1 };
            stack[sp++] = new Frame { Start = f.Start, Count = mid - f.Start, Parent = -1, Depth = f.Depth + 1 };
        }

        Array.Resize(ref nodes, nodeCount);
        return new MeshBvh3D(nodes, order);
    }

    private static int BinOf(float[] b, int t, int axis, double cmin, double k)
    {
        double c = (double)b[t + axis] + b[t + 3 + axis];
        int bin = (int)((c - cmin) * k);
        return bin < 0 ? 0 : bin >= Bins ? Bins - 1 : bin;
    }

    /// <summary>The canonical byte form: node count, then each node (six float bit patterns, offset, count), then the triangle order; all little-endian.</summary>
    internal byte[] ToBytes()
    {
        var bytes = new byte[4 + (Nodes.Length * 32) + (Order.Length * 4) + 4];
        Span<byte> s = bytes;
        BinaryPrimitives.WriteInt32LittleEndian(s, Nodes.Length);
        int p = 4;
        foreach (BvhNode n in Nodes)
        {
            BinaryPrimitives.WriteInt32LittleEndian(s[p..], BitConverter.SingleToInt32Bits(n.MinX));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 4)..], BitConverter.SingleToInt32Bits(n.MinY));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 8)..], BitConverter.SingleToInt32Bits(n.MinZ));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 12)..], BitConverter.SingleToInt32Bits(n.MaxX));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 16)..], BitConverter.SingleToInt32Bits(n.MaxY));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 20)..], BitConverter.SingleToInt32Bits(n.MaxZ));
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 24)..], n.Offset);
            BinaryPrimitives.WriteInt32LittleEndian(s[(p + 28)..], n.Count);
            p += 32;
        }

        BinaryPrimitives.WriteInt32LittleEndian(s[p..], Order.Length);
        p += 4;
        foreach (int o in Order)
        {
            BinaryPrimitives.WriteInt32LittleEndian(s[p..], o);
            p += 4;
        }

        return bytes;
    }
}
