using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>
/// The coded history of one thing (a body, a soft body, a rod, a particle). Frames are coded in blocks of at most
/// BlockFrames; inside a block each frame is predicted from the two before it (the first frame from zero, the second from
/// the first) and only the integer residual is stored, as zigzag varints. A run of frames whose residual is all zero costs
/// two bytes however long it is. See docs/spec/bake.md.
/// </summary>
internal sealed class BakeEntityEncoder
{
    internal readonly int Key, Gen, Kind, N;
    internal readonly int FirstFrame;
    internal int Frames;
    internal int LastSeen;
    internal int[]? Triangles;
    internal float[]? PrevRot; // last stored rotation (floats), for the quaternion sign rule

    private readonly int _len;      // integers per frame
    private readonly int _split;    // the first group is _split long; the rest is the second group (0 = one group)
    private readonly int _blockFrames;
    private readonly long[] _prev1, _prev2, _res;
    private readonly ByteSink _block = new(64);
    private int _inBlock;
    private int _zeroRun;
    internal readonly List<byte[]> Blocks = new();

    internal BakeEntityEncoder(int key, int gen, int kind, int n, int firstFrame, int length, int split, int blockFrames)
    {
        Key = key; Gen = gen; Kind = kind; N = n; FirstFrame = firstFrame; LastSeen = firstFrame;
        _len = length; _split = split; _blockFrames = blockFrames;
        _prev1 = new long[length]; _prev2 = new long[length]; _res = new long[length];
    }

    internal void Append(ReadOnlySpan<long> q)
    {
        int mask = 0;
        for (int i = 0; i < _len; i++)
        {
            long pred = _inBlock == 0 ? 0 : _inBlock == 1 ? _prev1[i] : unchecked(2 * _prev1[i] - _prev2[i]);
            long r = unchecked(q[i] - pred);
            _res[i] = r;
            if (r != 0) { mask |= (_split == 0 || i < _split) ? 1 : 2; }
        }

        if (mask == 0) { _zeroRun++; }
        else
        {
            FlushRun();
            _block.U8(mask);
            for (int i = 0; i < _len; i++)
            {
                bool inFirst = _split == 0 || i < _split;
                if (((mask & (inFirst ? 1 : 2)) != 0)) { _block.Var(BakeFormat.ZigZag(_res[i])); }
            }
        }

        Array.Copy(_prev1, _prev2, _len);
        q.CopyTo(_prev1);
        Frames++; _inBlock++;
        if (_inBlock == _blockFrames) { CloseBlock(); }
    }

    private void FlushRun()
    {
        if (_zeroRun == 0) { return; }
        _block.U8(0);
        _block.Var((ulong)_zeroRun);
        _zeroRun = 0;
    }

    private void CloseBlock()
    {
        FlushRun();
        Blocks.Add(_block.ToArray());
        _block.Clear();
        _inBlock = 0;
    }

    internal void Finish() { if (_inBlock > 0) { CloseBlock(); } }
}
