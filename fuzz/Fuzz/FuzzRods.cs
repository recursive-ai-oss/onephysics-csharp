using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Rods;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzRods
{
    internal static void Definition(byte[] input)
    {
        if (input.Length < 4) return;
        int n = 2+input[0]%30;
        var points = new Vector3D[n];
        for (int i = 0; i < n; i++) points[i] = new Vector3D(i*0.1,0,0);
        if (input.Length >= 16) points[input[1]%n] = new Vector3D(BitConverter.ToSingle(input,4),BitConverter.ToSingle(input,8),BitConverter.ToSingle(input,12));
        try
        {
            var d = new P3.RodDef { Points=points, PinnedParticles=[input[2]], Substeps=input[3] };
            if (input.Length >= 32) d.Material = new RodMaterial(BitConverter.ToSingle(input,16),BitConverter.ToSingle(input,20),BitConverter.ToSingle(input,24),BitConverter.ToSingle(input,28));
            d.Validate();
            var w = new P3.PhysicsWorld3D(); w.AddRod(d); var bytes=w.Snapshot(); w.Restore(bytes);
            if (!bytes.AsSpan().SequenceEqual(w.Snapshot())) throw new InvalidOperationException("Rod construction round trip changed bytes.");
        }
        catch (ArgumentException) { }
    }
    // An accepted snapshot must step to completion or fail with a typed error before changing anything.
    private static void StepChecked(Action<float> step,Func<ulong> hash)
    {
        for (int i=0;i<3;i++)
        {
            var before=hash();
            try { step(1f/60); }
            catch (Exception e) when (e is PhysicsException or ArgumentOutOfRangeException)
            { if (before!=hash()) throw new InvalidOperationException("A rod step failed after changing the world: "+e.Message); return; }
        }
    }
    private static byte[]? s_planar, s_spatial;
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            if (s_planar != null) return s_planar;
            var w=new P2.PhysicsWorld2D(new() { WorldId=36 }); var disc=w.AddCircle(.1f,at:new(1,0)); var line=w.AddRope(default,new(1,0),4); line.Attach(4,disc,new(.05f,0),breakForce:1e6f); w.Step(1f/60); return s_planar=w.Snapshot();
        }
        if (s_spatial != null) return s_spatial;
        var world=new P3.PhysicsWorld3D(new() { WorldId=36 }); var ball=world.AddSphere(.1f,at:new(1,0,0)); var rope=world.AddRope(default,new(1,0,0),4); rope.Attach(4,ball,new(.05f,0,0),breakForce:1e6f); world.Step(1f/60); return s_spatial=world.Snapshot();
    }
    internal static void Snapshot(byte[] input)
    {
        bool planar=input.Length == 0 || (input[0]&1) == 0;
        var bytes=(byte[])Baseline(planar).Clone();
        for (int i=1; i+2<input.Length; i+=3)
        {
            int offset=bytes.Length-1-(((input[i]<<8)|input[i+1])%Math.Min(1200,bytes.Length)); bytes[offset]^=input[i+2];
        }
        if (input.Length>0 && (input[0]&15)==15) Array.Resize(ref bytes,input[^1]*bytes.Length/256);
        if (planar)
        {
            var world=new P2.PhysicsWorld2D(new() { WorldId=36 }); var before=world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected rod snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted rod snapshot changed bytes.");
            StepChecked(world.Step,world.ComputeHash);
        }
        else
        {
            var world=new P3.PhysicsWorld3D(new() { WorldId=36 }); var before=world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!before.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Rejected rod snapshot changed world."); return; }
            if (!bytes.AsSpan().SequenceEqual(world.Snapshot())) throw new InvalidOperationException("Accepted rod snapshot changed bytes.");
            StepChecked(world.Step,world.ComputeHash);
        }
    }
}
