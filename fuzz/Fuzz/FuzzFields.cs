using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

internal static class FuzzFields
{
    private static byte[]? s_base;
    private static byte[] Baseline()
    {
        if(s_base!=null)return s_base;
        var world=new PhysicsWorld2D(new(){WorldId=35});var b=world.AddBox(new(2,1),density:500);b.MagneticMoment=new(1,0);b.BuoyancyFactor=0.5f;
        world.AddWater(0);world.AddField(new(){Kind=FieldKind.Point,Strength=-0.01f});world.Step(1f/60);return s_base=world.Snapshot();
    }
    internal static void Snapshot(byte[] input)
    {
        var bytes=(byte[])Baseline().Clone();
        // Field state is the trailing optional section: bias flips toward its new reader.
        for(int i=1;i+2<input.Length;i+=3)
        {
            int span=Math.Min(700,bytes.Length);int offset=bytes.Length-1-(((input[i]<<8)|input[i+1])%span);bytes[offset]^=input[i+2];
        }
        if(input.Length>0&&(input[0]&15)==15)Array.Resize(ref bytes,input[^1]*bytes.Length/256);
        var world=new PhysicsWorld2D(new(){WorldId=35});byte[] before=world.Snapshot();
        try {world.Restore(bytes);}
        catch(SnapshotFormatException)
        {
            if(!world.Snapshot().AsSpan().SequenceEqual(before))throw new InvalidOperationException("Rejected field snapshot changed world.");return;
        }
        if(!world.Snapshot().AsSpan().SequenceEqual(bytes))throw new InvalidOperationException("Accepted field snapshot changed bytes.");
    }
}
