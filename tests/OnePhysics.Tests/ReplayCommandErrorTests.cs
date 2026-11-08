using System;
using System.IO;
using System.Buffers.Binary;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class ReplayCommandErrorTests
{
    [Fact]
    public void MutatedCommandHeadersAndNamesAreRejectedWithInputOffsets()
    {
        byte[] data=FuzzReplay.CommandSeed(2);long offset;
        using(var input=new MemoryStream(data))
        {
            var r=new ReplayReader(input);while(r.ReadNext() && r.Kind!=ReplayRecordKind.Mutation){}
            Assert.Equal(ReplayRecordKind.Mutation,r.Kind);offset=r.RecordOffset;
        }
        foreach(int at in new[]{(int)offset+5+7,(int)offset+5+8,(int)offset+5+10})
        {
            byte[] bytes=(byte[])data.Clone();bytes[at]=255;
            using var stream=new MemoryStream(bytes);var reader=new ReplayReader(stream,"commands");
            var e=Assert.Throws<ReplayFormatException>(()=>{while(reader.ReadNext()){} });Assert.Equal("commands",e.InputName);
        }
    }
    [Fact]
    public void InvalidLiveOptionsAreTypedDuringAutomaticConstruction()
    {
        byte[] data=FuzzReplay.CommandSeed(2);
        using(var input=new MemoryStream(data))
        {
            var r=new ReplayReader(input);Assert.True(r.ReadNext());Assert.True(r.ReadNext());Assert.Equal(ReplayRecordKind.LiveOptions,r.Kind);
            BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan((int)r.RecordOffset+5+12),float.NaN);
        }
        using var stream=new MemoryStream(data);var player=new ReplayPlayer(stream,"options");
        var e=Assert.Throws<ReplayFormatException>(()=>player.CreateWorld2D());Assert.Equal("options",e.InputName);
    }
    [Fact]
    public void AutomaticConstructionTreatsCapacityValuesAsHintsNotAllocationCommands()
    {
        var world=new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D{WorldId=105});
        world.AddSphere(0.5f,at:new(0,3,0));world.Options.ContactCapacity=1<<24;
        using var stream=new MemoryStream();using(var recorder=world.StartRecording(stream))world.Step(1f/60);
        stream.Position=0;var player=new ReplayPlayer(stream);var copy=player.CreateWorld3D();
        Assert.Equal(1<<24,copy.Options.ContactCapacity);Assert.True(player.ReadNextStep(copy));Assert.Equal(world.ComputeHash(),copy.ComputeHash());
    }

    [Fact]
    public void IncompleteAndNestedCommandFramesCannotBeWritten()
    {
        using var stream=new MemoryStream();var w=new ReplayWriter(stream,2,0,new byte[]{1},commandTrack:true);
        Assert.Throws<InvalidOperationException>(()=>w.EndStep(0,0));
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.BeginStep(float.NaN));
        w.BeginStep(1f/60);
        Assert.Throws<InvalidOperationException>(()=>w.BeginStep(1f/60));
        Assert.Throws<InvalidOperationException>(()=>w.Complete());
        Assert.Throws<InvalidOperationException>(()=>w.WriteStep(1f/60));
        Assert.Throws<InvalidOperationException>(()=>w.WriteSnapshot(new byte[]{1}));
        w.EndStep(1,0);w.Complete();
    }
}
