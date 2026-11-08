using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using Bar;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Indicative only: same scene, one thread, five independent samples, collection outside the clock.
string sceneName = args.Length > 0 ? args[0] : "large-pile";
string method = args.Length > 1 ? args[1] : "fork";
if (method is not "fork" and not "restore") { throw new ArgumentException("method must be fork or restore"); }
Console.WriteLine($"runtime={RuntimeInformation.FrameworkDescription}; os={RuntimeInformation.OSDescription}; runtimeCpuCap={Environment.ProcessorCount}; scene={sceneName}; method={method}; indicative");
Console.WriteLine("osLogicalCores=" + (File.Exists("/proc/cpuinfo") ? File.ReadLines("/proc/cpuinfo").Count(s => s.StartsWith("processor", StringComparison.Ordinal)).ToString(CultureInfo.InvariantCulture) : "unavailable"));
Console.WriteLine("cpu=" + (File.Exists("/proc/cpuinfo") ? File.ReadLines("/proc/cpuinfo").FirstOrDefault(s => s.StartsWith("model name", StringComparison.Ordinal)) : "unavailable"));
string Load() => File.Exists("/proc/loadavg") ? File.ReadAllText("/proc/loadavg").Trim() : "unavailable";
Console.WriteLine("loadBefore=" + Load());

Func<object> copy;
Func<ulong> sourceHash;
Func<object, ulong> hash;
Func<byte[]> snapshot;
Action step;
Action verify;
if (sceneName == "large-pile")
{
    var scene = Gen.All().Single(s => s.Name == sceneName);
    var world = SceneBuilder2D.Build(scene, 1).World;
    step = () => world.Step((float)scene.Dt);
    sourceHash = world.ComputeHash; snapshot = world.Snapshot;
    hash = o => ((P2.PhysicsWorld2D)o).ComputeHash();
    copy = method == "fork" ? () => world.Fork() : () =>
    {
        var bytes = world.Snapshot();
        var other = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = world.Id });
        other.Restore(bytes); return other;
    };
    verify = () =>
    {
        var f = world.Fork(); var r = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = world.Id }); r.Restore(world.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            step(); f.Step((float)scene.Dt); r.Step((float)scene.Dt);
            if (world.ComputeHash() != f.ComputeHash() || world.ComputeHash() != r.ComputeHash())
            { throw new InvalidOperationException("Dense 2D fork/restore trajectory differs at " + i.ToString(CultureInfo.InvariantCulture) + ": " + world.FindDifferences(f)); }
        }
        if (!world.Snapshot().AsSpan().SequenceEqual(f.Snapshot()) || !world.Snapshot().AsSpan().SequenceEqual(r.Snapshot())) { throw new InvalidOperationException("Dense 2D snapshot bytes differ."); }
        Console.WriteLine("verified 300 dense 2D steps: source = structural fork = restored world");
    };
}
else
{
    var scene = Gen3.All().Single(s => s.Name == sceneName);
    var world = SceneBuilder3D.Build(scene, 1).World;
    SceneBuilder3D.AddSofts(scene, world);
    step = () => world.Step((float)scene.Dt);
    sourceHash = world.ComputeHash; snapshot = world.Snapshot;
    hash = o => ((P3.PhysicsWorld3D)o).ComputeHash();
    copy = method == "fork" ? () => world.Fork() : () =>
    {
        var bytes = world.Snapshot();
        var other = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = world.Id });
        other.Restore(bytes); return other;
    };
    verify = () =>
    {
        var f = world.Fork(); var r = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = world.Id }); r.Restore(world.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            step(); f.Step((float)scene.Dt); r.Step((float)scene.Dt);
            if (world.ComputeHash() != f.ComputeHash() || world.ComputeHash() != r.ComputeHash())
            { throw new InvalidOperationException("Dense 3D fork/restore trajectory differs at " + i.ToString(CultureInfo.InvariantCulture) + ": " + world.FindDifferences(f)); }
        }
        if (!world.Snapshot().AsSpan().SequenceEqual(f.Snapshot()) || !world.Snapshot().AsSpan().SequenceEqual(r.Snapshot())) { throw new InvalidOperationException("Dense 3D snapshot bytes differ."); }
        Console.WriteLine("verified 300 dense 3D steps: source = structural fork = restored world");
    };
}
for (int i = 0; i < 300; i++) { step(); }
ulong expected = sourceHash();
for (int i = 0; i < 2; i++) { if (hash(copy()) != expected) { throw new InvalidOperationException("Warm copy hash differs."); } }
var times = new double[5]; var allocated = new long[5];
for (int i = 0; i < times.Length; i++)
{
    GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
    long bytesBefore = GC.GetAllocatedBytesForCurrentThread();
    long start = Stopwatch.GetTimestamp();
    object result = copy();
    times[i] = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
    allocated[i] = GC.GetAllocatedBytesForCurrentThread() - bytesBefore;
    if (hash(result) != expected || sourceHash() != expected) { throw new InvalidOperationException("Copy or source hash differs."); }
    GC.KeepAlive(result);
    Console.WriteLine(FormattableString.Invariant($"sample={i}; ms={times[i]:F4}; allocatedBytes={allocated[i]}; loadDuring={Load()}"));
}
Array.Sort(times); Array.Sort(allocated);
long liveBefore = GC.GetTotalMemory(true);
object live = copy();
long retained = GC.GetTotalMemory(true) - liveBefore;
GC.KeepAlive(live);
Console.WriteLine(FormattableString.Invariant($"medianMs={times[2]:F4}; bestMs={times[0]:F4}; medianAllocatedBytes={allocated[2]}; retainedHeapDeltaBytes={retained}; processPeakWorkingSetBytes={Process.GetCurrentProcess().PeakWorkingSet64}"));
Console.WriteLine("loadAfter=" + Load());
var a = snapshot(); step(); var b = snapshot();
var literal = SnapshotEncoding.CreateDelta(a, b); var xor = SnapshotEncoding.CreateXorDelta(a, b);
Console.WriteLine($"sizes full={b.Length}; delta={literal.Length}; rle={SnapshotEncoding.Compress(b).Length}; planesRle={SnapshotEncoding.Compress(b, true).Length}; compressedDelta={SnapshotEncoding.Compress(literal).Length}; compressedXorDelta={SnapshotEncoding.Compress(xor).Length}; planesCompressedXorDelta={SnapshotEncoding.Compress(xor, true).Length}");

if (args.Length > 2 && args[2] == "verify") { verify(); }
