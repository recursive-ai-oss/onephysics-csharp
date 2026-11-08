using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 98 });
world.AddBox(new Vector2F(20, 1), new Vector2D(0, -0.5), type: BodyType.Static);
world.AddCircle(0.5f, new Vector2D(0, 2));
for (int i = 0; i < 30; i++) { world.Step(1f / 60); }
var fork = world.Fork();
var region = world.CaptureRegion(new Vector2D(-1, -1), new Vector2D(1, 3));
var regionTarget = fork.Fork();
regionTarget.RestoreRegion(region.ToArray());
if (region.ComputeHash() != regionTarget.ComputeRegionHash(new Vector2D(-1, -1), new Vector2D(1, 3))) { throw new InvalidOperationException("Regional values differ."); }
if (world.FindDifferences(fork).Count != 0) { throw new InvalidOperationException("Structural copy differs."); }
var raw = SnapshotDesync.Inspect(world.Core);
if (raw.Count == 0) { throw new InvalidOperationException("No described fields."); }
var snapshot = SnapshotEncoding.Decompress(world.SnapshotCompressed(true));
var target = new PhysicsWorld2D(new WorldOptions2D { WorldId = 98 }); target.Restore(snapshot);
for (int i = 0; i < 300; i++)
{
    world.Step(1f / 60); fork.Step(1f / 60); target.Step(1f / 60);
    if (world.ComputeHash() != fork.ComputeHash() || world.ComputeHash() != target.ComputeHash()) { throw new InvalidOperationException("Fork/restore differs."); }
}
Console.WriteLine("OK 2D snapshot forks, regions, fields and compression");
