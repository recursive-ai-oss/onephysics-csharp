using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz targets "rollback2d" and "rollback3d": the bytes are an operation stream (steps, mark, rollback, release, body creation, destruction, velocity and gravity changes) run on two worlds
// at once. One world uses Mark/RollbackTo; the other is the oracle and keeps a snapshot at every mark and restores it. The two must hash alike after every operation, and a rollback must give
// back the marked snapshot's bytes. Any exception is a finding: the operations are all valid by construction.
internal static class FuzzRollback
{
    private const float Dt = 1f / 60f;
    private static int s_marks, s_rollbacks;

    static FuzzRollback()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("rollback targets: marks " + s_marks + ", rollbacks " + s_rollbacks);
    }

    private sealed class Ops
    {
        private readonly byte[] _d;
        private int _i;
        public Ops(byte[] d) { _d = d; }
        public bool Done => _i >= _d.Length;
        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;
    }

    private static PhysicsWorld2D Build2D(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7, ThreadCount = threads });
        w.AddBox(new Vector2F(30, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 5; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D(0.3 * i, 0.5 + 1.05 * i)); }
        w.AddCircle(0.5f, at: new Vector2D(4, 2));
        for (int i = 0; i < 12; i++) { w.Step(Dt); }
        return w;
    }

    private static RecursiveAI.OnePhysics.Physics2D.Body Pick2D(PhysicsWorld2D w, int pick)
    {
        var all = new List<RecursiveAI.OnePhysics.Physics2D.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) { all.Add(body); } }
        return all.Count == 0 ? default : all[pick % all.Count];
    }

    public static void Run2D(byte[] data)
    {
        var r = new Ops(data);
        int threads = 1 + (r.Byte() & 1) * 3;
        var a = Build2D(threads);
        var b = Build2D(threads);
        var marks = new List<WorldMark>();
        var saved = new List<byte[]>();
        for (int op = 0; op < 48 && !r.Done; op++)
        {
            switch (r.Byte() % 9)
            {
                case 0:
                case 1:
                {
                    int n = 1 + r.Byte() % 8;
                    for (int k = 0; k < n; k++) { a.Step(Dt); b.Step(Dt); }
                    break;
                }
                case 2:
                    if (marks.Count < 6) { marks.Add(a.Mark()); saved.Add(b.Snapshot()); s_marks++; }
                    break;
                case 3:
                    if (marks.Count > 0)
                    {
                        int i = r.Byte() % marks.Count;
                        a.RollbackTo(marks[i]);
                        b.Restore(saved[i]);
                        s_rollbacks++;
                        marks.RemoveRange(i + 1, marks.Count - i - 1);
                        saved.RemoveRange(i + 1, saved.Count - i - 1);
                        if (!a.Snapshot().AsSpan().SequenceEqual(saved[i])) { throw new InvalidOperationException("2D rollback does not return to the marked snapshot bytes"); }
                    }
                    break;
                case 4:
                    if (marks.Count > 0)
                    {
                        int i = r.Byte() % marks.Count;
                        if (!a.ReleaseMark(marks[i])) { throw new InvalidOperationException("a live mark would not release"); }
                        marks.RemoveAt(i);
                        saved.RemoveAt(i);
                    }
                    break;
                case 5:
                {
                    double x = (r.Byte() - 128) / 16.0, y = 2 + r.Byte() / 16.0; float s = 0.3f + (r.Byte() % 8) * 0.1f;
                    a.AddBox(new Vector2F(s, s), at: new Vector2D(x, y)); b.AddBox(new Vector2F(s, s), at: new Vector2D(x, y));
                    break;
                }
                case 6:
                {
                    int pick = r.Byte(); var v = new Vector2F((r.Byte() - 128) / 8f, (r.Byte() - 128) / 8f);
                    var x = Pick2D(a, pick); var y = Pick2D(b, pick);
                    if (x.IsValid) { x.LinearVelocity = v; }
                    if (y.IsValid) { y.LinearVelocity = v; }
                    break;
                }
                case 7:
                {
                    int pick = r.Byte();
                    var x = Pick2D(a, pick); var y = Pick2D(b, pick);
                    if (x.IsValid) { x.Destroy(); }
                    if (y.IsValid) { y.Destroy(); }
                    break;
                }
                default:
                {
                    float g = -9.81f + (r.Byte() - 128) / 64f; a.Options.Gravity = new Vector2F(0, g); b.Options.Gravity = new Vector2F(0, g);
                    break;
                }
            }

            if (a.ComputeHash() != b.ComputeHash()) { throw new InvalidOperationException("2D rollback and restore diverged after operation " + op.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        }

        if (!a.Snapshot().AsSpan().SequenceEqual(b.Snapshot())) { throw new InvalidOperationException("2D snapshot bytes differ from the snapshot-and-restore oracle"); }
    }

    private static PhysicsWorld3D Build3D(int threads)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7, ThreadCount = threads });
        w.AddBox(new Vector3F(30, 1, 30), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 5; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0.3 * i, 0.5 + 1.05 * i, 0)); }
        w.AddSphere(0.5f, at: new Vector3D(4, 2, 0));
        for (int i = 0; i < 12; i++) { w.Step(Dt); }
        return w;
    }

    private static RecursiveAI.OnePhysics.Physics3D.Body Pick3D(PhysicsWorld3D w, int pick)
    {
        var all = new List<RecursiveAI.OnePhysics.Physics3D.Body>();
        foreach (var body in w.Bodies) { if (body.Type == BodyType.Dynamic) { all.Add(body); } }
        return all.Count == 0 ? default : all[pick % all.Count];
    }

    public static void Run3D(byte[] data)
    {
        var r = new Ops(data);
        int threads = 1 + (r.Byte() & 1) * 3;
        var a = Build3D(threads);
        var b = Build3D(threads);
        var marks = new List<WorldMark>();
        var saved = new List<byte[]>();
        for (int op = 0; op < 48 && !r.Done; op++)
        {
            switch (r.Byte() % 9)
            {
                case 0:
                case 1:
                {
                    int n = 1 + r.Byte() % 8;
                    for (int k = 0; k < n; k++) { a.Step(Dt); b.Step(Dt); }
                    break;
                }
                case 2:
                    if (marks.Count < 6) { marks.Add(a.Mark()); saved.Add(b.Snapshot()); s_marks++; }
                    break;
                case 3:
                    if (marks.Count > 0)
                    {
                        int i = r.Byte() % marks.Count;
                        a.RollbackTo(marks[i]);
                        b.Restore(saved[i]);
                        s_rollbacks++;
                        marks.RemoveRange(i + 1, marks.Count - i - 1);
                        saved.RemoveRange(i + 1, saved.Count - i - 1);
                        if (!a.Snapshot().AsSpan().SequenceEqual(saved[i])) { throw new InvalidOperationException("3D rollback does not return to the marked snapshot bytes"); }
                    }
                    break;
                case 4:
                    if (marks.Count > 0)
                    {
                        int i = r.Byte() % marks.Count;
                        if (!a.ReleaseMark(marks[i])) { throw new InvalidOperationException("a live mark would not release"); }
                        marks.RemoveAt(i);
                        saved.RemoveAt(i);
                    }
                    break;
                case 5:
                {
                    double x = (r.Byte() - 128) / 16.0, y = 2 + r.Byte() / 16.0, z = (r.Byte() - 128) / 16.0; float s = 0.3f + (r.Byte() % 8) * 0.1f;
                    a.AddBox(new Vector3F(s, s, s), at: new Vector3D(x, y, z)); b.AddBox(new Vector3F(s, s, s), at: new Vector3D(x, y, z));
                    break;
                }
                case 6:
                {
                    int pick = r.Byte(); var v = new Vector3F((r.Byte() - 128) / 8f, (r.Byte() - 128) / 8f, (r.Byte() - 128) / 8f);
                    var x = Pick3D(a, pick); var y = Pick3D(b, pick);
                    if (x.IsValid) { x.LinearVelocity = v; }
                    if (y.IsValid) { y.LinearVelocity = v; }
                    break;
                }
                case 7:
                {
                    int pick = r.Byte();
                    var x = Pick3D(a, pick); var y = Pick3D(b, pick);
                    if (x.IsValid) { x.Destroy(); }
                    if (y.IsValid) { y.Destroy(); }
                    break;
                }
                default:
                {
                    float g = -9.81f + (r.Byte() - 128) / 64f; a.Options.Gravity = new Vector3F(0, g, 0); b.Options.Gravity = new Vector3F(0, g, 0);
                    break;
                }
            }

            if (a.ComputeHash() != b.ComputeHash()) { throw new InvalidOperationException("3D rollback and restore diverged after operation " + op.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        }

        if (!a.Snapshot().AsSpan().SequenceEqual(b.Snapshot())) { throw new InvalidOperationException("3D snapshot bytes differ from the snapshot-and-restore oracle"); }
    }
}
