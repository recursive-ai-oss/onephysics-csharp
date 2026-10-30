using System.Diagnostics;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>This library's 3D world behind the shared 3D driver loop.</summary>
sealed class OnePhysics3Sim : ISim3
{
    P3.PhysicsWorld3D world = null!;
    P3.Body[] bodies = [];
    int threads;
    List<(P3.Character Character, CharacterScript Script, int Body)> characters = [];
    List<(P3.Vehicle Vehicle, VehicleScript Script, int Body)> vehicles = [];
    List<(P3.Joint Joint, JointDef Def)> drives = [];
    float[] vy = [];
    float gravity;
    int stepNo;

    public string Name => "onephysics";
    public string Version => "0.1.0-dev";
    public string Config => $"threads={threads}, substeps=4, default options";

    public void Build(Scene s, int threads)
    {
        this.threads = threads;
        (world, bodies, _) = SceneBuilder3D.Build(s, threads, out characters, out vehicles, out drives);
        softs = SceneBuilder3D.AddSofts(s, world);
        skinDefs = s.Softs.Select(f => f.Kind == "sleeve" ? f : null).ToArray();
        vy = new float[characters.Count]; gravity = (float)-s.Gy; stepNo = 0;
    }

    /// <summary>Live contact constraints, their manifold points and joints at the end of the run.</summary>
    public string Extra()
    {
        var cs = world.Engine.Core.Constraints;
        int contacts = 0, points = 0, joints = 0;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c)) continue;
            if (cs.KindOf(c) == ConstraintKind.Joint) joints++; else { contacts++; points += cs.PointCount[c]; }
        }
        return $"constraints_contacts={contacts}\nconstraints_points={points}\nconstraints_joints={joints}\n" + CountLines();
    }

    public void Step(double dt)
    {
        MoveCharacters((float)dt);
        DriveVehicles();
        DriveJoints();
        for (int i = 0; i < softs.Length; i++)
        {
            if (!softs[i].HasSkin) continue;
            var d = skinDefs[i]!;
            softs[i].SetSkeletonTransforms(SleeveRig.Pose(stepNo, new Vector3D(d.X, d.Y, d.Z), d.P[1]));
        }
        world.Step((float)dt);
        if (StageProfile.Requested && stepNo >= 60) Count();
        stepNo++;
    }

    // Work counts per step for the stage report (OP_PROFILE): awake bodies, live pair-table entries, pairs with an awake body (the ones the narrowphase revisits), contact constraints with points, their points and joints.
    readonly List<int[]> counts = [];

    void Count()
    {
        var e = world.Engine; var core = e.Core; var cs = core.Constraints; var owner = core.Shapes.Owner; var bodies = core.Bodies;
        int live = 0, awakePairs = 0;
        for (int p = 0; p < e.PairHigh; p++)
        {
            if (!e.PairLive[p]) continue;
            live++;
            if (bodies.IsAwake(owner[e.PairA[p]]) || bodies.IsAwake(owner[e.PairB[p]])) awakePairs++;
        }
        int contacts = 0, points = 0, joints = 0;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c)) continue;
            if (cs.KindOf(c) == ConstraintKind.Joint) joints++; else if (cs.PointCount[c] > 0) { contacts++; points += cs.PointCount[c]; }
        }
        counts.Add([core.AwakeBodies.Length, live, awakePairs, contacts, points, joints, e.StatNewPairs, e.StatUpdated, e.StatReached, e.StatFresh]);
    }

    string CountLines()
    {
        if (counts.Count == 0) return "";
        string[] names = ["awake_bodies", "pairs_live", "pairs_awake", "manifolds", "points", "joints", "new_pairs", "pairs_revisited", "pairs_tight_pass", "pairs_narrowphase"];
        var sb = new StringBuilder();
        for (int k = 0; k < names.Length; k++)
        {
            var col = counts.Select(d => d[k]).OrderBy(x => x).ToArray();
            sb.Append(CultureInfo.InvariantCulture, $"cnt_{names[k]}={col[col.Length / 2]}\n");
        }
        sb.Append("series_awake=").Append(string.Join(',', counts.Select(d => d[0]))).Append('\n');
        return sb.ToString();
    }

    // Scripted input, as in the 2D driver: walk along +x at the script's speed, reversing every period steps, jump every so many steps, stand still from the stop step; gravity is integrated into a vertical
    // speed that is held at a small downward value while grounded, so the mover keeps contact with ramps and stairs.
    void MoveCharacters(float dt)
    {
        for (int i = 0; i < characters.Count; i++)
        {
            var (ch, sc, _) = characters[i];
            bool active = stepNo < sc.Stop;
            float walk = active ? (float)sc.Speed * (((stepNo / sc.Period) & 1) == 0 ? 1f : -1f) : 0f;
            if (ch.IsGrounded && vy[i] <= 0f) vy[i] = active && sc.Jump > 0 && stepNo > 0 && (stepNo + i) % sc.Jump == 0 ? 5f : -1f;
            else vy[i] -= gravity * dt;
            ch.Move(new Vector3F(walk * dt, vy[i] * dt, 0f), dt);
        }
    }

    // Pose drives: every driven swing-twist joint is retargeted each step to a rotation about its drive axis (joint frame) by amplitude times a triangle wave 0..1..0 of the given period in steps.
    void DriveJoints()
    {
        foreach (var (joint, d) in drives)
        {
            double angle = d.DriveAmp * Metrics3.Tri01(stepNo, d.DrivePeriod), h = 0.5 * angle;
            float sn = (float)Math64.Sin(h), cs = (float)Math64.Cos(h);
            joint.SetPoseDrive(new QuaternionF((float)d.DriveAx * sn, (float)d.DriveAy * sn, (float)d.DriveAz * sn, cs), (float)d.DriveK, (float)d.DriveC, (float)d.DriveMax);
        }
    }

    void DriveVehicles()
    {
        foreach (var (car, sc, _) in vehicles)
        {
            var (throttle, steer, brake) = sc.At(stepNo);
            car.Drive((float)throttle, (float)steer, (float)brake);
        }
    }

    P3.SoftBody[] softs = [];
    SoftDef?[] skinDefs = [];
    public int SoftDoubles { get { int n = 0; foreach (var f in softs) n += 6 * f.ParticleCount; return n; } }
    public void SoftSnapshot(double[] d)
    {
        int k = 0;
        foreach (var f in softs)
        {
            var v = f.RawVelocities;
            for (int i = 0; i < f.ParticleCount; i++)
            {
                var p = f.GetPosition(i);
                d[k++] = p.X; d[k++] = p.Y; d[k++] = p.Z; d[k++] = v[3 * i]; d[k++] = v[3 * i + 1]; d[k++] = v[3 * i + 2];
            }
        }
    }

    public int WheelDoubles { get { int n = 0; foreach (var (car, _, _) in vehicles) n += 3 * car.WheelCount; return n; } }

    public void Wheels(double[] d)
    {
        int k = 0;
        foreach (var (car, _, _) in vehicles)
            for (int w = 0; w < car.WheelCount; w++) { var c = car.GetWheel(w).Center; d[k++] = c.X; d[k++] = c.Y; d[k++] = c.Z; }
    }

    public void WakeAll(Scene s)
    {
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Type == "dynamic") bodies[i].Wake();
    }

    public void Snapshot(double[] d)
    {
        for (int i = 0; i < bodies.Length; i++)
        {
            int o = i * 14; var b = bodies[i];
            var p = b.Position; var q = b.Rotation; var v = b.LinearVelocity; var w = b.AngularVelocity;
            d[o] = p.X; d[o + 1] = p.Y; d[o + 2] = p.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
            d[o + 7] = v.X; d[o + 8] = v.Y; d[o + 9] = v.Z; d[o + 10] = w.X; d[o + 11] = w.Y; d[o + 12] = w.Z;
            d[o + 13] = b.IsAwake ? 1 : 0;
        }
    }

    // A hash of every query's full result (shape, point, normal, fraction, distance, sub-shape; every overlapped shape in order), so two builds can be compared bit for bit on the scene's queries.
    ulong QueryDigest(Scene s)
    {
        ulong h = 14695981039346656037UL;
        void Mix(ulong v) { h = (h ^ v) * 1099511628211UL; }
        void MixHit(in P3.QueryHit3D x)
        {
            Mix(x.IsHit ? 1UL : 0UL);
            if (!x.IsHit) return;
            Mix((ulong)x.Shape.RawHandle.Index); Mix((ulong)BitConverter.DoubleToInt64Bits(x.Point.X)); Mix((ulong)BitConverter.DoubleToInt64Bits(x.Point.Y)); Mix((ulong)BitConverter.DoubleToInt64Bits(x.Point.Z));
            Mix((uint)BitConverter.SingleToInt32Bits(x.Normal.X)); Mix((uint)BitConverter.SingleToInt32Bits(x.Normal.Y)); Mix((uint)BitConverter.SingleToInt32Bits(x.Normal.Z));
            Mix((uint)BitConverter.SingleToInt32Bits(x.Fraction)); Mix((uint)BitConverter.SingleToInt32Bits(x.Distance)); Mix((ulong)x.SubShapeId);
        }

        var buffer = new P3.QueryHit3D[4096];
        foreach (var q in s.Queries)
        {
            var P = q.P;
            if (q.Kind == "ray")
            {
                bool hit = world.Raycast(new Vector3D(P[0], P[1], P[2]), new Vector3F((float)P[3], (float)P[4], (float)P[5]), out var x, (float)P[6]);
                MixHit(hit ? x : default);
            }
            else if (q.Kind == "overlap")
            {
                int n = world.OverlapSphereAll(new Vector3D(P[0], P[1], P[2]), (float)P[3], buffer);
                Mix((ulong)n);
                for (int i = 0; i < n; i++) MixHit(buffer[i]);
                bool near = world.OverlapSphere(new Vector3D(P[0], P[1], P[2]), (float)P[3], out var y);
                MixHit(near ? y : default);
            }
            else
            {
                float tx = (float)P[4], ty = (float)P[5], tz = (float)P[6];
                bool hit = world.SphereCast(new Vector3D(P[0], P[1], P[2]), (float)P[3], new Vector3F(tx, ty, tz), out var x, float.Sqrt(tx * tx + ty * ty + tz * tz));
                MixHit(hit ? x : default);
                int n = world.SphereCastAll(new Vector3D(P[0], P[1], P[2]), (float)P[3], new Vector3F(tx, ty, tz), buffer, float.Sqrt(tx * tx + ty * ty + tz * tz));
                Mix((ulong)n);
                for (int i = 0; i < n; i++) MixHit(buffer[i]);
            }
        }

        return h;
    }

    // The same protocol as the reference drivers; the first repetition is untimed when there are several.
    public void Queries(Scene s, StringBuilder sb)
    {
        sb.Append(CultureInfo.InvariantCulture, $"query_digest={QueryDigest(s):x16}\n");
        var ns = new double[3]; var count = new int[3]; var hits = new long[3]; var best = new double[] { double.MaxValue, double.MaxValue, double.MaxValue };
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        var buffer = new P3.QueryHit3D[256];
        string[] names = ["ray", "overlap", "cast"];
        for (int rep = 0; rep < s.QueryReps; rep++)
        {
            bool measure = s.QueryReps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long h = 0; int c = 0; long t0 = Stopwatch.GetTimestamp();
                foreach (var q in s.Queries)
                {
                    if (q.Kind != names[kind]) continue;
                    c++;
                    var P = q.P;
                    if (kind == 0)
                    {
                        if (world.Raycast(new Vector3D(P[0], P[1], P[2]), new Vector3F((float)P[3], (float)P[4], (float)P[5]), out _, (float)P[6])) h++;
                    }
                    else if (kind == 1)
                    {
                        h += world.OverlapSphereAll(new Vector3D(P[0], P[1], P[2]), (float)P[3], buffer);
                    }
                    else
                    {
                        float tx = (float)P[4], ty = (float)P[5], tz = (float)P[6];
                        float len = float.Sqrt(tx * tx + ty * ty + tz * tz);
                        if (world.SphereCast(new Vector3D(P[0], P[1], P[2]), (float)P[3], new Vector3F(tx, ty, tz), out _, len)) h++;
                    }
                }
                if (measure) { double el = (Stopwatch.GetTimestamp() - t0) * nsPerTick; ns[kind] += el; count[kind] += c; hits[kind] = h; if (el < best[kind]) best[kind] = el; }
            }
        }
        // The same queries through the batch forms of the query object, which spread them over the world's worker threads (the thread count of this run).
        var qo = new P3.PhysicsQuery3D(world);
        var rays = new List<P3.RayInput3D>(); var overlaps = new List<P3.SphereInput3D>(); var casts = new List<P3.SphereInput3D>();
        foreach (var q in s.Queries)
        {
            var P = q.P;
            if (q.Kind == "ray") rays.Add(new P3.RayInput3D(new Vector3D(P[0], P[1], P[2]), new Vector3F((float)P[3], (float)P[4], (float)P[5]), (float)P[6]));
            else if (q.Kind == "overlap") overlaps.Add(new P3.SphereInput3D(new Vector3D(P[0], P[1], P[2]), (float)P[3]));
            else if (q.Kind == "cast")
            {
                float tx = (float)P[4], ty = (float)P[5], tz = (float)P[6];
                casts.Add(new P3.SphereInput3D(new Vector3D(P[0], P[1], P[2]), (float)P[3], new Vector3F(tx, ty, tz), float.Sqrt(tx * tx + ty * ty + tz * tz)));
            }
        }

        var rayArr = rays.ToArray(); var ovArr = overlaps.ToArray(); var castArr = casts.ToArray();
        var rayHits = new P3.QueryHit3D[rayArr.Length]; var castHits = new P3.QueryHit3D[castArr.Length]; var counts = new int[ovArr.Length];
        var bns = new double[3]; var bcount = new int[3]; var bhits = new long[3]; var bbest = new double[] { double.MaxValue, double.MaxValue, double.MaxValue };
        for (int rep = 0; rep < s.QueryReps; rep++)
        {
            bool measure = s.QueryReps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long t0 = Stopwatch.GetTimestamp(); long h = 0; int c;
                if (kind == 0) { h = qo.RaycastBatch(rayArr, rayHits); c = rayArr.Length; }
                else if (kind == 1) { qo.OverlapSphereCountBatch(ovArr, counts); foreach (int n in counts) h += n; c = ovArr.Length; }
                else { h = qo.SphereCastBatch(castArr, castHits); c = castArr.Length; }
                if (measure) { double el = (Stopwatch.GetTimestamp() - t0) * nsPerTick; bns[kind] += el; bcount[kind] += c; bhits[kind] = h; if (el < bbest[kind]) bbest[kind] = el; }
            }
        }

        for (int k = 0; k < 3; k++) sb.Append(CultureInfo.InvariantCulture, $"query_{names[k]}_batch_ns={bns[k]:F0}\nquery_{names[k]}_batch_count={bcount[k]}\nquery_{names[k]}_batch_hits={bhits[k]}\nquery_{names[k]}_batch_best_ns={bbest[k]:F0}\nquery_{names[k]}_batch_best_count={bcount[k] / Math.Max(1, s.QueryReps > 1 ? s.QueryReps - 1 : 1)}\n");
        for (int k = 0; k < 3; k++) sb.Append(CultureInfo.InvariantCulture, $"query_{names[k]}_ns={ns[k]:F0}\nquery_{names[k]}_count={count[k]}\nquery_{names[k]}_hits={hits[k]}\nquery_{names[k]}_best_ns={best[k]:F0}\nquery_{names[k]}_best_count={count[k] / Math.Max(1, s.QueryReps > 1 ? s.QueryReps - 1 : 1)}\n");
    }
}
