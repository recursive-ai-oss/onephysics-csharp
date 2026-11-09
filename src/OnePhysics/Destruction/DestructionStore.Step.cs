using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Destruction;

internal sealed partial class DestructionStore
{
    private int[] _bodyRec = [];
    private int[] _shapePart = [];
    private int[] _loadNext = new int[64];
    private int[] _loadPart = new int[64];
    private Vector3F[] _loadForce = new Vector3F[64];
    private Vector3F[] _loadPoint = new Vector3F[64];
    private int _loadCount;
    private readonly List<int> _todo = new();

    private void AddLoad(Record rec, int part, Vector3F force, Vector3F point)
    {
        if (_loadCount == _loadNext.Length)
        {
            int n = _loadCount * 2;
            Array.Resize(ref _loadNext, n); Array.Resize(ref _loadPart, n); Array.Resize(ref _loadForce, n); Array.Resize(ref _loadPoint, n);
        }
        int i = _loadCount++;
        _loadPart[i] = part; _loadForce[i] = force; _loadPoint[i] = point;
        _loadNext[i] = rec.LoadHead; rec.LoadHead = i;
    }

    /// <summary>
    /// The destruction pass of a step, after events are delivered: reads the contact impulses of the step, breaks bonds whose stress exceeds their strength, fractures bodies whose impact impulse exceeds their
    /// threshold (both in ascending body slot order), then applies the debris policy. A world with nothing registered returns at once.
    /// </summary>
    internal void AfterStep(float dt)
    {
        if (Idle) return;
        PruneDead();
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies; var cs = core.Constraints;
        if (Records.Count != 0)
        {
            int nb = bodies.HighWater, ns = core.Shapes.HighWater;
            if (_bodyRec.Length < nb) _bodyRec = new int[Math.Max(nb, 2 * _bodyRec.Length)];
            if (_shapePart.Length < ns) _shapePart = new int[Math.Max(ns, 2 * _shapePart.Length)];
            Array.Fill(_bodyRec, -1, 0, nb);
            Array.Fill(_shapePart, -1, 0, ns);
            _loadCount = 0;
            for (int i = 0; i < Records.Count; i++)
            {
                var rec = Records[i];
                _bodyRec[rec.Body] = i; rec.Peak = 0f; rec.PeakRel = default; rec.LoadHead = -1;
                for (int p = 0; p < rec.Parts.Length; p++)
                {
                    int s = rec.Parts[p].Shape;
                    if ((uint)s < (uint)ns && core.Shapes.Handles.IsAlive(s, rec.Parts[p].ShapeGeneration) && core.Shapes.Owner[s] == rec.Body) _shapePart[s] = p;
                }
            }
            GatherLoads(core, cs, dt);
            _todo.Clear();
            for (int i = 0; i < Records.Count; i++)
            {
                var rec = Records[i];
                if (!bodies.IsDynamic(rec.Body) && rec.Bonds == null) continue;
                if (rec.Fracture is { } f && f.ImpulseThreshold > 0f && rec.Peak >= f.ImpulseThreshold && bodies.IsDynamic(rec.Body)) { _todo.Add(rec.Body); continue; }
                // A bond broken earlier whose compound could not be split yet (the per-step limit below) is retried here.
                if (rec.Bonds != null && (EvaluateStress(rec) || AnyDeadBond(rec))) _todo.Add(rec.Body);
            }
            long createdAtStart = _created;
            foreach (int body in _todo)
            {
                // A cascade of fractures (children that break at once, again and again) is spread over steps instead of exhausting memory in one.
                if (_created - createdAtStart >= MaxCreatedPerStep || Fragments.Count >= HardFragmentCap) break;
                int ri = FindRecord(body);
                if (ri < 0) continue;
                var rec = Records[ri];
                if (!BodyAlive(rec.Body, rec.Generation)) continue;
                if (rec.Fracture is { } f && rec.Bonds == null)
                {
                    var rel = rec.PeakRel;
                    if (Fracture(body, f, rel, rec.Peak, FractureKind.Impact) == null) Remove(body); // cannot be cut (unsupported shape or in a machine): stop trying
                }
                else if (rec.Bonds != null) SplitComponents(rec);
            }
        }
        ManageDebris();
    }

    /// <summary>The most bodies automatic fractures and bond breaks create in one step; what is left waits for the next step.</summary>
    internal const int MaxCreatedPerStep = 512;

    /// <summary>Automatic fracture and bond breaks stop while this many fragments are tracked (set <see cref="DebrisOptions.MaxFragments"/> lower to cull sooner); the bodies stay whole until the count falls.</summary>
    internal const int HardFragmentCap = 2048;

    private static bool AnyDeadBond(Record rec)
    {
        foreach (var b in rec.BondList) if (!b.Alive) return true;
        return false;
    }

    private void GatherLoads(WorldCore core, ConstraintStore cs, float dt)
    {
        bool planar = Host.Planar;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact) continue;
            int count = cs.PointCount[c];
            if (count == 0) continue;
            int a = cs.BodyA[c], b = cs.BodyB[c];
            int ra = a >= 0 ? _bodyRec[a] : -1, rb = b >= 0 ? _bodyRec[b] : -1;
            if (ra < 0 && rb < 0) continue;
            var pl = cs.Payload(c);
            var n = new Vector3F(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], planar ? 0f : pl[ManifoldLayout.Normal + 2]);
            float peak = 0f, best = 0f; int bestPoint = 0;
            for (int k = 0; k < count; k++)
            {
                float m = pl[ManifoldLayout.MaxNormalImpulse(k)];
                if (m > 0f) { peak += m; if (m > best) { best = m; bestPoint = k; } }
            }
            float scale = core.SubstepsOf(c) / dt;
            if (ra >= 0)
            {
                var rec = Records[ra];
                if (rec.Fracture != null && peak > rec.Peak) { rec.Peak = peak; rec.PeakRel = Anchor(pl, ManifoldLayout.AnchorA(bestPoint), planar); }
                if (rec.Bonds != null)
                {
                    int part = _shapePart[cs.ShapeA[c]];
                    if (part >= 0)
                        for (int k = 0; k < count; k++) AddLoad(rec, part, n * (-(pl[ManifoldLayout.NormalImpulse(k)] * scale)), Anchor(pl, ManifoldLayout.AnchorA(k), planar));
                }
            }
            if (rb >= 0)
            {
                var rec = Records[rb];
                if (rec.Fracture != null && peak > rec.Peak) { rec.Peak = peak; rec.PeakRel = Anchor(pl, ManifoldLayout.AnchorB(bestPoint), planar); }
                if (rec.Bonds != null)
                {
                    int part = _shapePart[cs.ShapeB[c]];
                    if (part >= 0)
                        for (int k = 0; k < count; k++) AddLoad(rec, part, n * (pl[ManifoldLayout.NormalImpulse(k)] * scale), Anchor(pl, ManifoldLayout.AnchorB(k), planar));
                }
            }
        }
    }

    private static Vector3F Anchor(Span<float> pl, int o, bool planar) => new(pl[o], pl[o + 1], planar ? 0f : pl[o + 2]);

    // ------------------------------------------------------------------ bond stress

    private Vector3D[] _r = [], _force = [], _moment = [], _G = [], _H = [];
    private int[] _depth = [], _order = [], _queue = [], _adjStart = [], _adjBond = [], _seen = [];
    private long[] _keys = [];
    private Vector3D[] _bondP = [], _bondM = [];

    private static void Ensure<T>(ref T[] a, int n) { if (a.Length < n) a = new T[Math.Max(n, 2 * a.Length)]; }

    private static Vector3D D(Vector3F v) => new(v.X, v.Y, v.Z);

    /// <summary>
    /// Resolves the loads on a breakable body into bond stresses and breaks the bonds over their strength. The model (support-graph flow, after the force balance of Bao, Hodgins and Müller 2007 on a bond network):
    /// each part needs the bond force that makes its motion equal its loads (contact forces from the solver's impulses, weight, and for a free body the inertial force of the rigid motion); that demand flows
    /// from the part towards the anchored parts (or, with none, the most loaded part), splitting over the bonds that lead one step closer in proportion to their area; each bond then carries a force and a moment
    /// about its face centre. Stress = tension / A + shear factor * shear / A + bending moment / W, with W = A^1.5 / 6 (3D) or L^2 / 6 (2D, unit thickness). Returns true when any bond broke this step.
    /// </summary>
    private bool EvaluateStress(Record rec)
    {
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        int body = rec.Body;
        bool dynamic = bodies.TypeOf(body) == BodyType.Dynamic;
        if (dynamic && !bodies.IsAwake(body)) return false;
        int n = rec.Parts.Length, m = rec.BondList.Length;
        if (n == 0) return false;
        Ensure(ref _r, n); Ensure(ref _force, n); Ensure(ref _moment, n); Ensure(ref _G, n); Ensure(ref _H, n);
        Ensure(ref _depth, n); Ensure(ref _order, n); Ensure(ref _queue, 2 * n + 1); Ensure(ref _adjStart, n + 1); Ensure(ref _adjBond, 2 * m + 1); Ensure(ref _seen, n); Ensure(ref _keys, n);
        Ensure(ref _bondP, m + 1); Ensure(ref _bondM, m + 1);
        var lc = host.LocalCenter(body);
        var g = host.Gravity * host.GravityScaleOf(body);
        var gD = D(g);
        for (int i = 0; i < n; i++) { _r[i] = D(host.ToWorld(body, rec.Parts[i].Centroid - lc)); _force[i] = default; _moment[i] = default; }
        // Loads: contact forces, summed per part and as moments about each part's centroid.
        Vector3D totalF = default, totalT = default;
        for (int l = rec.LoadHead; l >= 0; l = _loadNext[l])
        {
            int p = _loadPart[l];
            var F = D(_loadForce[l]); var at = D(_loadPoint[l]);
            _force[p] += F;
            _moment[p] += Vector3D.Cross(at - _r[p], F);
            totalF += F; totalT += Vector3D.Cross(at, F);
        }
        double mass = 0;
        for (int i = 0; i < n; i++) mass += rec.Parts[i].Mass;
        Vector3D aCm = default, alpha = default, omega = default;
        if (dynamic)
        {
            double M = host.MassOf(body);
            if (!(M > 0)) return false;
            aCm = (totalF + gD * M) / M;
            omega = D(AngularVelocityOf(body));
            var iw = D(host.InertiaTimes(body, ToF(omega)));
            alpha = D(host.InverseInertiaTimes(body, ToF(totalT - Vector3D.Cross(omega, iw))));
        }
        // Demand of each part: the force and moment its bonds must supply.
        for (int i = 0; i < n; i++)
        {
            double mi = rec.Parts[i].Mass;
            Vector3D accel = dynamic ? aCm + Vector3D.Cross(alpha, _r[i]) + Vector3D.Cross(omega, Vector3D.Cross(omega, _r[i])) : default;
            _G[i] = accel * mi - gD * mi - _force[i];
            _H[i] = -_moment[i];
        }
        // Adjacency of the live bonds.
        for (int i = 0; i <= n; i++) _adjStart[i] = 0;
        for (int k = 0; k < m; k++) { var b = rec.BondList[k]; b.Stress = 0f; if (b.Alive) { _adjStart[b.A + 1]++; _adjStart[b.B + 1]++; } }
        for (int i = 0; i < n; i++) _adjStart[i + 1] += _adjStart[i];
        for (int i = 0; i < n; i++) _seen[i] = _adjStart[i];
        for (int k = 0; k < m; k++) { var b = rec.BondList[k]; if (!b.Alive) continue; _adjBond[_seen[b.A]++] = k; _adjBond[_seen[b.B]++] = k; }
        // Depth from the anchors (a static body) or from the most loaded part of each component.
        for (int i = 0; i < n; i++) { _depth[i] = -1; _seen[i] = 0; }
        int qh = 0, qt = 0;
        if (!dynamic) for (int i = 0; i < n; i++) if (rec.Parts[i].Anchored) { _depth[i] = 0; _queue[qt++] = i; }
        Bfs(rec, ref qh, ref qt);
        for (int i = 0; i < n; i++)
        {
            if (_depth[i] >= 0) continue;
            // A component without an anchor: collect it, then root it at its most loaded part (largest force magnitude, ties lowest index).
            int start = qt;
            _queue[qt++] = i; _seen[i] = 1;
            for (int h = start; h < qt; h++)
            {
                int u = _queue[h];
                for (int e = _adjStart[u]; e < _adjStart[u + 1]; e++)
                {
                    var b = rec.BondList[_adjBond[e]];
                    int v = b.A == u ? b.B : b.A;
                    if (_seen[v] == 0) { _seen[v] = 1; _queue[qt++] = v; }
                }
            }
            int root = i; double best = -1;
            for (int h = start; h < qt; h++)
            {
                int u = _queue[h];
                double mag = _force[u].LengthSquared() + rec.Parts[u].Mass * rec.Parts[u].Mass;
                if (mag > best) { best = mag; root = u; }
            }
            _depth[root] = 0;
            qh = qt; _queue[qt++] = root;
            Bfs(rec, ref qh, ref qt);
        }
        // Flow from the deepest parts towards the sources.
        int maxDepth = 0;
        for (int i = 0; i < n; i++) if (_depth[i] > maxDepth) maxDepth = _depth[i];
        for (int i = 0; i < n; i++) { _order[i] = i; _keys[i] = ((long)(maxDepth - _depth[i]) << 32) | (uint)i; }
        Array.Sort(_keys, _order, 0, n);
        bool planar = host.Planar;
        bool broke = false;
        for (int idx = 0; idx < n; idx++)
        {
            int i = _order[idx];
            if (_depth[i] <= 0) continue;
            double total = 0;
            for (int e = _adjStart[i]; e < _adjStart[i + 1]; e++)
            {
                var b = rec.BondList[_adjBond[e]];
                int j = b.A == i ? b.B : b.A;
                if (_depth[j] == _depth[i] - 1) total += b.Measure;
            }
            if (!(total > 0)) continue;
            var Gi = _G[i]; var Hi = _H[i];
            for (int e = _adjStart[i]; e < _adjStart[i + 1]; e++)
            {
                int k = _adjBond[e];
                var b = rec.BondList[k];
                int j = b.A == i ? b.B : b.A;
                if (_depth[j] != _depth[i] - 1) continue;
                double w = b.Measure / total;
                var P = Gi * w;
                var pk = D(host.ToWorld(body, b.Centroid - lc));
                var M = Hi * w - Vector3D.Cross(pk - _r[i], P);
                // Pass the load on to the part closer to the source.
                _G[j] += P;
                _H[j] += Vector3D.Cross(pk - _r[j], P) + M;
                // Stress on the bond.
                var nrm = D(host.ToWorld(body, b.A == i ? b.Normal : -b.Normal)); // from i towards j
                double area = b.Measure;
                double pn = Vector3D.Dot(P, nrm);
                var pt = P - nrm * pn;
                double mn = Vector3D.Dot(M, nrm);
                var mt = M - nrm * mn;
                double section = planar ? area * area / 6.0 : Math.Sqrt(area) * area / 6.0;
                double stress = Math.Max(0.0, pn) / area + rec.Bonds!.ShearFactor * pt.Length() / area + mt.Length() / section;
                b.Stress = (float)stress;
                double limit = b.Strength > 0f ? b.Strength : rec.Bonds.Strength;
                if (stress >= limit)
                {
                    if (++b.Overload >= rec.Bonds.OverloadSteps) { b.Alive = false; _bondBreaks++; broke = true; }
                }
                else b.Overload = 0;
            }
        }
        _ = mass;
        return broke;
    }

    private void Bfs(Record rec, ref int qh, ref int qt)
    {
        while (qh < qt)
        {
            int u = _queue[qh++];
            for (int e = _adjStart[u]; e < _adjStart[u + 1]; e++)
            {
                var b = rec.BondList[_adjBond[e]];
                int v = b.A == u ? b.B : b.A;
                if (_depth[v] < 0) { _depth[v] = _depth[u] + 1; _queue[qt++] = v; }
            }
        }
    }

    // ------------------------------------------------------------------ splitting a compound into its connected components

    /// <summary>
    /// Replaces a breakable body by one body per connected component of its live bonds (components ordered by their lowest part index, flood fill in ascending part order).
    /// On a static body, a component holding an anchored part stays static; every other component becomes a dynamic fragment.
    /// </summary>
    internal bool SplitComponents(Record rec)
    {
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        int body = rec.Body;
        int n = rec.Parts.Length;
        var comp = new int[n];
        Array.Fill(comp, -1);
        int comps = 0;
        var stack = new Stack<int>();
        for (int s = 0; s < n; s++)
        {
            if (comp[s] >= 0) continue;
            comp[s] = comps; stack.Push(s);
            while (stack.Count > 0)
            {
                int u = stack.Pop();
                foreach (var b in rec.BondList)
                {
                    if (!b.Alive) continue;
                    int v = b.A == u ? b.B : b.B == u ? b.A : -1;
                    if (v >= 0 && comp[v] < 0) { comp[v] = comps; stack.Push(v); }
                }
            }
            comps++;
        }
        if (comps < 2 || !host.CanReplace(body)) return false;
        var infos = new List<ShapeInfo>();
        if (!host.ReadShapes(body, infos)) return false;
        var infoOfShape = new Dictionary<long, ShapeInfo>();
        foreach (var si in infos) infoOfShape[((long)si.Shape << 32) | (uint)si.Generation] = si;
        bool staticSource = bodies.TypeOf(body) == BodyType.Static;
        var groups = new List<Group>(comps);
        for (int c = 0; c < comps; c++) groups.Add(new Group { Type = BodyType.Dynamic });
        var anchored = new bool[comps];
        for (int p = 0; p < n; p++)
        {
            var part = rec.Parts[p];
            if (!infoOfShape.TryGetValue(((long)part.Shape << 32) | (uint)part.ShapeGeneration, out var si)) continue;
            groups[comp[p]].Add(si.Piece, si.Template, p);
            if (part.Anchored) anchored[comp[p]] = true;
        }
        if (staticSource) for (int c = 0; c < comps; c++) if (anchored[c]) groups[c].Type = BodyType.Static;

        var com = CentreOfMass(body);
        WakeNeighbours(body);
        var made = Materialize(body, groups);
        var srcHandle = core.HandleOfBody(body);
        host.DestroyBody(srcHandle);
        Remove(body);
        long step = host.StepCount;
        int first = EventFragments.Count;
        foreach (var c in made)
        {
            EventFragments.Add(c.Body);
            var nr = GetOrAdd(c.Body.Index);
            nr.Bonds = new BondSettings { Strength = rec.Bonds!.Strength, ShearFactor = rec.Bonds.ShearFactor, OverloadSteps = rec.Bonds.OverloadSteps };
            var map = new Dictionary<int, int>();
            var parts = new List<Part>(c.Group.Pieces.Count);
            for (int i = 0; i < c.Group.Pieces.Count; i++)
            {
                int tag = c.Group.PartTags[i];
                var old = rec.Parts[tag];
                map[tag] = parts.Count;
                parts.Add(new Part { Shape = c.Shapes[i].Index, ShapeGeneration = c.Shapes[i].Generation, Anchored = old.Anchored, Measure = old.Measure, Mass = old.Mass, Centroid = old.Centroid });
            }
            nr.Parts = parts.ToArray();
            var bonds = new List<Bond>();
            foreach (var b in rec.BondList)
                if (b.Alive && map.TryGetValue(b.A, out int na) && map.TryGetValue(b.B, out int nb))
                    bonds.Add(new Bond { A = na, B = nb, Measure = b.Measure, Centroid = b.Centroid, Normal = b.Normal, Strength = b.Strength, Overload = b.Overload });
            nr.BondList = bonds.ToArray();
            if (c.Group.Type == BodyType.Dynamic) AddFragment(c.Body, (float)c.Group.Measure(), step); else _created++;
        }
        Events.Add(new FractureRecord((int)FractureKind.BondBreak, srcHandle, first, made.Count, 0f, com));
        return true;
    }

    // ------------------------------------------------------------------ debris

    /// <summary>Applies the lifetime, sleep-to-static and budget rules to the tracked fragments, in sequence (age) order.</summary>
    internal void ManageDebris()
    {
        if (Fragments.Count == 0) return;
        var host = Host; var core = host.DestructionCore; var bodies = core.Bodies;
        long now = host.StepCount;
        // Lifetime.
        if (LifetimeSteps > 0)
        {
            for (int i = 0; i < Fragments.Count; i++)
            {
                var f = Fragments[i];
                if (now - f.BirthStep < LifetimeSteps) continue;
                host.DestroyBody(core.HandleOfBody(f.Body)); Remove(f.Body);
                Fragments.RemoveAt(i); i--; _expired++;
            }
        }
        // Settled fragments turn static (a new static body with the same shapes; the handle changes).
        if (FreezeAfterSteps > 0)
        {
            for (int i = 0; i < Fragments.Count; i++)
            {
                var f = Fragments[i];
                if (f.Frozen) continue;
                int b = f.Body;
                if (!bodies.IsDynamic(b)) continue;
                if (bodies.IsAwake(b)) { f.AsleepSteps = 0; continue; }
                if (++f.AsleepSteps < FreezeAfterSteps) continue;
                if (Freeze(f)) _frozen++;
            }
        }
        // Budget: cull the smallest first, then the oldest.
        if (MaxFragments > 0 && Fragments.Count > MaxFragments)
        {
            int cull = Fragments.Count - MaxFragments;
            var order = new List<Fragment>(Fragments);
            order.Sort((a, b) => { int c = a.Measure.CompareTo(b.Measure); return c != 0 ? c : a.Sequence.CompareTo(b.Sequence); });
            for (int i = 0; i < cull; i++)
            {
                var f = order[i];
                host.DestroyBody(core.HandleOfBody(f.Body)); Remove(f.Body);
                Fragments.Remove(f); _culled++;
            }
        }
    }

    private bool Freeze(Fragment f)
    {
        var host = Host; var core = host.DestructionCore;
        var infos = new List<ShapeInfo>();
        if (!host.CanReplace(f.Body) || !host.ReadShapes(f.Body, infos) || infos.Count == 0) { f.AsleepSteps = 0; return false; }
        var g = new Group { Type = BodyType.Static };
        foreach (var s in infos) g.Add(s.Piece, s.Template, -1);
        var made = Materialize(f.Body, [g]);
        if (made.Count != 1) { foreach (var c in made) host.DestroyBody(c.Body); f.AsleepSteps = 0; return false; }
        var old = core.HandleOfBody(f.Body);
        host.DestroyBody(old);
        Remove(f.Body);
        f.Body = made[0].Body.Index; f.Generation = made[0].Body.Generation; f.Frozen = true; f.AsleepSteps = 0;
        return true;
    }

    // ------------------------------------------------------------------ debug

    /// <summary>Adds every bond of every breakable body to the geometry, in body slot order.</summary>
    internal void CollectDebug(RecursiveAI.OnePhysics.Debug.DebugGeometry geometry)
    {
        var host = Host; var bodies = host.DestructionCore.Bodies;
        foreach (var rec in Records)
        {
            if (rec.Bonds == null || !BodyAlive(rec.Body, rec.Generation)) continue;
            var com = CentreOfMass(rec.Body);
            var lc = host.LocalCenter(rec.Body);
            var handle = host.DestructionCore.HandleOfBody(rec.Body);
            foreach (var b in rec.BondList)
            {
                var offset = host.ToWorld(rec.Body, b.Centroid - lc);
                var n = host.ToWorld(rec.Body, b.Normal);
                geometry.Add(new RecursiveAI.OnePhysics.Debug.DebugBond(handle, b.A, b.B, new Vector3D(com.X + offset.X, com.Y + offset.Y, com.Z + offset.Z), n, b.Measure, b.Alive ? b.Stress : 0f, b.Strength > 0f ? b.Strength : rec.Bonds.Strength, b.Alive));
            }
        }
        _ = bodies;
    }
}
