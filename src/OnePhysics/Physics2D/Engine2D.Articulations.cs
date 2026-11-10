using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Articulations;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IArticulationHost
{
    private ArticulationStore? _articulations;

    /// <summary>Reduced-coordinate articulations; created on first use and registered with the world.</summary>
    internal ArticulationStore Articulations => _articulations ??= new ArticulationStore(this);

    WorldCore IArticulationHost.Core => Core;
    bool IArticulationHost.Planar => true;
    int IArticulationHost.EngineSectionId => Id;
    ArticulationStore IArticulationHost.Articulations => Articulations;
    D3 IArticulationHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, 0.0);
    double IArticulationHost.ContactSlop => Opt.ContactSlop;
    double IArticulationHost.PushOutVelocity => Opt.PushOutVelocity;

    /// <summary>True for the body of an articulation link: the engine keeps its contacts up to date every step, even at rest, because the articulation drives it.</summary>
    private bool IsLinkBody(int b) => _articulations != null && _articulations.ArticulationOfBody(b) >= 0;

    /// <summary>Whether the pair of bodies is two links of one articulation that must not touch each other.</summary>
    private bool ArticulationBlocks(int ba, int bb) => _articulations != null && _articulations.SameArticulationBlocks(ba, bb);

    void IArticulationHost.SetLinkState(int body, D3 origin, Q4 rotation, D3 omega, D3 vOrigin)
    {
        // A planar link turns about z only: the rotation is (cos, sin) of the angle, from the quaternion (0, 0, sin(a/2), cos(a/2)).
        float c = (float)(rotation.W * rotation.W - rotation.Z * rotation.Z), s = (float)(2.0 * rotation.W * rotation.Z);
        var r = Core.Bodies.RotationOf(body);
        r[0] = c; r[1] = s;
        float lx = LocalCenter[2 * body], ly = LocalCenter[2 * body + 1];
        double ax = c * lx - s * ly, ay = s * lx + c * ly;
        var p = Core.Bodies.PositionOf(body);
        p[0] = origin.X + ax; p[1] = origin.Y + ay;
        SetLinkVelocityCore(body, ax, ay, omega, vOrigin);
    }

    void IArticulationHost.SetLinkVelocity(int body, D3 omega, D3 vOrigin)
    {
        var r = Core.Bodies.RotationOf(body);
        float lx = LocalCenter[2 * body], ly = LocalCenter[2 * body + 1];
        SetLinkVelocityCore(body, r[0] * lx - r[1] * ly, r[1] * lx + r[0] * ly, omega, vOrigin);
    }

    private void SetLinkVelocityCore(int body, double ax, double ay, D3 omega, D3 vOrigin)
    {
        var v = Core.Bodies.LinearVelocityOf(body);
        v[0] = (float)(vOrigin.X - omega.Z * ay); v[1] = (float)(vOrigin.Y + omega.Z * ax);
        Core.Bodies.AngularVelocityOf(body)[0] = (float)omega.Z;
    }

    void IArticulationHost.GatherLinkContacts(List<HostContact> into)
    {
        var cs = Core.Constraints;
        var art = Articulations;
        var bodies = Core.Bodies;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact) { continue; }
            int count = cs.PointCount[c];
            if (count == 0) { continue; }
            int ba = cs.BodyA[c], bb = cs.BodyB[c];
            bool aLink = art.ArticulationOfBody(ba) >= 0, bLink = art.ArticulationOfBody(bb) >= 0;
            if (!aLink && !bLink) { continue; }
            var pl = cs.Payload(c);
            var n = new D3(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], 0.0);
            double mu = pl[ManifoldLayout.Friction];
            for (int side = 0; side < 2; side++)
            {
                bool linkIsA = side == 0;
                if (!(linkIsA ? aLink : bLink)) { continue; }
                int link = linkIsA ? ba : bb, other = linkIsA ? bb : ba;
                var pos = bodies.PositionOf(link);
                for (int k = 0; k < count; k++)
                {
                    int ia = ManifoldLayout.AnchorA(k), ib = ManifoldLayout.AnchorB(k);
                    var ra = new D3(pl[ia], pl[ia + 1], 0.0); var rb = new D3(pl[ib], pl[ib + 1], 0.0);
                    var rl = linkIsA ? ra : rb; var ro = linkIsA ? rb : ra;
                    into.Add(new HostContact
                    {
                        Constraint = c, LinkBody = link, Other = other,
                        Point = new D3(pos[0] + rl.X, pos[1] + rl.Y, 0.0), Normal = linkIsA ? n : -n, OtherArm = ro,
                        Separation = pl[ManifoldLayout.Separation(k)], Friction = mu,
                    });
                }
            }
        }
    }

    void IArticulationHost.GatherSupportContacts(int body, List<HostContact> into)
    {
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        _edgeScratch.Clear();
        for (int e = bodies.FirstEdge(body); e >= 0; e = cs.NextEdge(e)) { _edgeScratch.Add(e >> 1); }
        _edgeScratch.Sort();
        foreach (int c in _edgeScratch)
        {
            if (cs.KindOf(c) != ConstraintKind.Contact || cs.PointCount[c] == 0) { continue; }
            int ba = cs.BodyA[c], bb = cs.BodyB[c];
            bool bodyIsB = bb == body;
            int other = bodyIsB ? ba : bb;
            if (bodies.TypeOf(other) != BodyType.Static) { continue; }
            var pl = cs.Payload(c);
            var n = new D3(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], 0.0);
            if (!bodyIsB) { n = -n; }
            for (int k = 0; k < cs.PointCount[c]; k++)
            {
                int ia = bodyIsB ? ManifoldLayout.AnchorB(k) : ManifoldLayout.AnchorA(k);
                into.Add(new HostContact { Constraint = c, LinkBody = -1, Other = other, Normal = n, OtherArm = new D3(pl[ia], pl[ia + 1], 0.0), Separation = pl[ManifoldLayout.Separation(k)], Friction = pl[ManifoldLayout.Friction] });
            }
        }
    }

    private readonly List<int> _edgeScratch = new();

    void IArticulationHost.BodyMotion(int body, out D3 position, out D3 velocity, out D3 omega)
    {
        var p = Core.Bodies.PositionOf(body); var v = Core.Bodies.LinearVelocityOf(body);
        position = new D3(p[0], p[1], 0.0); velocity = new D3(v[0], v[1], 0.0); omega = new D3(0.0, 0.0, Core.Bodies.AngularVelocityOf(body)[0]);
    }

    void IArticulationHost.BodyInverse(int body, out double invMass, out M3 invInertia)
    {
        if (!Core.Bodies.IsDynamic(body)) { invMass = 0.0; invInertia = default; return; }
        invMass = Core.Bodies.InverseMass[body];
        invInertia = new M3(0, 0, 0, 0, 0, 0, 0, 0, Core.Bodies.InverseInertia[body]);
    }

    void IArticulationHost.AddBodyVelocity(int body, D3 dv, D3 dw)
    {
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(body)) { return; }
        if (!bodies.IsAwake(body)) { Core.WakeIsland(bodies.Island[body]); }
        var v = bodies.LinearVelocityOf(body);
        v[0] += (float)dv.X; v[1] += (float)dv.Y;
        bodies.AngularVelocityOf(body)[0] += (float)dw.Z;
    }

    /// <summary>Mass (per unit depth: area times density), centre of mass and inertia about it of a body's shapes, as a dynamic body would have them; for building a link from its shapes.</summary>
    internal bool ShapeMassProperties(int b, out float mass, out Vector2F center, out float inertia)
    {
        double m = 0, cx = 0, cy = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
            var md = Geom[s].ComputeMass();
            double dm = (double)Density[s] * md.Area;
            m += dm; cx += dm * md.CentroidX; cy += dm * md.CentroidY;
        }

        mass = 0f; center = default; inertia = 0f;
        if (!(m > 0)) { return false; }
        float nx = (float)(cx / m), ny = (float)(cy / m);
        double i = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
            i += (double)Density[s] * Geom[s].ComputeMass().InertiaAbout(nx, ny);
        }

        mass = (float)m; center = new Vector2F(nx, ny); inertia = (float)i;
        return float.IsFinite(mass) && mass > 0f && float.IsFinite(inertia);
    }

    /// <summary>Sets the extents a link body needs for the narrowphase's reuse and search bounds (a kinematic body gets none from the mass update).</summary>
    internal void SetLinkExtents(int b)
    {
        MaxExtent[b] = ComputeMaxExtent(b, LocalCenter[2 * b], LocalCenter[2 * b + 1]);
        float minExt = float.MaxValue;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            ref readonly var g = ref Geom[s];
            var cv = g.Vertices;
            float me = g.Radius;
            if (g.Kind == ShapeKind2D.Polygon)
            {
                var nrm = g.Normals;
                me = float.MaxValue;
                for (int i = 0; i < cv.Length; i++) { float d = Vec2.Dot(nrm[i], cv[i] - g.Centroid); if (d < me) { me = d; } }
                me += g.Radius;
            }

            if (me < minExt) { minExt = me; }
        }

        MinExtent[b] = minExt;
    }

    private struct LinkPairQuery : IOverlapCallback
    {
        public List<int> Found;

        public bool Report(int key, int userId) { Found.Add(userId); return true; }
    }

    private readonly List<int> _linkFound = new();

    /// <summary>
    /// Finds the pairs of link shapes with static and kinematic shapes, which the broadphase leaves out (it pairs a moving kinematic proxy with dynamic ones only). Each link shape asks the static and kinematic trees for boxes
    /// overlapping its stored box; candidates are taken in ascending shape order, so the pairs are added in an order that depends on state alone. Returns whether a pair was added that needs its first narrowphase run.
    /// </summary>
    private bool AddLinkPairs()
    {
        var art = _articulations;
        if (art == null || art.Count == 0) { return false; }
        bool any = false;
        var query = new LinkPairQuery { Found = _linkFound };
        for (int i = 0; i < art.Handles.HighWater; i++)
        {
            if (!art.Handles.IsLive(i)) { continue; }
            var st = art.States[i]!;
            for (int k = 0; k < st.Body.Length; k++)
            {
                for (int s = ShapeHead[st.Body[k]]; s >= 0; s = NextShape[s])
                {
                    _linkFound.Clear();
                    var fat = Bp.FatBoundsOfLive(Proxy[s]);
                    Bp.QueryAabb(in fat, ref query, BodyTypeMask.Static | BodyTypeMask.Kinematic);
                    _linkFound.Sort();
                    foreach (int u in _linkFound)
                    {
                        if (u == s) { continue; }
                        int a = Math.Min(s, u), b = Math.Max(s, u);
                        if (PairMap.ContainsKey(PairKey(a, b)) || !PairAllowed(a, b)) { continue; }
                        int np = AllocatePair(a, b);
                        if (_fresh.Length < PairHigh) { Array.Resize(ref _fresh, Math.Max(PairHigh, _fresh.Length * 2)); }
                        _fresh[np] = IsSensor(a) || IsSensor(b);
                        any |= _fresh[np];
                    }
                }
            }
        }

        return any;
    }

    internal void StepArticulations() { if (_articulations != null && _articulations.Count != 0) { _articulations.Step(Core.ContactStepDt); } }

    internal void EndArticulations() { if (_articulations != null && _articulations.Count != 0) { _articulations.EndStep(); } }

    internal bool ArticulationsConsistent() => _articulations == null || _articulations.Consistent();

    /// <summary>Destroying a link takes the whole articulation with it: the registry entry now, the other link bodies right after the body itself.</summary>
    private List<BodyHandle> TakeArticulationSiblings(int b)
    {
        var s = _articulations!.States[_articulations.ArticulationOfBody(b)]!;
        var others = new List<BodyHandle>();
        for (int k = 0; k < s.Body.Length; k++) { if (s.Body[k] != b) { others.Add(new BodyHandle(Core.Id, s.Body[k], s.BodyGen[k])); } }
        _articulations.Remove(_articulations.ArticulationOfBody(b));
        return others;
    }

    /// <summary>Moves the articulations' bases with an origin shift.</summary>
    internal void ShiftArticulations(double x, double y) => _articulations?.Shift(new D3(x, y, 0.0));
}
