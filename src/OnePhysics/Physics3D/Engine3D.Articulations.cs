using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.Articulations;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IArticulationHost
{
    private ArticulationStore? _articulations;

    /// <summary>Reduced-coordinate articulations; created on first use and registered with the world.</summary>
    internal ArticulationStore Articulations => _articulations ??= new ArticulationStore(this);

    WorldCore IArticulationHost.Core => Core;
    bool IArticulationHost.Planar => false;
    int IArticulationHost.EngineSectionId => Id;
    ArticulationStore IArticulationHost.Articulations => Articulations;
    D3 IArticulationHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, Opt.Gravity.Z);
    double IArticulationHost.ContactSlop => Opt.ContactSlop;
    double IArticulationHost.PushOutVelocity => Opt.PushOutVelocity;

    /// <summary>True for the body of an articulation link: the engine keeps its contacts up to date every step, even at rest, because the articulation drives it.</summary>
    private bool IsLinkBody(int b) => _articulations != null && _articulations.ArticulationOfBody(b) >= 0;

    /// <summary>Whether the pair of bodies is two links of one articulation that must not touch each other.</summary>
    private bool ArticulationBlocks(int ba, int bb) => _articulations != null && _articulations.SameArticulationBlocks(ba, bb);

    void IArticulationHost.SetLinkState(int body, D3 origin, Q4 rotation, D3 omega, D3 vOrigin)
    {
        var q = new QuaternionF((float)rotation.X, (float)rotation.Y, (float)rotation.Z, (float)rotation.W);
        var lc = LocalCenterOf(body);
        var arm = q.Rotate(lc);
        var p = Core.Bodies.PositionOf(body);
        p[0] = origin.X + arm.X; p[1] = origin.Y + arm.Y; p[2] = origin.Z + arm.Z;
        var r = Core.Bodies.RotationOf(body);
        r[0] = q.X; r[1] = q.Y; r[2] = q.Z; r[3] = q.W;
        SetLinkVelocityCore(body, q, omega, vOrigin);
    }

    void IArticulationHost.SetLinkVelocity(int body, D3 omega, D3 vOrigin) => SetLinkVelocityCore(body, RotationOfBody(body), omega, vOrigin);

    private void SetLinkVelocityCore(int body, QuaternionF q, D3 omega, D3 vOrigin)
    {
        var lc = LocalCenterOf(body);
        var arm = q.Rotate(lc);
        var com = new D3(vOrigin.X + (omega.Y * arm.Z - omega.Z * arm.Y), vOrigin.Y + (omega.Z * arm.X - omega.X * arm.Z), vOrigin.Z + (omega.X * arm.Y - omega.Y * arm.X));
        var v = Core.Bodies.LinearVelocityOf(body);
        v[0] = (float)com.X; v[1] = (float)com.Y; v[2] = (float)com.Z;
        var w = Core.Bodies.AngularVelocityOf(body);
        w[0] = (float)omega.X; w[1] = (float)omega.Y; w[2] = (float)omega.Z;
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
            var n = new D3(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], pl[ManifoldLayout.Normal + 2]);
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
                    var ra = new D3(pl[ia], pl[ia + 1], pl[ia + 2]); var rb = new D3(pl[ib], pl[ib + 1], pl[ib + 2]);
                    var rl = linkIsA ? ra : rb; var ro = linkIsA ? rb : ra;
                    into.Add(new HostContact
                    {
                        Constraint = c, LinkBody = link, Other = other,
                        Point = new D3(pos[0] + rl.X, pos[1] + rl.Y, pos[2] + rl.Z), Normal = linkIsA ? n : -n, OtherArm = ro,
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
            var n = new D3(pl[ManifoldLayout.Normal], pl[ManifoldLayout.Normal + 1], pl[ManifoldLayout.Normal + 2]);
            if (!bodyIsB) { n = -n; }
            for (int k = 0; k < cs.PointCount[c]; k++)
            {
                int ia = bodyIsB ? ManifoldLayout.AnchorB(k) : ManifoldLayout.AnchorA(k);
                into.Add(new HostContact { Constraint = c, LinkBody = -1, Other = other, Normal = n, OtherArm = new D3(pl[ia], pl[ia + 1], pl[ia + 2]), Separation = pl[ManifoldLayout.Separation(k)], Friction = pl[ManifoldLayout.Friction] });
            }
        }
    }

    private readonly List<int> _edgeScratch = new();

    void IArticulationHost.BodyMotion(int body, out D3 position, out D3 velocity, out D3 omega)
    {
        var p = Core.Bodies.PositionOf(body); var v = Core.Bodies.LinearVelocityOf(body); var w = Core.Bodies.AngularVelocityOf(body);
        position = new D3(p[0], p[1], p[2]); velocity = new D3(v[0], v[1], v[2]); omega = new D3(w[0], w[1], w[2]);
    }

    void IArticulationHost.BodyInverse(int body, out double invMass, out M3 invInertia)
    {
        if (!Core.Bodies.IsDynamic(body)) { invMass = 0.0; invInertia = default; return; }
        invMass = Core.Bodies.InverseMass[body];
        RefreshWorldInertia(body);
        var i = WInvI.AsSpan(6 * body, 6);
        invInertia = new M3(i[0], i[1], i[2], i[1], i[3], i[4], i[2], i[4], i[5]);
    }

    void IArticulationHost.AddBodyVelocity(int body, D3 dv, D3 dw)
    {
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(body)) { return; }
        if (!bodies.IsAwake(body)) { Core.WakeIsland(bodies.Island[body]); }
        var v = bodies.LinearVelocityOf(body);
        v[0] += (float)dv.X; v[1] += (float)dv.Y; v[2] += (float)dv.Z;
        var w = bodies.AngularVelocityOf(body);
        w[0] += (float)dw.X; w[1] += (float)dw.Y; w[2] += (float)dw.Z;
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

    /// <summary>Mass, centre of mass and inertia tensor (about it) of a body's shapes at their densities, as a dynamic body would have them; for building a link from its shapes.</summary>
    internal bool ShapeMassProperties(int b, out float mass, out Vector3F center, out Matrix3x3F inertia)
    {
        _parts.Clear();
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
            var g = Geoms[GeomOf[s]]!;
            if (g.IsStaticSoup || (g.IsConvex && g.Convex.Kind == ConvexShapeKind3D.Empty)) { continue; }
            var m = g.UnitMass;
            if ((g.Compound != null || g.Soup is SdfSoup3D) && m.Mass == 0f) { continue; }
            var l = Local[s];
            _parts.Add(new MassPropertiesF(m.Mass * Density[s], l.TransformPoint(m.Center), InertiaF.Rotate(m.Inertia, l.Rotation) * Density[s]));
        }

        mass = 0f; center = default; inertia = default;
        if (_parts.Count == 0) { return false; }
        var total = InertiaF.Combine(System.Runtime.InteropServices.CollectionsMarshal.AsSpan(_parts));
        mass = total.Mass; center = total.Center; inertia = total.Inertia;
        return mass > 0f && float.IsFinite(mass);
    }

    /// <summary>Sets the extents a link body needs for the narrowphase's reuse and search bounds (a kinematic body gets none from the mass update).</summary>
    internal void SetLinkExtents(int b)
    {
        ComputeExtents(b, LocalCenterOf(b), out float maxE, out float minE);
        MaxExtent[b] = maxE; MinExtent[b] = minE;
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
    internal void ShiftArticulations(Vector3D offset) => _articulations?.Shift(new D3(offset.X, offset.Y, offset.Z));
}
