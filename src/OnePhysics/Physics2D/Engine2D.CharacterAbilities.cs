using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : ICharacterExtHost
{
    private CharacterExtStore? _charExt;

    /// <summary>Seats, ledge hangs, climbing, rope grips and carrying; created on first use and registered with the world.</summary>
    public CharacterExtStore Extensions => _charExt ??= new CharacterExtStore(this);

    WorldCore ICharacterExtHost.Core => Core;
    bool ICharacterExtHost.Planar => true;
    int ICharacterExtHost.EngineSectionId => Id;
    CharacterExtStore ICharacterExtHost.Extensions => Extensions;
    int ICharacterExtHost.CharacterHighWater => CharHandles.HighWater;
    bool ICharacterExtHost.CharacterLive(int character) => CharHandles.IsLive(character);
    int ICharacterExtHost.CharacterGeneration(int character) => CharHandles.GenerationOf(character);
    int ICharacterExtHost.CharacterBody(int character) => CharBody[character];

    internal bool CharacterExtensionsConsistent() => _charExt == null || _charExt.Consistent();

    // ------------------------------------------------------------------ poses

    private void BodyOrigin(int b, out double x, out double y, out Rot2 q) => FrameOf(b, out x, out y, out q);

    /// <summary>The rotation of a planar quaternion (about z) as a cosine and sine, by the double-angle formulas.</summary>
    private static Rot2 PlanarRotation(QuaternionF q) => new(q.W * q.W - q.Z * q.Z, 2f * q.Z * q.W);

    /// <summary>The planar quaternion of a rotation: the half-angle by the half-angle formulas, so the pair round-trips.</summary>
    private static QuaternionF PlanarQuaternion(float cos, float sin)
    {
        // cos(a/2) and sin(a/2) from cos a and sin a: w = sqrt((1 + cos) / 2), z = sin / (2 w); the angle pi (w = 0) is z = 1.
        float w2 = 0.5f * (1f + cos);
        if (!(w2 > 1e-12f)) { return new QuaternionF(0f, 0f, 1f, 0f); }
        float w = float.Sqrt(w2);
        return new QuaternionF(0f, 0f, sin / (2f * w), w);
    }

    /// <summary>Puts a character's kinematic body at a pose with a velocity, without recording and without waking anything (the same placement as a teleport).</summary>
    private void PlaceCharacter(int c, double x, double y, Rot2 rotation, float vx, float vy)
    {
        int b = CharBody[c];
        var bodies = Core.Bodies;
        CancelKinematicDrive(b);
        var r = bodies.RotationOf(b);
        r[0] = rotation.C; r[1] = rotation.S;
        float lx = LocalCenter[2 * b], ly = LocalCenter[2 * b + 1];
        var p = bodies.PositionOf(b);
        p[0] = WorldCore.Grid(x) + (double)(rotation.C * lx - rotation.S * ly);
        p[1] = WorldCore.Grid(y) + (double)(rotation.S * lx + rotation.C * ly);
        Dp[2 * b] = 0f; Dp[2 * b + 1] = 0f;
        for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh])
        {
            Bp.MoveProxy(Proxy[sh], TightBox(b, in Geom[sh]), 0.0, 0.0);
        }

        var lin = bodies.LinearVelocityOf(b);
        lin[0] = vx; lin[1] = vy;
        CharVx[c] = vx; CharVy[c] = vy;
    }

    private float GripHeight(int c) => 0.5f * CharDef[c].Height - 0.2f;

    private void CharacterPosition(int c, out double x, out double y) => BodyOrigin(CharBody[c], out x, out y, out _);

    private void MarkInAir(int c)
    {
        CharGround[c] = GroundState.InAir; CharNx[c] = 0f; CharNy[c] = 0f; CharGroundBody[c] = -1; CharGroundGen[c] = 0;
    }

    // ------------------------------------------------------------------ the step hooks

    /// <summary>Before the step: a gripped rod carries the weight of whoever holds it, and characters standing on soft particles or a rod add their mass to what carries them.</summary>
    public void BeginCharacterAbilities(float dt)
    {
        var ext = _charExt;
        if (ext != null && ext.ActiveCount > 0)
        {
            for (int c = 0; c < CharHandles.HighWater; c++)
            {
                if (!CharHandles.IsLive(c) || c >= ext.States.Length || ext.States[c].Mode != CharacterMode.Roped) { continue; }
                ref var st = ref ext.States[c];
                if (Rods.Alive(st.Rod, st.RodGen)) { RefreshGripMasses(st.Rod); }
            }
        }

        ApplySupportMass(dt);
    }

    /// <summary>After the whole step: every seated, hanging, climbing or roped character is put exactly where its anchor went, and characters carried by their ground move with it.</summary>
    public void StepCharacterAbilities()
    {
        var ext = _charExt;
        if (ext == null || ext.ActiveCount == 0) { return; }
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c) || c >= ext.States.Length) { continue; }
            ref var st = ref ext.States[c];
            switch (st.Mode)
            {
                case CharacterMode.Free:
                    if (st.Riding) { SyncCarried(c, ref st); }
                    break;
                case CharacterMode.Roped:
                    SyncRope(c, ref st);
                    break;
                default:
                    SyncAnchored(c, ref st);
                    break;
            }
        }
    }

    private void SyncCarried(int c, ref CharacterExtState st)
    {
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen) || Core.Bodies.TypeOf(st.Body) == BodyType.Static)
        {
            var off = st; off.Riding = false;
            _charExt!.Set(c, in off);
            return;
        }

        BodyOrigin(st.Body, out double ox, out double oy, out Rot2 q);
        var l = q.Rotate(new Vec2(st.Local.X, st.Local.Y));
        double x = ox + (double)l.X, y = oy + (double)l.Y;
        var v = BodyVelocityAt(st.Body, x, y);
        PlaceCharacter(c, x, y, Rot2.Identity, v.X, v.Y);
    }

    private void SyncAnchored(int c, ref CharacterExtState st)
    {
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { DropHold(c, ref st); return; }
        BodyOrigin(st.Body, out double ox, out double oy, out Rot2 q);
        var l = q.Rotate(new Vec2(st.Local.X, st.Local.Y));
        double x = ox + (double)l.X, y = oy + (double)l.Y;
        var v = BodyVelocityAt(st.Body, x, y);
        Rot2 rot = Rot2.Identity;
        if (st.Mode == CharacterMode.Seated)
        {
            var lr = PlanarRotation(st.LocalRotation);
            rot = Rot2.FromVector(q.C * lr.C - q.S * lr.S, q.S * lr.C + q.C * lr.S);
        }

        PlaceCharacter(c, x, y, rot, v.X, v.Y);
    }

    private void SyncRope(int c, ref CharacterExtState st)
    {
        if (!Rods.Alive(st.Rod, st.RodGen)) { DropHold(c, ref st); return; }
        var d = Rods.Data[st.Rod]!;
        if (st.Particle + 1 >= d.PositionData.Length) { DropHold(c, ref st); return; }
        GripPoint(d, st.Particle, st.Fraction, out double gx, out double gy, out float vx, out float vy);
        PlaceCharacter(c, gx, gy - (double)GripHeight(c), Rot2.Identity, vx, vy);
    }

    /// <summary>The hold ended on its own (the body or rod is gone): the character falls free with the velocity it had.</summary>
    private void DropHold(int c, ref CharacterExtState st)
    {
        bool roped = st.Mode == CharacterMode.Roped;
        int rod = st.Rod, gen = st.RodGen;
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        if (roped && Rods.Alive(rod, gen)) { RefreshGripMasses(rod); }
        CharacterPosition(c, out double x, out double y);
        PlaceCharacter(c, x, y, Rot2.Identity, CharVx[c], CharVy[c]);
    }

    // ------------------------------------------------------------------ seats

    public void MountCharacter(int c, int seatBody, Vector2F local, float angleCos, float angleSin)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode == CharacterMode.Roped) { ReleaseCharacter(c); }
        var next = new CharacterExtState
        {
            CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, Mode = CharacterMode.Seated, Body = seatBody, BodyGen = Core.Bodies.Handles.GenerationOf(seatBody),
            Local = new Vector3F(local.X, local.Y, 0f), LocalRotation = PlanarQuaternion(angleCos, angleSin),
        };
        _charExt!.Set(c, in next);
        SyncAnchored(c, ref _charExt.States[c]);
        MarkInAir(c);
    }

    /// <summary>Lets go of whatever the character holds or sits in; returns the velocity it leaves with (the seat's, the rope's, or zero).</summary>
    public Vector2F ReleaseCharacter(int c)
    {
        var ext = Extensions;
        ref var st = ref ext.At(c);
        if (st.Mode == CharacterMode.Free) { return default; }
        Vector2F vel = default;
        CharacterPosition(c, out double x, out double y);
        int rod = st.Rod, gen = st.RodGen;
        bool roped = st.Mode == CharacterMode.Roped;
        if (st.Mode == CharacterMode.Seated || st.Mode == CharacterMode.Roped) { vel = new Vector2F(CharVx[c], CharVy[c]); }
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        ext.Set(c, in keep);
        MarkInAir(c);
        if (roped && Rods.Alive(rod, gen)) { RefreshGripMasses(rod); }
        PlaceCharacter(c, x, y, Rot2.Identity, vel.X, vel.Y);
        return vel;
    }

    /// <summary>Gets up from a seat: the character keeps the seat point's velocity and steps out by <paramref name="exitOffset"/> in the body's frame.</summary>
    public Vector2F DismountCharacter(int c, Vector2F exitOffset)
    {
        var st = Extensions.At(c);
        var vel = new Vector2F(CharVx[c], CharVy[c]);
        CharacterPosition(c, out double x, out double y);
        if ((exitOffset.X != 0f || exitOffset.Y != 0f) && Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen))
        {
            BodyOrigin(st.Body, out _, out _, out Rot2 q);
            var o = q.Rotate(new Vec2(exitOffset.X, exitOffset.Y));
            x += (double)o.X; y += (double)o.Y;
        }

        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        PlaceCharacter(c, x, y, Rot2.Identity, vel.X, vel.Y);
        return vel;
    }

    // ------------------------------------------------------------------ ropes

    private static void GripPoint(RodParticles d, int particle, float fraction, out double x, out double y, out float vx, out float vy)
    {
        var a = d.PositionData[particle]; var b = d.PositionData[particle + 1];
        x = a.X + (b.X - a.X) * fraction; y = a.Y + (b.Y - a.Y) * fraction;
        var va = d.VelocityData[particle]; var vb = d.VelocityData[particle + 1];
        vx = va.X + (vb.X - va.X) * fraction; vy = va.Y + (vb.Y - va.Y) * fraction;
    }

    /// <summary>The mass a rod particle has before any character holds it: half of each neighbouring segment, as the rod computes it when it is built.</summary>
    private static float BaseParticleMass(RodParticles d, int p)
    {
        double area = Math.PI * d.Material.Radius * d.Material.Radius;
        float mass = 0f;
        if (p > 0) { mass += (float)(area * d.Material.Density * d.Length[p - 1]) * 0.5f; }
        if (p < d.Length.Length) { mass += (float)(area * d.Material.Density * d.Length[p]) * 0.5f; }
        return mass;
    }

    /// <summary>
    /// Sets the inverse masses of a rod's particles from its own mass plus the characters holding it: a holder's mass is split between the two particles of its grip by the grip's fraction.
    /// The rod then swings, stretches and bends as the combined system, with no extra force or constraint. Pinned particles stay pinned.
    /// </summary>
    private float[] _gripAdded = new float[64];

    private void RefreshGripMasses(int rod)
    {
        var d = Rods.Data[rod]!;
        var ext = Extensions;
        int n = d.PositionData.Length;
        if (_gripAdded.Length < n) { _gripAdded = new float[Math.Max(n, 2 * _gripAdded.Length)]; }
        var added = _gripAdded;
        Array.Clear(added, 0, n);
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c) || c >= ext.States.Length) { continue; }
            ref var st = ref ext.States[c];
            if (st.Mode != CharacterMode.Roped || st.Rod != rod || st.Particle + 1 >= n) { continue; }
            added[st.Particle] += CharDef[c].Mass * (1f - st.Fraction);
            added[st.Particle + 1] += CharDef[c].Mass * st.Fraction;
        }

        for (int p = 0; p < n; p++)
        {
            if (d.Pinned[p]) { continue; }
            float baseMass = BaseParticleMass(d, p);
            if (!(baseMass > 0f)) { continue; }
            d.InverseMass[p] = added[p] > 0f ? 1f / (baseMass + added[p]) : 1f / baseMass;
        }
    }

    public void GrabRod(int c, int rod, int generation, float position)
    {
        var d = Rods.Data[rod]!;
        int n = d.PositionData.Length;
        if (!(position >= 0f && position <= n - 1)) { throw new ArgumentOutOfRangeException(nameof(position), position, "The grip position must be within [0, " + (n - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + "] (particle index units)."); }
        ref var cur = ref Extensions.At(c);
        int prevRod = cur.Mode == CharacterMode.Roped ? cur.Rod : -1, prevGen = cur.RodGen;
        int i = Math.Min((int)position, n - 2);
        var st = new CharacterExtState { CanClimb = cur.CanClimb, ExactCarry = cur.ExactCarry, Mode = CharacterMode.Roped, Rod = rod, RodGen = generation, Particle = i, Fraction = position - i, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in st);
        RefreshGripMasses(rod);
        if (prevRod >= 0 && prevRod != rod && Rods.Alive(prevRod, prevGen)) { RefreshGripMasses(prevRod); }
        MarkInAir(c);
        SyncRope(c, ref _charExt.States[c]);
    }

    /// <summary>Finds the point of a rod nearest to the character's hands and grips it when it is within <paramref name="reach"/>.</summary>
    public bool TryGrabRod(int c, int rod, int generation, float reach)
    {
        var d = Rods.Data[rod]!;
        CharacterPosition(c, out double cx, out double cy);
        double hx = cx, hy = cy + (double)GripHeight(c);
        double best = double.MaxValue; float bestS = 0f;
        for (int j = 0; j + 1 < d.PositionData.Length; j++)
        {
            var a = d.PositionData[j]; var b = d.PositionData[j + 1];
            double ex = b.X - a.X, ey = b.Y - a.Y, l2 = ex * ex + ey * ey;
            double t = l2 > 0.0 ? CharacterGeometry.Clamp01(((hx - a.X) * ex + (hy - a.Y) * ey) / l2) : 0.0;
            double dx = a.X + ex * t - hx, dy = a.Y + ey * t - hy;
            double dist = dx * dx + dy * dy;
            if (dist < best) { best = dist; bestS = j + (float)t; }
        }

        if (!(Math.Sqrt(best) <= reach + d.Material.Radius)) { return false; }
        GrabRod(c, rod, generation, bestS);
        return true;
    }

    /// <summary>Moves a rope grip along the rod by <paramref name="distance"/> metres (positive goes up, towards the end of the rod that is higher).</summary>
    public void ClimbRope(int c, float distance)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode != CharacterMode.Roped) { throw new InvalidOperationException("The character is not holding a rod."); }
        if (!Rods.Alive(st.Rod, st.RodGen)) { return; }
        var d = Rods.Data[st.Rod]!;
        int n = d.PositionData.Length;
        var a = d.PositionData[st.Particle]; var b = d.PositionData[st.Particle + 1];
        double len = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
        if (!(len > 1e-9)) { return; }
        double along = (b.Y - a.Y) / len;
        double sign = along >= -1e-6 ? 1.0 : -1.0;
        double s = st.Particle + (double)st.Fraction + sign * distance / len;
        s = s < 0.0 ? 0.0 : s > n - 1 ? n - 1 : s;
        int i = Math.Min((int)s, n - 2);
        st.Particle = i; st.Fraction = (float)(s - i);
        RefreshGripMasses(st.Rod);
        SyncRope(c, ref st);
    }

    /// <summary>True when the body is a character that is holding this rod: its capsule and the rod do not collide while it hangs there.</summary>
    private bool HoldsRod(int body, RodParticles rod)
    {
        var ext = _charExt;
        if (ext == null || ext.ActiveCount == 0) { return false; }
        int c = CharacterOfBody(body);
        if (c < 0 || c >= ext.States.Length) { return false; }
        ref var st = ref ext.States[c];
        return st.Mode == CharacterMode.Roped && (uint)st.Rod < (uint)Rods.Data.Length && ReferenceEquals(Rods.Data[st.Rod], rod);
    }

    // ------------------------------------------------------------------ ledges and climbable surfaces

    private int _ledgeIgnore = -1;
    private Func<Shape, bool>? _ledgePredicate;

    private QueryFilter2D LedgeFilter(int ignoreBody, uint layerMask)
    {
        _ledgePredicate ??= sh => { int o = Core.Shapes.Owner[sh.Handle.Index]; return o != _ledgeIgnore && !IsCharacterBody(o); };
        _ledgeIgnore = ignoreBody;
        return new QueryFilter2D { Predicate = _ledgePredicate, LayerMask = layerMask };
    }

    private struct Ledge
    {
        public int Body, BodyGen;
        public double Tx, Ty;        // where the hands go
        public float Nx, Ny;         // wall normal, towards the character
    }

    /// <summary>The direction a facing vector means in the plane: left or right.</summary>
    private static float Side(Vector2F facing)
    {
        if (!float.IsFinite(facing.X) || !float.IsFinite(facing.Y) || !(Math.Abs(facing.X) > 1e-6f)) { throw new ArgumentOutOfRangeException(nameof(facing), facing, "The facing direction must be finite and have a horizontal part."); }
        return facing.X < 0f ? -1f : 1f;
    }

    /// <summary>
    /// Looks for a ledge beside a character at (cx, cy): a ray at chest height along <paramref name="side"/> finds the wall, a ray straight down from just above the hands, a little inside the wall, finds the top.
    /// The wall must be steep, the top flat enough to stand on, and the top within a hand's reach (0.3 m below to 0.4 m above the hands). Works on every shape the rays do: polygons, chains, tile grids, height fields.
    /// </summary>
    private bool FindLedge(int c, double cx, double cy, float side, out Ledge ledge)
    {
        ledge = default;
        var def = CharDef[c];
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        float handUp = 0.5f * def.Height - 0.1f;
        var chest = new Vector2D(cx, cy + (double)(handUp - 0.3f));
        float reach = def.Radius + 0.4f;
        var dir = new Vector2F(side, 0f);
        if (!World!.Raycast(chest, dir, out var wall, reach, in filter)) { return false; }
        var nw = wall.Normal;
        if (!(wall.Distance > 0f) || Math.Abs(nw.Y) > 0.5f || nw.X * side > -0.5f) { return false; }
        var o = new Vector2D(wall.Point.X - (double)nw.X * 0.1, cy + (double)(handUp + 0.5f));
        if (!World.Raycast(o, new Vector2F(0f, -1f), out var top, 1.1f, in filter)) { return false; }
        if (!(top.Distance > 0.02f) || top.Normal.Y < 0.7f) { return false; }
        double inset = (top.Point.X - wall.Point.X) * nw.X;
        ledge = new Ledge { Body = top.Body.RawHandle.Index, BodyGen = top.Body.RawHandle.Generation, Tx = top.Point.X - (double)nw.X * inset, Ty = top.Point.Y, Nx = nw.X, Ny = nw.Y };
        return true;
    }

    private void HangCentre(int c, in Ledge l, out double x, out double y)
    {
        var def = CharDef[c];
        float drop = 0.5f * def.Height - 0.1f;
        float outward = def.Radius + 0.02f;
        x = l.Tx + (double)(l.Nx * outward);
        y = l.Ty - (double)drop;
    }

    private void EnterHold(int c, CharacterMode mode, int body, double x, double y, float nx, float ny)
    {
        ref var cur = ref Extensions.At(c);
        BodyOrigin(body, out double ox, out double oy, out Rot2 q);
        var local = q.InverseRotate(new Vec2((float)(x - ox), (float)(y - oy)));
        var nl = q.InverseRotate(new Vec2(nx, ny));
        var st = new CharacterExtState
        {
            CanClimb = cur.CanClimb, ExactCarry = cur.ExactCarry, Mode = mode, Body = body, BodyGen = Core.Bodies.Handles.GenerationOf(body),
            Local = new Vector3F(local.X, local.Y, 0f), LocalRotation = QuaternionF.Identity, Normal = new Vector3F(nl.X, nl.Y, 0f), Along = new Vector3F(0f, 1f, 0f),
        };
        if (cur.Mode == CharacterMode.Roped) { ReleaseCharacter(c); }
        _charExt!.Set(c, in st);
        MarkInAir(c);
        SyncAnchored(c, ref _charExt.States[c]);
        CharVx[c] = 0f; CharVy[c] = 0f;
        var lin = Core.Bodies.LinearVelocityOf(CharBody[c]);
        lin[0] = 0f; lin[1] = 0f;
    }

    public bool TryGrabLedge(int c, Vector2F facing)
    {
        if (!Extensions.At(c).CanClimb) { return false; }
        float side = Side(facing);
        CharacterPosition(c, out double cx, out double cy);
        if (!FindLedge(c, cx, cy, side, out var ledge)) { return false; }
        HangCentre(c, in ledge, out double x, out double y);
        EnterHold(c, CharacterMode.Hanging, ledge.Body, x, y, ledge.Nx, ledge.Ny);
        return true;
    }

    public bool TryGrabSurface(int c, Vector2F facing)
    {
        if (!Extensions.At(c).CanClimb) { return false; }
        var def = CharDef[c];
        float side = Side(facing);
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        CharacterPosition(c, out double cx, out double cy);
        if (!World!.Raycast(new Vector2D(cx, cy), new Vector2F(side, 0f), out var hit, def.Radius + 0.4f, in filter)) { return false; }
        if (!(hit.Distance >= 0f) || Math.Abs(hit.Normal.Y) > 0.5f || hit.Normal.X * side > -0.5f) { return false; }
        if (!Extensions.IsClimbable(hit.Shape.Handle.Index, hit.Shape.Handle.Generation)) { return false; }
        float off = def.Radius + 0.02f;
        EnterHold(c, CharacterMode.Climbing, hit.Body.RawHandle.Index, hit.Point.X + (double)(hit.Normal.X * off), cy, hit.Normal.X, hit.Normal.Y);
        return true;
    }

    /// <summary>The held surface's current normal and the character's centre, in world coordinates.</summary>
    private void HoldFrame(in CharacterExtState st, out double x, out double y, out float nx, out float ny)
    {
        BodyOrigin(st.Body, out double ox, out double oy, out Rot2 q);
        var l = q.Rotate(new Vec2(st.Local.X, st.Local.Y));
        x = ox + (double)l.X; y = oy + (double)l.Y;
        var n = q.Rotate(new Vec2(st.Normal.X, st.Normal.Y));
        nx = n.X; ny = n.Y;
    }

    /// <summary>Pulls a hanging character up and over onto the ledge it holds, when the capsule fits on top.</summary>
    public bool ClimbUpLedge(int c)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode != CharacterMode.Hanging) { throw new InvalidOperationException("The character is not hanging from a ledge."); }
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { return false; }
        HoldFrame(in st, out double x, out double y, out float nx, out _);
        return MantleOnto(c, x, y, nx);
    }

    private bool MantleOnto(int c, double cx, double cy, float nx)
    {
        var def = CharDef[c];
        float side = nx > 0f ? -1f : 1f;
        if (!FindLedge(c, cx, cy, side, out var ledge)) { return false; }
        float inward = def.Radius + 0.05f;
        float lift = 0.5f * def.Height + def.Skin + 0.01f;
        double tx = ledge.Tx + (double)(side * inward), ty = ledge.Ty + (double)lift;
        if (!CapsuleFitsAt(c, tx, ty)) { return false; }
        ref var st = ref Extensions.At(c);
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        PlaceCharacter(c, tx, ty, Rot2.Identity, 0f, 0f);
        return true;
    }

    /// <summary>True when the character's capsule at the given centre overlaps nothing deeper than a skin.</summary>
    private bool CapsuleFitsAt(int c, double x, double y)
    {
        var def = CharDef[c];
        float halfLength = 0.5f * def.Height - def.Radius;
        var ctx = new MoverCtx
        {
            Capsule = Shape2D.CreateCapsule(new Vec2(0f, -halfLength), new Vec2(0f, halfLength), def.Radius),
            Radius = def.Radius, HalfLength = halfLength, Skin = def.Skin, Range = 0f, MaxPush = 0.5f * def.Radius, Mask = def.LayerMask, IgnoreBody = CharBody[c],
        };
        MvGather(MvBox(x, y, 0f, 0f, ctx, 0.01f), ctx);
        MvPlanes(x, y, ctx);
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (_mvPlanes[i].Separation < -0.5f * def.Skin - 0.01f && !IsCharacterBody(MvOwner(_mvMeta[i].Slot))) { return false; }
        }

        return true;
    }

    /// <summary>The moves of a character that is not walking: climb a surface, climb a rope. A hanging or seated character does not move. Returns the contact flags (always none).</summary>
    public CharacterContact MoveBusy(int c, Vector2F displacement, float dt)
    {
        ref var st = ref Extensions.At(c);
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        if (!float.IsFinite(displacement.X) || !float.IsFinite(displacement.Y) || !(displacement.X * displacement.X + displacement.Y * displacement.Y <= 1e10f))
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The displacement must be finite and at most 100000 long.");
        }

        switch (st.Mode)
        {
            case CharacterMode.Roped:
                ClimbRope(c, displacement.Y);
                break;
            case CharacterMode.Climbing:
                MoveClimbing(c, displacement.Y);
                break;
        }

        return CharacterContact.None;
    }

    private void MoveClimbing(int c, float v)
    {
        var def = CharDef[c];
        ref var st = ref Extensions.At(c);
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { DropHold(c, ref st); return; }
        HoldFrame(in st, out double x, out double y, out float nx, out _);
        if (v == 0f) { return; }
        double cy = y + (double)v;
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        if (v < 0f)
        {
            // Climbing down onto the ground: stand on it.
            var feet = new Vector2D(x, y - (double)(0.5f * def.Height - 0.05f));
            if (World!.Raycast(feet, new Vector2F(0f, -1f), out var ground, 0.05f - v + 0.02f, in filter) && ground.Normal.Y >= 0.7f && ground.Distance > 0f)
            {
                ReleaseCharacter(c);
                return;
            }
        }

        // The surface is looked for at the hands: the climb ends when they pass the top.
        float handUp = 0.5f * def.Height - 0.1f;
        var from = new Vector2D(x + (double)(nx * 0.5f), cy + (double)handUp);
        if (World!.Raycast(from, new Vector2F(-nx, 0f), out var hit, def.Radius + 0.9f, in filter) && hit.Distance > 0f && hit.Normal.X * nx >= 0.7f
            && Extensions.IsClimbable(hit.Shape.Handle.Index, hit.Shape.Handle.Generation) && hit.Body.RawHandle.Index == st.Body)
        {
            float off = def.Radius + 0.02f;
            EnterHold(c, CharacterMode.Climbing, st.Body, hit.Point.X + (double)(hit.Normal.X * off), cy, hit.Normal.X, hit.Normal.Y);
            return;
        }

        if (v > 0f) { MantleOnto(c, x, cy, nx); }
    }

    /// <summary>A jump off the held surface: lets go and returns a velocity away from it, <paramref name="speed"/> along its normal and <paramref name="upSpeed"/> up.</summary>
    public Vector2F WallJump(int c, float speed, float upSpeed)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode != CharacterMode.Hanging && st.Mode != CharacterMode.Climbing) { throw new InvalidOperationException("The character is not hanging from a ledge or climbing a surface."); }
        HoldFrame(in st, out _, out _, out float nx, out float ny);
        ReleaseCharacter(c);
        var v = new Vector2F(nx * speed, ny * speed + upSpeed);
        CharVx[c] = v.X; CharVy[c] = v.Y;
        var lin = Core.Bodies.LinearVelocityOf(CharBody[c]);
        lin[0] = v.X; lin[1] = v.Y;
        return v;
    }

    public bool CharacterIsBusy(int c) => _charExt != null && c < _charExt.States.Length && _charExt.States[c].Mode != CharacterMode.Free;

    public bool CharacterCarriesByPose(int c) => _charExt != null && c < _charExt.States.Length && _charExt.States[c].ExactCarry;

    public CharacterMode ModeOf(int c) => _charExt != null && c < _charExt.States.Length ? _charExt.States[c].Mode : CharacterMode.Free;

    /// <summary>After a move of a pose-carried character: remember where it stands in the frame of the moving body under it, so the step can take it along exactly.</summary>
    public void UpdateCarried(int c, Vector2D position, Body ground)
    {
        var st = Extensions.At(c);
        if (ground.World == null || !ground.IsValid || ground.Type == BodyType.Static)
        {
            if (st.Riding) { st.Riding = false; _charExt!.Set(c, in st); }
            return;
        }

        int gb = ground.RawHandle.Index;
        BodyOrigin(gb, out double ox, out double oy, out Rot2 q);
        st.Riding = true;
        st.Body = gb; st.BodyGen = ground.RawHandle.Generation;
        var l = q.InverseRotate(new Vec2((float)(position.X - ox), (float)(position.Y - oy)));
        st.Local = new Vector3F(l.X, l.Y, 0f);
        st.LocalRotation = QuaternionF.Identity;
        _charExt!.Set(c, in st);
    }

    /// <summary>A teleport ends every hold and carry (the position is the caller's now).</summary>
    public void DropForTeleport(int c)
    {
        if (_charExt == null || c >= _charExt.States.Length) { return; }
        ref var st = ref _charExt.States[c];
        if (st.Mode == CharacterMode.Free && !st.Riding) { return; }
        int rod = st.Rod, gen = st.RodGen;
        bool roped = st.Mode == CharacterMode.Roped;
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        _charExt.Set(c, in keep);
        if (roped && Rods.Alive(rod, gen)) { RefreshGripMasses(rod); }
        CharVx[c] = 0f; CharVy[c] = 0f;
    }

    // ------------------------------------------------------------------ debug

    /// <summary>One entry per character with a hold: where it is held (seat, ledge, surface point or rope grip) and the surface normal.</summary>
    public void AddDebugHolds(Debug.DebugGeometry g)
    {
        var ext = _charExt;
        if (ext == null) { return; }
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c) || c >= ext.States.Length || ext.States[c].Mode == CharacterMode.Free) { continue; }
            ref var st = ref ext.States[c];
            Vector3D point; Vector3F normal = default; BodyHandle held = default;
            if (st.Mode == CharacterMode.Roped)
            {
                if (!Rods.Alive(st.Rod, st.RodGen)) { continue; }
                GripPoint(Rods.Data[st.Rod]!, st.Particle, st.Fraction, out double gx, out double gy, out _, out _);
                point = new Vector3D(gx, gy, 0.0);
            }
            else
            {
                if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { continue; }
                BodyOrigin(st.Body, out double ox, out double oy, out Rot2 q);
                var l = q.Rotate(new Vec2(st.Local.X, st.Local.Y));
                point = new Vector3D(ox + (double)l.X, oy + (double)l.Y, 0.0);
                if (st.Mode != CharacterMode.Seated) { var n = q.Rotate(new Vec2(st.Normal.X, st.Normal.Y)); normal = new Vector3F(n.X, n.Y, 0f); }
                held = Core.HandleOfBody(st.Body);
            }

            g.Add(new Debug.DebugCharacterHold(Core.HandleOfBody(CharBody[c]), (byte)st.Mode, held, point, normal));
        }
    }
}
