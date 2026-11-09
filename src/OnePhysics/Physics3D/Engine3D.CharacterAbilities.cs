using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : ICharacterExtHost
{
    private CharacterExtStore? _charExt;

    /// <summary>Seats, ledge hangs, climbing, rope grips and carrying; created on first use and registered with the world.</summary>
    public CharacterExtStore Extensions => _charExt ??= new CharacterExtStore(this);

    WorldCore ICharacterExtHost.Core => Core;
    bool ICharacterExtHost.Planar => false;
    int ICharacterExtHost.EngineSectionId => Id;
    CharacterExtStore ICharacterExtHost.Extensions => Extensions;
    int ICharacterExtHost.CharacterHighWater => CharHandles.HighWater;
    bool ICharacterExtHost.CharacterLive(int character) => CharHandles.IsLive(character);
    int ICharacterExtHost.CharacterGeneration(int character) => CharHandles.GenerationOf(character);
    int ICharacterExtHost.CharacterBody(int character) => CharBody[character];

    internal bool CharacterExtensionsConsistent() => _charExt == null || _charExt.Consistent();

    /// <summary>True while some character is seated, hanging, climbing, roped or carried by its ground.</summary>
    private bool HasBusyCharacters => _charExt != null && _charExt.ActiveCount > 0;

    // ------------------------------------------------------------------ poses

    private void BodyOrigin(int b, out Vector3D origin, out QuaternionF q)
    {
        var p = Core.Bodies.PositionOf(b);
        q = RotationOfBody(b);
        var off = q.Rotate(LocalCenterOf(b));
        origin = new Vector3D(p[0] - (double)off.X, p[1] - (double)off.Y, p[2] - (double)off.Z);
    }

    /// <summary>Puts a character's kinematic body at a pose with a velocity, without recording and without waking anything (the same placement as a teleport).</summary>
    private void PlaceCharacter(int c, Vector3D position, QuaternionF rotation, Vector3F velocity)
    {
        int b = CharBody[c];
        var bodies = Core.Bodies;
        CancelKinematicDrive(b);
        var r = bodies.RotationOf(b);
        r[0] = rotation.X; r[1] = rotation.Y; r[2] = rotation.Z; r[3] = rotation.W;
        var off = rotation.Rotate(LocalCenterOf(b));
        var p = bodies.PositionOf(b);
        p[0] = WorldCore.Grid(position.X) + (double)off.X; p[1] = WorldCore.Grid(position.Y) + (double)off.Y; p[2] = WorldCore.Grid(position.Z) + (double)off.Z;
        Dp[3 * b] = 0f; Dp[3 * b + 1] = 0f; Dp[3 * b + 2] = 0f;
        for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh])
        {
            Bp.MoveProxy(Proxy[sh], TightBox(b, sh), 0.0, 0.0, 0.0);
        }

        var lin = bodies.LinearVelocityOf(b);
        lin[0] = velocity.X; lin[1] = velocity.Y; lin[2] = velocity.Z;
        CharVx[c] = velocity.X; CharVy[c] = velocity.Y; CharVz[c] = velocity.Z;
    }

    private QuaternionF UprightOf(int c) => AlignY(UnitUp(CharDef[c].Up));

    private float GripHeight(int c) => 0.5f * CharDef[c].Height - 0.2f;

    private Vector3D CharacterPosition(int c)
    {
        int b = CharBody[c];
        var p = Core.Bodies.PositionOf(b);
        var off = RotationOfBody(b).Rotate(LocalCenterOf(b));
        return new Vector3D(p[0] - (double)off.X, p[1] - (double)off.Y, p[2] - (double)off.Z);
    }

    private void MarkInAir(int c)
    {
        CharGround[c] = GroundState.InAir; CharNx[c] = 0f; CharNy[c] = 0f; CharNz[c] = 0f; CharGroundBody[c] = -1; CharGroundGen[c] = 0;
    }

    // ------------------------------------------------------------------ the step hooks

    /// <summary>Before the rods begin their step: a gripped rod carries the weight of whoever holds it.</summary>
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

        BodyOrigin(st.Body, out var origin, out var q);
        var local = q.Rotate(st.Local);
        var pos = new Vector3D(origin.X + (double)local.X, origin.Y + (double)local.Y, origin.Z + (double)local.Z);
        var vel = BodyVelocityAt(st.Body, pos.X, pos.Y, pos.Z);
        PlaceCharacter(c, pos, UprightOf(c), vel);
    }

    private void SyncAnchored(int c, ref CharacterExtState st)
    {
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { DropHold(c, ref st); return; }
        BodyOrigin(st.Body, out var origin, out var q);
        var local = q.Rotate(st.Local);
        var pos = new Vector3D(origin.X + (double)local.X, origin.Y + (double)local.Y, origin.Z + (double)local.Z);
        var vel = BodyVelocityAt(st.Body, pos.X, pos.Y, pos.Z);
        PlaceCharacter(c, pos, st.Mode == CharacterMode.Seated ? q * st.LocalRotation : UprightOf(c), vel);
    }

    private void SyncRope(int c, ref CharacterExtState st)
    {
        if (!Rods.Alive(st.Rod, st.RodGen)) { DropHold(c, ref st); return; }
        var d = Rods.Data[st.Rod]!;
        if (st.Particle + 1 >= d.PositionData.Length) { DropHold(c, ref st); return; }
        GripPoint(d, st.Particle, st.Fraction, out var grip, out var vel);
        var up = UnitUp(CharDef[c].Up) * GripHeight(c);
        PlaceCharacter(c, new Vector3D(grip.X - (double)up.X, grip.Y - (double)up.Y, grip.Z - (double)up.Z), UprightOf(c), vel);
    }

    /// <summary>The hold ended on its own (the body or rod is gone): the character falls free with the velocity it had.</summary>
    private void DropHold(int c, ref CharacterExtState st)
    {
        bool roped = st.Mode == CharacterMode.Roped;
        int rod = st.Rod, gen = st.RodGen;
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        if (roped && Rods.Alive(rod, gen)) { RefreshGripMasses(rod); }
        PlaceCharacter(c, CharacterPosition(c), UprightOf(c), new Vector3F(CharVx[c], CharVy[c], CharVz[c]));
    }

    // ------------------------------------------------------------------ seats

    public void MountCharacter(int c, int seatBody, Vector3F local, QuaternionF localRotation)
    {
        RequireVirtual(c);
        ref var st = ref Extensions.At(c);
        if (st.Mode == CharacterMode.Roped) { ReleaseCharacter(c); }
        var next = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, Mode = CharacterMode.Seated, Body = seatBody, BodyGen = Core.Bodies.Handles.GenerationOf(seatBody), Local = local, LocalRotation = localRotation.LengthSquared() == 0f ? QuaternionF.Identity : Normalize(localRotation, nameof(localRotation)) };
        _charExt!.Set(c, in next);
        SyncAnchored(c, ref _charExt.States[c]);
        MarkInAir(c);
    }

    private void RequireVirtual(int c)
    {
        if (CharDef[c].Kind != CharacterKind.Virtual) { throw new InvalidOperationException("Seats, ledges, climbing and rope grips need a virtual character; a rigid character is a dynamic body the solver moves."); }
    }

    /// <summary>Lets go of whatever the character holds or sits in; returns the velocity it leaves with (the seat's, the rope's, or zero).</summary>
    public Vector3F ReleaseCharacter(int c)
    {
        var ext = Extensions;
        ref var st = ref ext.At(c);
        if (st.Mode == CharacterMode.Free) { return default; }
        Vector3F vel = default;
        var pos = CharacterPosition(c);
        int rod = st.Rod, gen = st.RodGen;
        bool roped = st.Mode == CharacterMode.Roped;
        if (st.Mode == CharacterMode.Seated || st.Mode == CharacterMode.Roped) { vel = new Vector3F(CharVx[c], CharVy[c], CharVz[c]); }
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry };
        ext.Set(c, in keep);
        MarkInAir(c);
        if (roped && Rods.Alive(rod, gen)) { RefreshGripMasses(rod); }
        PlaceCharacter(c, pos, UprightOf(c), vel);
        return vel;
    }

    // ------------------------------------------------------------------ ropes

    private static void GripPoint(RodParticles d, int particle, float fraction, out Vector3D position, out Vector3F velocity)
    {
        var a = d.PositionData[particle]; var b = d.PositionData[particle + 1];
        position = new Vector3D(a.X + (b.X - a.X) * fraction, a.Y + (b.Y - a.Y) * fraction, a.Z + (b.Z - a.Z) * fraction);
        var va = d.VelocityData[particle]; var vb = d.VelocityData[particle + 1];
        velocity = new Vector3F(va.X + (vb.X - va.X) * fraction, va.Y + (vb.Y - va.Y) * fraction, va.Z + (vb.Z - va.Z) * fraction);
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
        RequireVirtual(c);
        var d = Rods.Data[rod]!;
        int n = d.PositionData.Length;
        if (!(position >= 0f && position <= n - 1)) { throw new ArgumentOutOfRangeException(nameof(position), position, "The grip position must be within [0, " + (n - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + "] (particle index units)."); }
        ref var cur = ref Extensions.At(c);
        int prevRod = cur.Mode == CharacterMode.Roped ? cur.Rod : -1, prevGen = cur.RodGen;
        int i = Math.Min((int)position, n - 2);
        var st = new CharacterExtState { CanClimb = cur.CanClimb, ExactCarry = cur.ExactCarry, Mode = CharacterMode.Roped, Rod = rod, RodGen = generation, Particle = i, Fraction = position - i, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in st);
        RefreshGripMasses(rod);
        if (prevRod >= 0 && (prevRod != rod) && Rods.Alive(prevRod, prevGen)) { RefreshGripMasses(prevRod); }
        MarkInAir(c);
        // The character keeps the velocity it had: the grip takes it over as the rope's own, one step later; set the pose now.
        SyncRope(c, ref _charExt.States[c]);
    }

    /// <summary>Finds the point of a rod nearest to the character's hands and grips it when it is within <paramref name="reach"/>.</summary>
    public bool TryGrabRod(int c, int rod, int generation, float reach)
    {
        RequireVirtual(c);
        var d = Rods.Data[rod]!;
        var up = UnitUp(CharDef[c].Up);
        var hand = CharacterPosition(c);
        var h = new Vector3D(hand.X + (double)up.X * GripHeight(c), hand.Y + (double)up.Y * GripHeight(c), hand.Z + (double)up.Z * GripHeight(c));
        double best = double.MaxValue; float bestS = 0f;
        for (int j = 0; j + 1 < d.PositionData.Length; j++)
        {
            var a = d.PositionData[j]; var b = d.PositionData[j + 1];
            double ex = b.X - a.X, ey = b.Y - a.Y, ez = b.Z - a.Z, l2 = ex * ex + ey * ey + ez * ez;
            double t = l2 > 0.0 ? CharacterGeometry.Clamp01(((h.X - a.X) * ex + (h.Y - a.Y) * ey + (h.Z - a.Z) * ez) / l2) : 0.0;
            double dx = a.X + ex * t - h.X, dy = a.Y + ey * t - h.Y, dz = a.Z + ez * t - h.Z;
            double dist = dx * dx + dy * dy + dz * dz;
            if (dist < best) { best = dist; bestS = j + (float)t; }
        }

        if (!(Math.Sqrt(best) <= reach + d.Material.Radius)) { return false; }
        GrabRod(c, rod, generation, bestS);
        return true;
    }

    /// <summary>Moves a rope grip along the rod by <paramref name="distance"/> metres (positive goes up, towards the end of the rod that is higher against the character's up direction).</summary>
    public void ClimbRope(int c, float distance)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode != CharacterMode.Roped) { throw new InvalidOperationException("The character is not holding a rod."); }
        if (!Rods.Alive(st.Rod, st.RodGen)) { return; }
        var d = Rods.Data[st.Rod]!;
        int n = d.PositionData.Length;
        var a = d.PositionData[st.Particle]; var b = d.PositionData[st.Particle + 1];
        var up = UnitUp(CharDef[c].Up);
        double len = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y) + (b.Z - a.Z) * (b.Z - a.Z));
        if (!(len > 1e-9)) { return; }
        double along = ((b.X - a.X) * up.X + (b.Y - a.Y) * up.Y + (b.Z - a.Z) * up.Z) / len;
        double sign = along >= -1e-6 ? 1.0 : -1.0;
        double s = st.Particle + (double)st.Fraction + sign * distance / len;
        s = s < 0.0 ? 0.0 : s > n - 1 ? n - 1 : s;
        int i = Math.Min((int)s, n - 2);
        st.Particle = i; st.Fraction = (float)(s - i);
        RefreshGripMasses(st.Rod);
        SyncRope(c, ref st);
    }

    // ------------------------------------------------------------------ ledges and climbable surfaces

    private int _ledgeIgnore = -1;
    private Func<Shape, bool>? _ledgePredicate;

    private QueryFilter3D LedgeFilter(int ignoreBody, uint layerMask)
    {
        _ledgePredicate ??= sh => { int o = Core.Shapes.Owner[sh.Handle.Index]; return o != _ledgeIgnore && !IsCharacterBody(o); };
        _ledgeIgnore = ignoreBody;
        return new QueryFilter3D { Predicate = _ledgePredicate, LayerMask = layerMask };
    }

    private struct Ledge
    {
        public int Body, BodyGen, Shape, ShapeGen;
        public Vector3D Top;          // where the hands go
        public Vector3F Normal;       // wall normal, towards the character
    }

    private static Vector3F Horizontal(Vector3F v, Vector3F up)
    {
        var h = v - up * Vector3F.Dot(v, up);
        float l2 = h.LengthSquared();
        if (!(l2 > 1e-10f) || !float.IsFinite(l2)) { throw new ArgumentOutOfRangeException(nameof(v), v, "The facing direction must be finite and not along the character's up direction."); }
        return h / float.Sqrt(l2);
    }

    /// <summary>
    /// Looks for a ledge in front of a character at <paramref name="centre"/>: a ray at chest height along <paramref name="facing"/> finds the wall, a ray straight down from just above the hands, a little
    /// inside the wall, finds the top. The wall must be steep, the top flat enough to stand on, and the top within a hand's reach (0.3 m below to 0.4 m above the hands). Works on every shape the rays do: boxes,
    /// meshes, height fields, voxel grids.
    /// </summary>
    private bool FindLedge(int c, Vector3D centre, Vector3F facing, out Ledge ledge)
    {
        ledge = default;
        var def = CharDef[c];
        var up = UnitUp(def.Up);
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        float handUp = 0.5f * def.Height - 0.1f;
        var chest = new Vector3D(centre.X + (double)up.X * (handUp - 0.3f), centre.Y + (double)up.Y * (handUp - 0.3f), centre.Z + (double)up.Z * (handUp - 0.3f));
        float reach = def.Radius + 0.4f;
        if (!World!.Raycast(chest, facing, out var wall, reach, in filter)) { return false; }
        var nw = wall.Normal;
        if (!(wall.Distance > 0f) || Math.Abs(Vector3F.Dot(nw, up)) > 0.5f || Vector3F.Dot(nw, facing) > -0.5f) { return false; }
        // Straight down onto the top from half a metre above the hands, a hand's width inside the wall.
        double above = (double)(handUp + 0.5f);
        var o = new Vector3D(wall.Point.X - (double)nw.X * 0.1 + (double)up.X * (above - (handUp - 0.3f)), wall.Point.Y - (double)nw.Y * 0.1 + (double)up.Y * (above - (handUp - 0.3f)), wall.Point.Z - (double)nw.Z * 0.1 + (double)up.Z * (above - (handUp - 0.3f)));
        if (!World.Raycast(o, up * -1f, out var top, 1.1f, in filter)) { return false; }
        if (!(top.Distance > 0.02f) || Vector3F.Dot(top.Normal, up) < 0.7f) { return false; }
        // The hands go at the wall's edge, so the top point is slid back along the wall's normal onto the wall plane.
        float inset = Vector3F.Dot(new Vector3F((float)(top.Point.X - wall.Point.X), (float)(top.Point.Y - wall.Point.Y), (float)(top.Point.Z - wall.Point.Z)), nw);
        ledge = new Ledge
        {
            Body = top.Body.RawHandle.Index, BodyGen = top.Body.RawHandle.Generation, Shape = top.Shape.Handle.Index, ShapeGen = top.Shape.Handle.Generation,
            Top = new Vector3D(top.Point.X - (double)nw.X * inset, top.Point.Y - (double)nw.Y * inset, top.Point.Z - (double)nw.Z * inset), Normal = nw,
        };
        return true;
    }

    private Vector3D HangCentre(int c, in Ledge l)
    {
        var def = CharDef[c];
        var up = UnitUp(def.Up);
        float drop = 0.5f * def.Height - 0.1f;
        float out_ = def.Radius + 0.02f;
        return new Vector3D(l.Top.X + (double)(l.Normal.X * out_ - up.X * drop), l.Top.Y + (double)(l.Normal.Y * out_ - up.Y * drop), l.Top.Z + (double)(l.Normal.Z * out_ - up.Z * drop));
    }

    private void EnterHold(int c, CharacterMode mode, int body, Vector3D centre, Vector3F normal, Vector3F along)
    {
        ref var cur = ref Extensions.At(c);
        BodyOrigin(body, out var origin, out var q);
        var inv = q.Conjugated();
        var local = inv.Rotate(new Vector3F((float)(centre.X - origin.X), (float)(centre.Y - origin.Y), (float)(centre.Z - origin.Z)));
        var st = new CharacterExtState
        {
            CanClimb = cur.CanClimb, ExactCarry = cur.ExactCarry, Mode = mode, Body = body, BodyGen = Core.Bodies.Handles.GenerationOf(body), Local = local,
            LocalRotation = QuaternionF.Identity, Normal = inv.Rotate(normal), Along = inv.Rotate(along),
        };
        if (cur.Mode == CharacterMode.Roped) { ReleaseCharacter(c); }
        _charExt!.Set(c, in st);
        MarkInAir(c);
        SyncAnchored(c, ref _charExt.States[c]);
        CharVx[c] = 0f; CharVy[c] = 0f; CharVz[c] = 0f;
        var lin = Core.Bodies.LinearVelocityOf(CharBody[c]);
        lin[0] = 0f; lin[1] = 0f; lin[2] = 0f;
    }

    public bool TryGrabLedge(int c, Vector3F facing)
    {
        RequireVirtual(c);
        if (!Extensions.At(c).CanClimb) { return false; }
        var up = UnitUp(CharDef[c].Up);
        var f = Horizontal(facing, up);
        if (!FindLedge(c, CharacterPosition(c), f, out var ledge)) { return false; }
        var along = Vector3F.Cross(up, ledge.Normal).Normalized();
        EnterHold(c, CharacterMode.Hanging, ledge.Body, HangCentre(c, in ledge), ledge.Normal, along);
        return true;
    }

    public bool TryGrabSurface(int c, Vector3F facing)
    {
        RequireVirtual(c);
        if (!Extensions.At(c).CanClimb) { return false; }
        var def = CharDef[c];
        var up = UnitUp(def.Up);
        var f = Horizontal(facing, up);
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        var pos = CharacterPosition(c);
        if (!World!.Raycast(pos, f, out var hit, def.Radius + 0.4f, in filter)) { return false; }
        if (!(hit.Distance >= 0f) || Math.Abs(Vector3F.Dot(hit.Normal, up)) > 0.5f || Vector3F.Dot(hit.Normal, f) > -0.5f) { return false; }
        if (!Extensions.IsClimbable(hit.Shape.Handle.Index, hit.Shape.Handle.Generation)) { return false; }
        var nw = hit.Normal;
        float off = def.Radius + 0.02f;
        var centre = new Vector3D(hit.Point.X + (double)nw.X * off, hit.Point.Y + (double)nw.Y * off, hit.Point.Z + (double)nw.Z * off);
        // Keep the height the character has; only the distance to the surface changes.
        double lift = Vector3F.Dot(new Vector3F((float)(pos.X - centre.X), (float)(pos.Y - centre.Y), (float)(pos.Z - centre.Z)), up);
        centre = new Vector3D(centre.X + (double)up.X * lift, centre.Y + (double)up.Y * lift, centre.Z + (double)up.Z * lift);
        EnterHold(c, CharacterMode.Climbing, hit.Body.RawHandle.Index, centre, nw, default);
        return true;
    }

    /// <summary>The held surface's current normal and the character's centre, in world coordinates.</summary>
    private void HoldFrame(int c, in CharacterExtState st, out Vector3D centre, out Vector3F normal, out Vector3F along)
    {
        BodyOrigin(st.Body, out var origin, out var q);
        var l = q.Rotate(st.Local);
        centre = new Vector3D(origin.X + (double)l.X, origin.Y + (double)l.Y, origin.Z + (double)l.Z);
        normal = q.Rotate(st.Normal);
        along = q.Rotate(st.Along);
        _ = c;
    }

    /// <summary>Pulls a hanging character over the ledge onto its top, when the capsule fits there.</summary>
    public bool ClimbUpLedge(int c)
    {
        var ext = Extensions;
        ref var st = ref ext.At(c);
        if (st.Mode != CharacterMode.Hanging) { throw new InvalidOperationException("The character is not hanging from a ledge."); }
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { return false; }
        HoldFrame(c, in st, out var centre, out var nw, out _);
        return MantleOnto(c, centre, nw);
    }

    private bool MantleOnto(int c, Vector3D centre, Vector3F nw)
    {
        var def = CharDef[c];
        var up = UnitUp(def.Up);
        var f = nw * -1f;
        if (!FindLedge(c, centre, f, out var ledge)) { return false; }
        float inward = def.Radius + 0.05f;
        float lift = 0.5f * def.Height + def.Skin + 0.01f;
        var target = new Vector3D(ledge.Top.X + (double)(f.X * inward + up.X * lift), ledge.Top.Y + (double)(f.Y * inward + up.Y * lift), ledge.Top.Z + (double)(f.Z * inward + up.Z * lift));
        if (!CapsuleFitsAt(c, target)) { return false; }
        ref var st = ref Extensions.At(c);
        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        PlaceCharacter(c, target, UprightOf(c), default);
        return true;
    }

    /// <summary>True when the character's capsule at <paramref name="p"/> overlaps nothing deeper than a skin.</summary>
    private bool CapsuleFitsAt(int c, Vector3D p)
    {
        var def = CharDef[c];
        var ctx = MakeMoverCtx(in def, CharBody[c]);
        ctx.Range = 0f;
        MvGather(MvBox(p, default, in ctx, 0.01f), in ctx);
        MvPlanes(p, in ctx);
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (_mvPlanes[i].Separation < -0.5f * def.Skin - 0.01f && !IsCharacterBody(MvOwner(_mvMeta[i].Slot))) { return false; }
        }

        return true;
    }

    /// <summary>The moves of a character that is not walking: shimmy along a ledge, climb a surface, climb a rope. Returns the contact flags (always none).</summary>
    public CharacterContact MoveBusy(int c, Vector3F displacement, float dt)
    {
        ref var st = ref Extensions.At(c);
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        if (!float.IsFinite(displacement.X) || !float.IsFinite(displacement.Y) || !float.IsFinite(displacement.Z) || !(displacement.LengthSquared() <= 1e10f))
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The displacement must be finite and at most 100000 long.");
        }

        var up = UnitUp(CharDef[c].Up);
        switch (st.Mode)
        {
            case CharacterMode.Roped:
            {
                ClimbRope(c, Vector3F.Dot(displacement, up));
                break;
            }
            case CharacterMode.Hanging:
            {
                if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { DropHold(c, ref st); break; }
                HoldFrame(c, in st, out var centre, out var nw, out var along);
                float s = Vector3F.Dot(displacement, along);
                if (s == 0f) { break; }
                var cand = new Vector3D(centre.X + (double)along.X * s, centre.Y + (double)along.Y * s, centre.Z + (double)along.Z * s);
                if (FindLedge(c, cand, nw * -1f, out var ledge) && ledge.Body == st.Body && ledge.BodyGen == st.BodyGen)
                {
                    var next = HangCentre(c, in ledge);
                    float rise = Vector3F.Dot(new Vector3F((float)(next.X - centre.X), (float)(next.Y - centre.Y), (float)(next.Z - centre.Z)), up);
                    if (Math.Abs(rise) <= 0.25f) { EnterHold(c, CharacterMode.Hanging, ledge.Body, next, ledge.Normal, Vector3F.Cross(up, ledge.Normal).Normalized()); }
                }

                break;
            }
            case CharacterMode.Climbing:
            {
                MoveClimbing(c, displacement, up);
                break;
            }
        }

        return CharacterContact.None;
    }

    private void MoveClimbing(int c, Vector3F displacement, Vector3F up)
    {
        var def = CharDef[c];
        ref var st = ref Extensions.At(c);
        if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { DropHold(c, ref st); return; }
        HoldFrame(c, in st, out var centre, out var nw, out _);
        var tg = Vector3F.Cross(up, nw);
        float tl = tg.Length();
        tg = tl > 1e-6f ? tg / tl : default;
        float v = Vector3F.Dot(displacement, up), s = Vector3F.Dot(displacement, tg);
        if (v == 0f && s == 0f) { return; }
        var cand = new Vector3D(centre.X + (double)(up.X * v + tg.X * s), centre.Y + (double)(up.Y * v + tg.Y * s), centre.Z + (double)(up.Z * v + tg.Z * s));
        var filter = LedgeFilter(CharBody[c], def.LayerMask);
        if (v < 0f)
        {
            // Climbing down onto the ground: stand on it.
            var feet = new Vector3D(centre.X - (double)up.X * (0.5f * def.Height - 0.05f), centre.Y - (double)up.Y * (0.5f * def.Height - 0.05f), centre.Z - (double)up.Z * (0.5f * def.Height - 0.05f));
            if (World!.Raycast(feet, up * -1f, out var ground, 0.05f - v + 0.02f, in filter) && Vector3F.Dot(ground.Normal, up) >= 0.7f && ground.Distance > 0f)
            {
                ReleaseCharacter(c);
                return;
            }
        }

        // The surface is looked for at the hands: the climb ends when they pass the top.
        float handUp = 0.5f * def.Height - 0.1f;
        var from = new Vector3D(cand.X + (double)(nw.X * 0.5f + up.X * handUp), cand.Y + (double)(nw.Y * 0.5f + up.Y * handUp), cand.Z + (double)(nw.Z * 0.5f + up.Z * handUp));
        if (World!.Raycast(from, nw * -1f, out var hit, def.Radius + 0.9f, in filter) && hit.Distance > 0f && Vector3F.Dot(hit.Normal, nw) >= 0.7f
            && Extensions.IsClimbable(hit.Shape.Handle.Index, hit.Shape.Handle.Generation) && hit.Body.RawHandle.Index == st.Body)
        {
            float off = def.Radius + 0.02f;
            // Slide along the surface: the hit point is at hand height, so it is brought back down to the candidate's height along the surface.
            var next = new Vector3D(hit.Point.X + (double)hit.Normal.X * off - (double)up.X * handUp, hit.Point.Y + (double)hit.Normal.Y * off - (double)up.Y * handUp, hit.Point.Z + (double)hit.Normal.Z * off - (double)up.Z * handUp);
            EnterHold(c, CharacterMode.Climbing, st.Body, next, hit.Normal, default);
            return;
        }

        if (v > 0f)
        {
            // The surface ends above: climb over the top when there is a ledge to stand on.
            MantleOnto(c, cand, nw);
        }
    }

    /// <summary>A jump off the held surface: lets go and returns a velocity away from it, <paramref name="speed"/> along its normal and <paramref name="upSpeed"/> up.</summary>
    public Vector3F WallJump(int c, float speed, float upSpeed)
    {
        ref var st = ref Extensions.At(c);
        if (st.Mode != CharacterMode.Hanging && st.Mode != CharacterMode.Climbing) { throw new InvalidOperationException("The character is not hanging from a ledge or climbing a surface."); }
        HoldFrame(c, in st, out _, out var nw, out _);
        ReleaseCharacter(c);
        var v = nw * speed + UnitUp(CharDef[c].Up) * upSpeed;
        CharVx[c] = v.X; CharVy[c] = v.Y; CharVz[c] = v.Z;
        var lin = Core.Bodies.LinearVelocityOf(CharBody[c]);
        lin[0] = v.X; lin[1] = v.Y; lin[2] = v.Z;
        return v;
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
                GripPoint(Rods.Data[st.Rod]!, st.Particle, st.Fraction, out point, out _);
            }
            else
            {
                if (!Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen)) { continue; }
                BodyOrigin(st.Body, out var origin, out var q);
                var l = q.Rotate(st.Local);
                point = new Vector3D(origin.X + (double)l.X, origin.Y + (double)l.Y, origin.Z + (double)l.Z);
                if (st.Mode != CharacterMode.Seated) { normal = q.Rotate(st.Normal); }
                held = Core.HandleOfBody(st.Body);
            }

            g.Add(new Debug.DebugCharacterHold(Core.HandleOfBody(CharBody[c]), (byte)st.Mode, held, point, normal));
        }
    }
}

internal sealed partial class Engine3D
{
    public bool CharacterIsBusy(int c) => _charExt != null && c < _charExt.States.Length && _charExt.States[c].Mode != CharacterMode.Free;

    public bool CharacterCarriesByPose(int c) => _charExt != null && c < _charExt.States.Length && _charExt.States[c].ExactCarry;

    /// <summary>After a move of a pose-carried character: remember where it stands in the frame of the moving body under it, so the step can take it along exactly.</summary>
    public void UpdateCarried(int c, Vector3D position, Body ground)
    {
        var st = Extensions.At(c);
        if (ground.World == null || !ground.IsValid || ground.Type == BodyType.Static)
        {
            if (st.Riding) { st.Riding = false; _charExt!.Set(c, in st); }
            return;
        }

        int gb = ground.RawHandle.Index;
        BodyOrigin(gb, out var origin, out var q);
        st.Riding = true;
        st.Body = gb; st.BodyGen = ground.RawHandle.Generation;
        st.Local = q.Conjugated().Rotate(new Vector3F((float)(position.X - origin.X), (float)(position.Y - origin.Y), (float)(position.Z - origin.Z)));
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
        CharVx[c] = 0f; CharVy[c] = 0f; CharVz[c] = 0f;
    }
}

internal sealed partial class Engine3D
{
    public CharacterMode ModeOf(int c) => _charExt != null && c < _charExt.States.Length ? _charExt.States[c].Mode : CharacterMode.Free;

    /// <summary>Gets up from a seat: the character keeps the seat point's velocity and steps out by <paramref name="exitOffset"/> in the body's frame.</summary>
    public Vector3F DismountCharacter(int c, Vector3F exitOffset)
    {
        var st = Extensions.At(c);
        var vel = new Vector3F(CharVx[c], CharVy[c], CharVz[c]);
        var pos = CharacterPosition(c);
        if ((exitOffset.X != 0f || exitOffset.Y != 0f || exitOffset.Z != 0f) && Core.Bodies.Handles.IsAlive(st.Body, st.BodyGen))
        {
            BodyOrigin(st.Body, out _, out var q);
            var o = q.Rotate(exitOffset);
            pos = new Vector3D(pos.X + (double)o.X, pos.Y + (double)o.Y, pos.Z + (double)o.Z);
        }

        var keep = new CharacterExtState { CanClimb = st.CanClimb, ExactCarry = st.ExactCarry, LocalRotation = QuaternionF.Identity };
        _charExt!.Set(c, in keep);
        MarkInAir(c);
        PlaceCharacter(c, pos, UprightOf(c), vel);
        return vel;
    }
}

internal sealed partial class Engine3D
{
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
}
