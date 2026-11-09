using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Soft bodies and rods as ground and obstacles for the virtual mover. Their surfaces are spheres (soft particles) and capsules (rod segments);
    // they enter the plane list like shapes do, with negative slots, and the weight and pushes of the character go into their particles as velocity changes.

    private struct DefContact
    {
        public int Slot, Index;
        public float Along, Sep, Nx, Ny, Nz;
    }

    private DefContact[] _mvDef = new DefContact[32];
    private int _mvDefCount;

    private const int MvRodBase = 1 << 20;

    private static int MvSoftSlot(int body) => -1 - body;

    private static int MvRodSlot(int rod) => -1 - MvRodBase - rod;

    /// <summary>The rigid body that owns a plane's shape, or -1 for a soft body or rod.</summary>
    private int MvOwner(int slot) => slot >= 0 ? Core.Shapes.Owner[slot] : -1;

    /// <summary>True when soft bodies or rods exist; refreshes the soft bodies' bounds, which the plane search reads.</summary>
    private bool MvPrepareDeformables()
    {
        bool any = false;
        if (SoftHandles.Count > 0)
        {
            for (int i = 0; i < SoftHandles.HighWater; i++)
            {
                if (SoftHandles.IsLive(i)) { ComputeSoftBounds(SoftData[i]!); any = true; }
            }
        }

        return any || Rods.Count > 0;
    }

    private void MvAddDefContact(int slot, int index, float along, float sep, Vector3F n)
    {
        if (_mvDefCount == _mvDef.Length) { Array.Resize(ref _mvDef, _mvDefCount * 2); }
        _mvDef[_mvDefCount++] = new DefContact { Slot = slot, Index = index, Along = along, Sep = sep, Nx = n.X, Ny = n.Y, Nz = n.Z };
    }

    private void MvDeformablePlanes(Vector3D p, in MoverCtx c)
    {
        double half = c.HalfLength;
        // The capsule's axis, relative to p.
        var a0 = new Vector3D(-(double)c.Up.X * half, -(double)c.Up.Y * half, -(double)c.Up.Z * half);
        var a1 = new Vector3D((double)c.Up.X * half, (double)c.Up.Y * half, (double)c.Up.Z * half);
        double reach = c.Radius + c.Range + 1e-3;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            double rp = d.Radius;
            double r = reach + rp;
            double ex = Math.Abs(c.Up.X) * half + r, ey = Math.Abs(c.Up.Y) * half + r, ez = Math.Abs(c.Up.Z) * half + r;
            if (d.MaxX < p.X - ex || d.MinX > p.X + ex || d.MaxY < p.Y - ey || d.MinY > p.Y + ey || d.MaxZ < p.Z - ez || d.MinZ > p.Z + ez) { continue; }
            for (int k = 0; k < d.N; k++)
            {
                double qx = d.Pos[3 * k] - p.X, qy = d.Pos[3 * k + 1] - p.Y, qz = d.Pos[3 * k + 2] - p.Z;
                if (Math.Abs(qx) > ex || Math.Abs(qy) > ey || Math.Abs(qz) > ez) { continue; }
                // Closest point of the axis segment to the particle centre.
                double t = qx * c.Up.X + qy * c.Up.Y + qz * c.Up.Z;
                t = t < -half ? -half : t > half ? half : t;
                double dx = (double)c.Up.X * t - qx, dy = (double)c.Up.Y * t - qy, dz = (double)c.Up.Z * t - qz;
                double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                float sep = (float)(dist - (c.Radius + rp));
                if (sep > c.Range) { continue; }
                // Normal from the particle towards the capsule; a particle exactly on the axis is pushed out upwards.
                Vector3F n = dist > 1e-9 ? new Vector3F((float)(dx / dist), (float)(dy / dist), (float)(dz / dist)) : c.Up;
                MvAddDefContact(MvSoftSlot(i), k, 0f, sep, n);
                MvAddPlane(n, sep, p.X + qx + (double)n.X * rp, p.Y + qy + (double)n.Y * rp, p.Z + qz + (double)n.Z * rp, MvSoftSlot(i), in c, k);
            }
        }

        for (int rod = 0; rod < Rods.Data.Length; rod++)
        {
            var d = Rods.Data[rod];
            if (d == null) { continue; }
            double rr = d.Material.Radius;
            double r = reach + rr;
            double ex = Math.Abs(c.Up.X) * half + r, ey = Math.Abs(c.Up.Y) * half + r, ez = Math.Abs(c.Up.Z) * half + r;
            for (int j = 0; j + 1 < d.PositionData.Length; j++)
            {
                var pa = d.PositionData[j]; var pb = d.PositionData[j + 1];
                double ax = pa.X - p.X, ay = pa.Y - p.Y, az = pa.Z - p.Z, bx = pb.X - p.X, by = pb.Y - p.Y, bz = pb.Z - p.Z;
                if ((ax > ex && bx > ex) || (ax < -ex && bx < -ex) || (ay > ey && by > ey) || (ay < -ey && by < -ey) || (az > ez && bz > ez) || (az < -ez && bz < -ez)) { continue; }
                SegmentSegment(a0, a1, new Vector3D(ax, ay, az), new Vector3D(bx, by, bz), out _, out double tt, out Vector3D c1, out Vector3D c2);
                double dx = c1.X - c2.X, dy = c1.Y - c2.Y, dz = c1.Z - c2.Z;
                double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                float sep = (float)(dist - (c.Radius + rr));
                if (sep > c.Range) { continue; }
                Vector3F n = dist > 1e-9 ? new Vector3F((float)(dx / dist), (float)(dy / dist), (float)(dz / dist)) : c.Up;
                MvAddDefContact(MvRodSlot(rod), j, (float)tt, sep, n);
                MvAddPlane(n, sep, p.X + c2.X + (double)n.X * rr, p.Y + c2.Y + (double)n.Y * rr, p.Z + c2.Z + (double)n.Z * rr, MvRodSlot(rod), in c, j, (float)tt);
            }
        }
    }

    /// <summary>Closest points of two segments (Ericson, Real-Time Collision Detection, 5.1.9): parameters s and t in [0,1] and the points.</summary>
    internal static void SegmentSegment(Vector3D p1, Vector3D q1, Vector3D p2, Vector3D q2, out double s, out double t, out Vector3D c1, out Vector3D c2)
    {
        var d1 = q1 - p1; var d2 = q2 - p2; var r = p1 - p2;
        double a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r);
        const double Eps = 1e-12;
        if (a <= Eps && e <= Eps) { s = t = 0.0; }
        else if (a <= Eps) { s = 0.0; t = Clamp01(f / e); }
        else
        {
            double cc = Dot(d1, r);
            if (e <= Eps) { t = 0.0; s = Clamp01(-cc / a); }
            else
            {
                double b = Dot(d1, d2), denom = a * e - b * b;
                s = denom > Eps * a * e ? Clamp01((b * f - cc * e) / denom) : 0.0;
                t = (b * s + f) / e;
                if (t < 0.0) { t = 0.0; s = Clamp01(-cc / a); }
                else if (t > 1.0) { t = 1.0; s = Clamp01((b - cc) / a); }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
    }

    private static double Dot(Vector3D a, Vector3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    private static double Clamp01(double v) => v < 0.0 ? 0.0 : v > 1.0 ? 1.0 : v;

    private static bool IsSupportContact(in DefContact k, in MoverCtx c) => k.Sep <= 2f * c.Skin + 1e-3f && k.Nx * c.Up.X + k.Ny * c.Up.Y + k.Nz * c.Up.Z >= c.CosSlope;

    /// <summary>A character standing on a soft body wakes it: the weight is added to the particles under its feet during the step (see <see cref="ApplySupportMass"/>).</summary>
    private void MvWakeDeformables(in MoverCtx c)
    {
        for (int i = 0; i < _mvDefCount; i++)
        {
            ref DefContact k = ref _mvDef[i];
            if (!IsSupportContact(in k, in c)) { continue; }
            int v = -1 - k.Slot;
            if (v < MvRodBase) { var d = SoftData[v]!; d.Asleep = false; d.SleepTime = 0f; }
        }
    }

    // The support masses of the step: a character standing on soft particles or a rod is carried by them, so for the step its mass is added to the particles under its feet, shared equally
    // among the contacts. The solver then holds the weight with its own constraints (momentum balances, gravity pulls the extra mass down, the character follows the support at its next move);
    // no impulse is invented. The masses go back after the step, so snapshots never hold them.
    private int[] _supSoftBody = new int[16], _supSoftParticle = new int[16];
    private int _supSoftCount;
    private float _supDt;
    private int[] _supRods = new int[4];
    private int _supRodCount;

    /// <summary>Before the soft and rod steps: adds the mass of every character that stands on soft particles or rod segments to the particles under it.</summary>
    private void ApplySupportMass(float dt)
    {
        _supDt = dt;
        _supSoftCount = 0; _supRodCount = 0;
        if (SoftHandles.Count == 0 && Rods.Count == 0) { return; }
        bool prepared = false;
        for (int ch = 0; ch < CharHandles.HighWater; ch++)
        {
            if (!CharHandles.IsLive(ch) || CharDef[ch].Kind != CharacterKind.Virtual || CharGround[ch] != GroundState.OnGround || CharGroundBody[ch] >= 0) { continue; }
            if (CharacterIsBusy(ch)) { continue; }
            if (!prepared) { MvPrepareDeformables(); prepared = true; }
            var def = CharDef[ch];
            var c = MakeMoverCtx(in def, CharBody[ch]);
            c.Range = 3f * c.Skin;
            c.Deformables = true;
            _mvPlaneCount = 0; _mvDefCount = 0;
            MvDeformablePlanes(CharacterPosition(ch), in c);
            int count = 0;
            for (int i = 0; i < _mvDefCount; i++) { if (IsSupportContact(in _mvDef[i], in c)) { count++; } }
            if (count == 0) { continue; }
            float share = def.Mass / count;
            for (int i = 0; i < _mvDefCount; i++)
            {
                ref DefContact k = ref _mvDef[i];
                if (!IsSupportContact(in k, in c)) { continue; }
                int v = -1 - k.Slot;
                if (v < MvRodBase) { AddSoftMass(v, k.Index, share); }
                else { AddRodSupport(v - MvRodBase, k.Index, k.Along, share); }
            }
        }

        ApplyRodSupportMass();
    }

    private void AddSoftMass(int body, int particle, float mass)
    {
        var d = SoftData[body]!;
        if (d.InvMass[particle] == 0f) { return; }
        float own = 1f / d.InvMass[particle];
        d.InvMass[particle] = 1f / (own + mass);
        // The weight is the character's, not the body's: a body with its own gravity scale feels the difference as a velocity change over the step.
        float miss = (1f - d.Def.GravityScale) * (mass / (own + mass)) * _supDt;
        if (miss != 0f) { d.Vel[3 * particle] += Opt.Gravity.X * miss; d.Vel[3 * particle + 1] += Opt.Gravity.Y * miss; d.Vel[3 * particle + 2] += Opt.Gravity.Z * miss; }
        if (_supSoftCount == _supSoftBody.Length) { Array.Resize(ref _supSoftBody, _supSoftCount * 2); Array.Resize(ref _supSoftParticle, _supSoftCount * 2); }
        _supSoftBody[_supSoftCount] = body; _supSoftParticle[_supSoftCount++] = particle;
    }

    private void AddRodSupport(int rod, int segment, float along, float mass)
    {
        int slot = -1;
        for (int i = 0; i < _supRodCount; i++) { if (_supRods[i] == rod) { slot = i; break; } }
        if (slot < 0)
        {
            if (_supRodCount == _supRods.Length) { Array.Resize(ref _supRods, _supRodCount * 2); }
            slot = _supRodCount;
            _supRods[_supRodCount++] = rod;
            _supRodAdds ??= new System.Collections.Generic.List<float[]>();
            while (_supRodAdds.Count <= slot) { _supRodAdds.Add(Array.Empty<float>()); }
            int n = Rods.Data[rod]!.PositionData.Length;
            if (_supRodAdds[slot].Length < n) { _supRodAdds[slot] = new float[n]; } else { Array.Clear(_supRodAdds[slot]); }
        }

        var add = _supRodAdds![slot];
        add[segment] += mass * (1f - along);
        add[segment + 1] += mass * along;
    }

    private System.Collections.Generic.List<float[]>? _supRodAdds;

    private void ApplyRodSupportMass()
    {
        for (int s = 0; s < _supRodCount; s++)
        {
            var d = Rods.Data[_supRods[s]]!;
            var add = _supRodAdds![s];
            for (int p = 0; p < d.PositionData.Length; p++)
            {
                if (!(add[p] > 0f) || d.Pinned[p] || !(d.InverseMass[p] > 0f)) { continue; }
                float own = 1f / d.InverseMass[p];
                d.InverseMass[p] = 1f / (own + add[p]);
                float miss = (1f - d.GravityScale) * (add[p] / (own + add[p])) * _supDt;
                if (miss != 0f) { d.VelocityData[p] += new Vector3F(Opt.Gravity.X * miss, Opt.Gravity.Y * miss, Opt.Gravity.Z * miss); }
            }
        }
    }

    /// <summary>After the soft and rod steps: takes the support masses off again.</summary>
    public void EndCharacterSupport()
    {
        for (int i = 0; i < _supSoftCount; i++)
        {
            var d = SoftData[_supSoftBody[i]];
            d?.RefreshInvMass(_supSoftParticle[i]);
        }

        _supSoftCount = 0;
        for (int i = 0; i < _supRodCount; i++)
        {
            if (Rods.Data[_supRods[i]] != null) { RefreshGripMasses(_supRods[i]); }
        }

        _supRodCount = 0;
    }

    /// <summary>Adds an impulse (newton-seconds) at a soft particle or at a point of a rod segment, as velocity changes by inverse mass; pinned particles take nothing.</summary>
    private void ApplyDeformableImpulse(int slot, int index, float along, Vector3F impulse)
    {
        int v = -1 - slot;
        if (v < MvRodBase)
        {
            var d = SoftData[v]!;
            float inv = d.InvMass[index];
            d.Vel[3 * index] += impulse.X * inv; d.Vel[3 * index + 1] += impulse.Y * inv; d.Vel[3 * index + 2] += impulse.Z * inv;
            d.Asleep = false; d.SleepTime = 0f;
            return;
        }

        var rod = Rods.Data[v - MvRodBase]!;
        float w0 = 1f - along, w1 = along;
        var v0 = rod.VelocityData[index]; var v1 = rod.VelocityData[index + 1];
        float i0 = rod.InverseMass[index] * w0, i1 = rod.InverseMass[index + 1] * w1;
        rod.VelocityData[index] = new Vector3F(v0.X + impulse.X * i0, v0.Y + impulse.Y * i0, v0.Z + impulse.Z * i0);
        rod.VelocityData[index + 1] = new Vector3F(v1.X + impulse.X * i1, v1.Y + impulse.Y * i1, v1.Z + impulse.Z * i1);
    }

    /// <summary>A virtual character walking into a soft particle or a rod segment brings it up to the walking speed along the push direction, with the same bounded force as for a rigid body.</summary>
    private void MvPushDeformable(in CharacterDef def, in MoverHitRec h, Vector3F displacement, float dt, in MoverCtx c)
    {
        var hn = new Vector3F(h.Nx, h.Ny, h.Nz);
        if (Vector3F.Dot(hn, c.Up) >= c.CosSlope) { return; }
        float want = -Vector3F.Dot(displacement, hn) / dt;
        int v = -1 - h.Slot;
        float have, effectiveMass;
        float along = 0f;
        if (v < MvRodBase)
        {
            var d = SoftData[v]!;
            float inv = d.InvMass[h.Particle];
            if (!(inv > 0f)) { return; }
            have = -(d.Vel[3 * h.Particle] * hn.X + d.Vel[3 * h.Particle + 1] * hn.Y + d.Vel[3 * h.Particle + 2] * hn.Z);
            effectiveMass = 1f / inv;
        }
        else
        {
            var rod = Rods.Data[v - MvRodBase]!;
            along = 0f;
            for (int i = 0; i < _mvDefCount; i++)
            {
                if (_mvDef[i].Slot == h.Slot && _mvDef[i].Index == h.Particle) { along = _mvDef[i].Along; break; }
            }

            float w0 = 1f - along, w1 = along;
            float k = w0 * w0 * rod.InverseMass[h.Particle] + w1 * w1 * rod.InverseMass[h.Particle + 1];
            if (!(k > 0f)) { return; }
            var v0 = rod.VelocityData[h.Particle]; var v1 = rod.VelocityData[h.Particle + 1];
            have = -((w0 * v0.X + w1 * v1.X) * hn.X + (w0 * v0.Y + w1 * v1.Y) * hn.Y + (w0 * v0.Z + w1 * v1.Z) * hn.Z);
            effectiveMass = 1f / k;
        }

        float dv = (want - have) * def.PushStrength;
        if (!(dv > 0f)) { return; }
        float impulse = Math.Min(dv * effectiveMass, def.MaxPushForce * dt);
        ApplyDeformableImpulse(h.Slot, h.Particle, along, hn * -impulse);
    }
}
