using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // Soft bodies and rods as ground and obstacles for the character mover: soft particles are discs and rod segments are capsules; they enter the plane list like shapes do, with negative slots,
    // and a character standing on them adds its mass to the particles under its feet for the step.

    private struct DefContact
    {
        public int Slot, Index;
        public float Along, Sep, Nx, Ny;
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

    private void MvAddDefContact(int slot, int index, float along, float sep, float nx, float ny)
    {
        if (_mvDefCount == _mvDef.Length) { Array.Resize(ref _mvDef, _mvDefCount * 2); }
        _mvDef[_mvDefCount++] = new DefContact { Slot = slot, Index = index, Along = along, Sep = sep, Nx = nx, Ny = ny };
    }

    /// <summary>One plane per direction, like the 3D mover: parallel planes are implied by the nearest one.</summary>
    private void MvAddDeformPlane(float nx, float ny, float sep, double px, double py, int slot, int particle, float along, in MoverCtx c)
    {
        bool walkable = ny >= c.CosSlope;
        float separation = walkable ? sep : sep - c.Skin;
        float push = c.MaxPush;
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (_mvPlanes[i].PushLimit != push || _mvPlanes[i].Normal.X * nx + _mvPlanes[i].Normal.Y * ny <= 0.99995f) { continue; }
            if (separation < _mvPlanes[i].Separation)
            {
                _mvPlanes[i].Separation = separation;
                _mvMeta[i] = new PlaneMeta { Slot = slot, Px = px, Py = py, Particle = particle, Along = along };
            }

            return;
        }

        if (_mvPlaneCount == _mvPlanes.Length)
        {
            Array.Resize(ref _mvPlanes, _mvPlaneCount * 2);
            Array.Resize(ref _mvMeta, _mvPlaneCount * 2);
        }

        int k = _mvPlaneCount++;
        _mvPlanes[k] = new CollisionPlane2D { Normal = new Vector2F(nx, ny), Separation = separation, PushLimit = push };
        _mvMeta[k] = new PlaneMeta { Slot = slot, Px = px, Py = py, Particle = particle, Along = along };
    }

    private void MvDeformablePlanes(double x, double y, in MoverCtx c)
    {
        double half = c.HalfLength;
        // The capsule's axis, relative to (x, y): straight up and down.
        var a0 = new Vector3D(0.0, -half, 0.0);
        var a1 = new Vector3D(0.0, half, 0.0);
        double reach = c.Radius + c.Range + 1e-3;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            double rp = d.Radius;
            double r = reach + rp;
            double ex = r, ey = half + r;
            if (d.MaxX < x - ex || d.MinX > x + ex || d.MaxY < y - ey || d.MinY > y + ey) { continue; }
            for (int k = 0; k < d.N; k++)
            {
                double qx = d.Pos[2 * k] - x, qy = d.Pos[2 * k + 1] - y;
                if (Math.Abs(qx) > ex || Math.Abs(qy) > ey) { continue; }
                double t = qy < -half ? -half : qy > half ? half : qy;
                double dx = -qx, dy = t - qy;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                float sep = (float)(dist - (c.Radius + rp));
                if (sep > c.Range) { continue; }
                float nx = dist > 1e-9 ? (float)(dx / dist) : 0f, ny = dist > 1e-9 ? (float)(dy / dist) : 1f;
                MvAddDefContact(MvSoftSlot(i), k, 0f, sep, nx, ny);
                MvAddDeformPlane(nx, ny, sep, x + qx + (double)nx * rp, y + qy + (double)ny * rp, MvSoftSlot(i), k, 0f, in c);
            }
        }

        for (int rod = 0; rod < Rods.Data.Length; rod++)
        {
            var d = Rods.Data[rod];
            if (d == null) { continue; }
            double rr = d.Material.Radius;
            double r = reach + rr;
            double ex = r, ey = half + r;
            for (int j = 0; j + 1 < d.PositionData.Length; j++)
            {
                var pa = d.PositionData[j]; var pb = d.PositionData[j + 1];
                double ax = pa.X - x, ay = pa.Y - y, bx = pb.X - x, by = pb.Y - y;
                if ((ax > ex && bx > ex) || (ax < -ex && bx < -ex) || (ay > ey && by > ey) || (ay < -ey && by < -ey)) { continue; }
                CharacterGeometry.SegmentSegment(a0, a1, new Vector3D(ax, ay, 0.0), new Vector3D(bx, by, 0.0), out _, out double tt, out Vector3D c1, out Vector3D c2);
                double dx = c1.X - c2.X, dy = c1.Y - c2.Y;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                float sep = (float)(dist - (c.Radius + rr));
                if (sep > c.Range) { continue; }
                float nx = dist > 1e-9 ? (float)(dx / dist) : 0f, ny = dist > 1e-9 ? (float)(dy / dist) : 1f;
                MvAddDefContact(MvRodSlot(rod), j, (float)tt, sep, nx, ny);
                MvAddDeformPlane(nx, ny, sep, x + c2.X + (double)nx * rr, y + c2.Y + (double)ny * rr, MvRodSlot(rod), j, (float)tt, in c);
            }
        }
    }

    private static bool IsSupportContact(in DefContact k, in MoverCtx c) => k.Sep <= 2f * c.Skin + 1e-3f && k.Ny >= c.CosSlope;

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
    // among the contacts. The solver then holds the weight with its own constraints; no impulse is invented. The masses go back after the step, so snapshots never hold them.
    private int[] _supSoftBody = new int[16], _supSoftParticle = new int[16];
    private int _supSoftCount;
    private float _supDt;
    private int[] _supRods = new int[4];
    private int _supRodCount;
    private System.Collections.Generic.List<float[]>? _supRodAdds;

    /// <summary>Before the soft and rod steps: adds the mass of every character that stands on soft particles or rod segments to the particles under it.</summary>
    private void ApplySupportMass(float dt)
    {
        _supDt = dt;
        _supSoftCount = 0; _supRodCount = 0;
        if (SoftHandles.Count == 0 && Rods.Count == 0) { return; }
        bool prepared = false;
        for (int ch = 0; ch < CharHandles.HighWater; ch++)
        {
            if (!CharHandles.IsLive(ch) || CharGround[ch] != GroundState.OnGround || CharGroundBody[ch] >= 0) { continue; }
            if (CharacterIsBusy(ch)) { continue; }
            if (!prepared) { MvPrepareDeformables(); prepared = true; }
            var def = CharDef[ch];
            float halfLength = 0.5f * def.Height - def.Radius;
            double cosSlope = Math64.SinCos(def.SolverSlopeRadians).Item2;
            var c = new MoverCtx
            {
                Radius = def.Radius, HalfLength = halfLength, CosSlope = (float)cosSlope, Step = def.StepOffset, Skin = def.Skin, Range = 3f * def.Skin, MaxPush = 0.5f * def.Radius,
                Mask = def.LayerMask, IgnoreBody = CharBody[ch], Deformables = true,
            };
            _mvPlaneCount = 0; _mvDefCount = 0;
            if (_mvPlanes.Length == 0) { _mvPlanes = new CollisionPlane2D[32]; _mvMeta = new PlaneMeta[32]; }
            CharacterPosition(ch, out double px, out double py);
            MvDeformablePlanes(px, py, in c);
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
        if (miss != 0f) { d.Vel[2 * particle] += Opt.Gravity.X * miss; d.Vel[2 * particle + 1] += Opt.Gravity.Y * miss; }
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
                if (miss != 0f) { d.VelocityData[p] += new Vector3F(Opt.Gravity.X * miss, Opt.Gravity.Y * miss, 0f); }
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
    private void ApplyDeformableImpulse(int slot, int index, float along, float ix, float iy)
    {
        int v = -1 - slot;
        if (v < MvRodBase)
        {
            var d = SoftData[v]!;
            float inv = d.InvMass[index];
            d.Vel[2 * index] += ix * inv; d.Vel[2 * index + 1] += iy * inv;
            d.Asleep = false; d.SleepTime = 0f;
            return;
        }

        var rod = Rods.Data[v - MvRodBase]!;
        float w0 = 1f - along, w1 = along;
        var v0 = rod.VelocityData[index]; var v1 = rod.VelocityData[index + 1];
        float i0 = rod.InverseMass[index] * w0, i1 = rod.InverseMass[index + 1] * w1;
        rod.VelocityData[index] = new Vector3F(v0.X + ix * i0, v0.Y + iy * i0, 0f);
        rod.VelocityData[index + 1] = new Vector3F(v1.X + ix * i1, v1.Y + iy * i1, 0f);
    }

    /// <summary>A character walking into a soft particle or a rod segment brings it up to the walking speed along the push direction, with the same bounded force as for a rigid body.</summary>
    private void MvPushDeformable(in CharacterDef def, in MoverHitRec h, Vector2F displacement, float dt, in MoverCtx c)
    {
        if (h.Ny >= c.CosSlope) { return; }
        float want = (displacement.X * -h.Nx + displacement.Y * -h.Ny) / dt;
        int v = -1 - h.Slot;
        float have, effectiveMass;
        float along = 0f;
        if (v < MvRodBase)
        {
            var d = SoftData[v]!;
            float inv = d.InvMass[h.Particle];
            if (!(inv > 0f)) { return; }
            have = d.Vel[2 * h.Particle] * -h.Nx + d.Vel[2 * h.Particle + 1] * -h.Ny;
            effectiveMass = 1f / inv;
        }
        else
        {
            var rod = Rods.Data[v - MvRodBase]!;
            for (int i = 0; i < _mvDefCount; i++)
            {
                if (_mvDef[i].Slot == h.Slot && _mvDef[i].Index == h.Particle) { along = _mvDef[i].Along; break; }
            }

            float w0 = 1f - along, w1 = along;
            float k = w0 * w0 * rod.InverseMass[h.Particle] + w1 * w1 * rod.InverseMass[h.Particle + 1];
            if (!(k > 0f)) { return; }
            var v0 = rod.VelocityData[h.Particle]; var v1 = rod.VelocityData[h.Particle + 1];
            have = (w0 * v0.X + w1 * v1.X) * -h.Nx + (w0 * v0.Y + w1 * v1.Y) * -h.Ny;
            effectiveMass = 1f / k;
        }

        float dv = (want - have) * def.PushStrength;
        if (!(dv > 0f)) { return; }
        float impulse = Math.Min(dv * effectiveMass, def.MaxPushForce * dt);
        ApplyDeformableImpulse(h.Slot, h.Particle, along, -h.Nx * impulse, -h.Ny * impulse);
    }
}
