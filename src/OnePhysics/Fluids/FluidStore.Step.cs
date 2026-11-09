using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    /// <summary>Statistics of the last step, for tests and inspection; not saved.</summary>
    internal int LastContacts, LastCoupledContacts, LastDeformables, LastCandidates, LastSubsteps;
    internal float LastMaxSpeed, LastMaxDensityError, LastMeanDensityError;
    internal bool LastStepped;

    private void SetUpStep(float dt)
    {
        var st = Settings;
        _h = dt / st.Substeps;
        _invDt = 1f / dt;
        _k = new FluidKernels(Dim, Spacing * KernelScale);
        _spacing2 = Spacing * Spacing;
        _spec = 0.25f * Spacing;
        var g = Host.Gravity;
        _g.X = g.X; _g.Y = g.Y; _g.Z = Dim == 3 ? g.Z : 0f;
        _anyVisc = false; _anyVort = false;
        for (int f = 0; f < FluidHandles.HighWater; f++)
        {
            if (!FluidHandles.IsLive(f)) { continue; }
            var s = Fluids[f]!; var d = s.Def;
            _mass[f] = s.Mass; _invMass[f] = 1f / s.Mass; _n0[f] = s.N0;
            _visc[f] = d.Viscosity; _vort[f] = d.Vorticity; _tens[f] = d.TensileCorrection; _fric[f] = d.Friction;
            _maxSpeed[f] = d.MaxSpeed; _gscale[f] = d.GravityScale;
            _epsA[f] = d.Relaxation / _spacing2;
            if (d.Viscosity > 0f) { _anyVisc = true; }
            if (d.Vorticity > 0f) { _anyVort = true; }
        }

        // Expected neighbours of a rest lattice, with room for compression.
        double ratio = KernelScale;
        double expected = Dim == 3 ? 4.18879 * ratio * ratio * ratio : 3.14159 * ratio * ratio;
        int stride = (int)Math.Ceiling(expected * 2.0) + 8;
        if (stride != _stride)
        {
            _stride = stride;
            _nbr = Array.Empty<int>();
        }

        long need = (long)Math.Max(Count, 64) * _stride;
        if (need > 1_500_000_000L) { throw new FluidBudgetException("The neighbour lists of " + Count.ToString(System.Globalization.CultureInfo.InvariantCulture) + " particles at this kernel radius would take too much memory; use a smaller KernelRadiusScale or fewer particles."); }
        EnsureWork(Count);
        if (_nbr.Length < need) { _nbr = new int[(int)Math.Min(Math.Max(need, (long)_nbr.Length * 2), 1_500_000_000L)]; }
    }

    /// <summary>Steps the fluids once, after the rigid step. Emitters add particles first; drains remove them last; a settled fluid sleeps.</summary>
    internal void Step(float dt)
    {
        StepCount++;
        Events.Clear();
        LastStepped = false;
        if (FluidHandles.Count == 0) { return; }
        EmitAll(dt);
        if (Count == 0)
        {
            _bodyCount = 0;
            EvaluateWetness(dt);
            return;
        }

        SetUpStep(dt);
        // Bounds, the local origin and the candidate list.
        double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue, mxx = double.MinValue, mxy = double.MinValue, mxz = double.MinValue;
        float vmax = 0f;
        for (int i = 0; i < Count; i++)
        {
            double x = PX[i], y = PY[i], z = PZ[i];
            if (x < mnx) { mnx = x; } if (x > mxx) { mxx = x; }
            if (y < mny) { mny = y; } if (y > mxy) { mxy = y; }
            if (z < mnz) { mnz = z; } if (z > mxz) { mxz = z; }
            float v2 = PVx[i] * PVx[i] + PVy[i] * PVy[i] + PVz[i] * PVz[i];
            if (v2 > vmax) { vmax = v2; }
        }

        vmax = float.Sqrt(vmax);
        float gm = float.Sqrt(_g.X * _g.X + _g.Y * _g.Y + _g.Z * _g.Z);
        float travel = (vmax + 2f * gm * dt + 1f) * dt;
        double margin = travel + _k.H;
        _ox = Math.Floor(mnx - margin); _oy = Math.Floor(mny - margin); _oz = Dim == 3 ? Math.Floor(mnz - margin) : 0.0;

        PrepareCandidates(mnx - margin, mny - margin, Dim == 3 ? mnz - margin : -1.0, mxx + margin, mxy + margin, Dim == 3 ? mxz + margin : 1.0);
        _def.Clear();
        Host.GatherDeformables(mnx - margin, mny - margin, Dim == 3 ? mnz - margin : -1.0, mxx + margin, mxy + margin, Dim == 3 ? mxz + margin : 1.0, _def);
        if (Asleep)
        {
            if (!ShouldWake(dt)) { LastCandidates = _candCount; return; }
            Wake();
        }

        LastStepped = true;
        LastCandidates = _candCount; LastDeformables = _def.Count;
        for (int i = 0; i < Count; i++)
        {
            _x[i] = (float)(PX[i] - _ox); _y[i] = (float)(PY[i] - _oy); _z[i] = Dim == 3 ? (float)(PZ[i] - _oz) : 0f;
        }

        _def.Substeps = Settings.Substeps;
        if (_def.Count > 0) { BuildDeformableGrid(); }
        LastContacts = 0; LastCoupledContacts = 0;
        int substeps = Settings.Substeps, iterations = Settings.Iterations;
        var ex = Host.Core.Executor;
        for (int sub = 0; sub < substeps; sub++)
        {
            _lastSubstep = sub == substeps - 1;
            AdvanceBodies();
            ex.For((Count + Chunk - 1) / Chunk, new PredictTask { S = this });
            SortParticles();
            ex.For(_taskCount, new NeighbourTask { S = this });
            ex.For(_taskCount, new DetectTask { S = this });
            JoinContacts();
            LastContacts = _csCount; LastCoupledContacts = _coupledCount;
            for (int it = 0; it < iterations; it++)
            {
                ex.For(_taskCount, new LambdaTask { S = this });
                ex.For(_taskCount, new DeltaTask { S = this });
                ex.For(_taskCount, new ApplyTask { S = this, Move = true });
                if (_coupledCount > 0) { SolveCoupled(); }
            }

            if (_coupledCount > 0) { ex.For(_taskCount, new ApplyTask { S = this, Move = false }); }
            ex.For(_taskCount, new FinishTask { S = this });
            if (_anyVisc || _anyVort)
            {
                ex.For(_taskCount, new SmoothTask { S = this });
                ex.For(_taskCount, new ConfineTask { S = this });
            }

            UpdateBodies();
        }

        // Back to double positions, in the final order.
        float vm2 = 0f;
        for (int i = 0; i < Count; i++)
        {
            PX[i] = _ox + _x[i]; PY[i] = _oy + _y[i]; PZ[i] = Dim == 3 ? _oz + _z[i] : 0.0;
            float v2 = PVx[i] * PVx[i] + PVy[i] * PVy[i] + PVz[i] * PVz[i];
            if (v2 > vm2) { vm2 = v2; }
        }

        _qValid = false;
        LastMaxSpeed = float.Sqrt(vm2);
        LastSubsteps = substeps;
        ApplyToBodies();
        Host.ApplyDeformables(_def);
        DrainAll();
        UpdateSleep(dt);
        EvaluateWetness(dt);
    }

    private void ApplyToBodies()
    {
        float wake = Settings.WakeBodySpeed;
        for (int i = 0; i < _bodyCount; i++)
        {
            ref var b = ref _bodies[i];
            if (!b.Dynamic) { continue; }
            if (b.Tx == 0f && b.Ty == 0f && b.Tz == 0f && b.Ux == 0f && b.Uy == 0f && b.Uz == 0f) { continue; }
            float dvx = b.Tx * b.InvMass, dvy = b.Ty * b.InvMass, dvz = b.Tz * b.InvMass;
            float dwx = b.Iw0 * b.Ux + b.Iw1 * b.Uy + b.Iw2 * b.Uz, dwy = b.Iw1 * b.Ux + b.Iw3 * b.Uy + b.Iw4 * b.Uz, dwz = b.Iw2 * b.Ux + b.Iw4 * b.Uy + b.Iw5 * b.Uz;
            Host.ApplyBodyImpulse(b.Body, dvx, dvy, dvz, dwx, dwy, dwz, wake);
        }
    }

    private void UpdateSleep(float dt)
    {
        var st = Settings;
        if (!(st.SleepSpeed > 0f) || HasActiveEmitter()) { QuietTime = 0f; return; }
        if (LastMaxSpeed < st.SleepSpeed) { QuietTime += dt; } else { QuietTime = 0f; }
        if (QuietTime >= st.TimeToSleep)
        {
            Asleep = true;
            for (int i = 0; i < Count; i++) { PVx[i] = 0f; PVy[i] = 0f; PVz[i] = 0f; }
        }
    }

    private bool HasActiveEmitter()
    {
        for (int e = 0; e < EmitterHandles.HighWater; e++)
        {
            if (EmitterHandles.IsLive(e) && Emitters[e]!.Def.Enabled && Emitters[e]!.Def.Speed > 0f) { return true; }
        }

        return false;
    }

    /// <summary>A sleeping fluid wakes when an emitter runs, or a moving body or a deformable's particle comes within reach of it. Static geometry that changes does not wake it; call Wake from the host.</summary>
    private bool ShouldWake(float dt)
    {
        if (HasActiveEmitter()) { return true; }
        if (_def.Count > 0) { return true; }
        for (int i = 0; i < _bodyCount; i++)
        {
            ref var b = ref _bodies[i];
            if (!b.Moves) { continue; }
            float v2 = b.Vx * b.Vx + b.Vy * b.Vy + b.Vz * b.Vz + b.Wx * b.Wx + b.Wy * b.Wy + b.Wz * b.Wz;
            if (v2 > 1e-6f) { return true; }
            // A body at rest that falls onto the fluid is moving next step; one that rests on it is not.
            if (b.Dynamic) { return true; }
        }

        _ = dt;
        return false;
    }

    internal List<FluidEvent> Events = new();
}
