using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private int ContactGroupFloats => Core.Controls.Active ? W2D.ControlGroupFloats : W2D.GroupFloats;
    private void CommitMassControl(int b, BodyControlState value)
    {
        var previous = Core.Controls.Get(b); Core.Controls.Set(b, value);
        try { UpdateMass(b, true); }
        catch (ArgumentException) { Core.Controls.Set(b, previous); throw; }
        Core.Wake(Core.HandleOfBody(b));
    }
    private static void CheckMass(float value) { if (!BodyControlStore.ValidMass(value)) throw new ArgumentOutOfRangeException(nameof(value), value, "Mass and principal inertia must be within 1e-18..1e30."); }
    public void SetMassOverride(int b, float value) { CheckMass(value); var s = Core.Controls.Get(b); s.Mass = value; s.Overrides |= 1; CommitMassControl(b, s); }
    public void ResetMassOverride(int b, byte flag)
    {
        var s = Core.Controls.Get(b); s.Overrides &= (byte)~flag;
        if ((flag & 1) != 0) s.Mass = 0;
        if ((flag & 2) != 0) s.Center = default;
        if ((flag & 4) != 0) { s.Principal = default; s.InertiaRotation = default; }
        CommitMassControl(b, s);
    }
    public void SetCenterOverride(int b, Vector2F value)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || Math.Abs(value.X) > 2097152 || Math.Abs(value.Y) > 2097152 ) throw new ArgumentOutOfRangeException(nameof(value), value, "Center must be finite within ±2097152.");
        var s = Core.Controls.Get(b); s.Center = new(value.X, value.Y, 0); s.Overrides |= 2; CommitMassControl(b, s);
    }
    public void SetInertiaOverride(int b, float value)
    {
        CheckMass(value);
        var s = Core.Controls.Get(b); s.Principal = new(value, 0, 0); s.InertiaRotation = QuaternionF.Identity; s.Overrides |= 4; CommitMassControl(b, s);
    }
    public void CancelKinematicDrive(int b)
    {
        if (Core.Bodies.TypeOf(b) != BodyType.Kinematic) return;
        var state = Core.Controls.Get(b); if ((state.Overrides & 24) == 0) return;
        state.Overrides &= unchecked((byte)~24); state.TargetPosition = default; state.TargetRotation = default; Core.Controls.Set(b, state);
    }
    private void RequireKinematic(int b) { if (Core.Bodies.TypeOf(b) != BodyType.Kinematic) throw new ArgumentException("MoveKinematic requires a kinematic body; dynamic drives are not supported.", nameof(b)); }
    public void SetKinematicTarget(int b, Vector2D position, float angle)
    {
        RequireKinematic(b);
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || Math.Abs(position.X) > 2097152 || Math.Abs(position.Y) > 2097152 ) throw new ArgumentOutOfRangeException(nameof(position), position, "Position must be finite within ±2097152.");
        if (!float.IsFinite(angle)) throw new ArgumentOutOfRangeException(nameof(angle)); var (sn, cs) = Math64.SinCos(angle); var rotation = new QuaternionF(0, 0, (float)sn, (float)cs);
        var s = Core.Controls.Get(b); s.TargetPosition = new(WorldCore.Grid(position.X), WorldCore.Grid(position.Y), 0); s.TargetRotation = rotation; s.Overrides |= 24; Core.Controls.Set(b, s);
    }
    public void SetKinematicPosition(int b, Vector2D position)
    {
        var s = Core.Controls.Get(b);
        var savedRotation = (s.Overrides & 8) != 0 ? s.TargetRotation : new QuaternionF(0,0,Core.Bodies.RotationOf(b)[1],Core.Bodies.RotationOf(b)[0]);
        SetKinematicTarget(b, position, (s.Overrides & 8) != 0 ? (float)Math64.Atan2(s.TargetRotation.Z, s.TargetRotation.W) : new Body(World!, Core.HandleOfBody(b)).Angle);
        s = Core.Controls.Get(b); s.TargetRotation = savedRotation; Core.Controls.Set(b, s);
    }
    public void SetKinematicRotation(int b, float rotation)
    {
        var s = Core.Controls.Get(b); var p = (s.Overrides & 8) != 0 ? s.TargetPosition : new Vector3D(new Body(World!, Core.HandleOfBody(b)).Position.X, new Body(World!, Core.HandleOfBody(b)).Position.Y, 0);
        SetKinematicTarget(b, new(p.X, p.Y), rotation);
    }
    private void TargetMotion(int b, BodyControlState s, float dt, out Vector3F linear, out Vector3F angular)
    {
        var cp = Core.Bodies.PositionOf(b);
        float lx=LocalCenter[2*b], ly=LocalCenter[2*b+1];
        double x=s.TargetPosition.X+(double)(s.TargetRotation.W*lx-s.TargetRotation.Z*ly), y=s.TargetPosition.Y+(double)(s.TargetRotation.Z*lx+s.TargetRotation.W*ly);
        var r=Core.Bodies.RotationOf(b);
        double angle=Math64.Atan2((double)r[0]*s.TargetRotation.Z-(double)r[1]*s.TargetRotation.W,(double)r[0]*s.TargetRotation.W+(double)r[1]*s.TargetRotation.Z);
        linear=new((float)((x-cp[0])/dt),(float)((y-cp[1])/dt),0); angular=new(0,0,(float)(angle/dt));
    }
    public void ValidateTargetMotion(float dt)
    {
        var bodies=Core.Bodies;
        for(int b=0;b<bodies.HighWater;b++)
        {
            var s=Core.Controls.Get(b);if(!bodies.IsLive(b)||(s.Overrides&8)==0)continue;
            TargetMotion(b,s,dt,out var v,out var w);
            double linear2=(double)v.X*v.X+(double)v.Y*v.Y+(double)v.Z*v.Z, angular2=(double)w.X*w.X+(double)w.Y*w.Y+(double)w.Z*w.Z;
            float linearLimit=Math.Min(Opt.MaxLinearSpeed,s.Settings.MaxLinearSpeed), angularLimit=Math.Min(Core.Controls.DefaultAngularSpeed,s.Settings.MaxAngularSpeed);
            if(!double.IsFinite(linear2)||!double.IsFinite(angular2)||linear2>(double)linearLimit*linearLimit||angular2>(double)angularLimit*angularLimit)throw new ArgumentOutOfRangeException(nameof(dt),dt,"Kinematic target exceeds the body's speed limits at this dt.");
            int mask=(int)s.Settings.Constraints;
            if(((mask&1)!=0&&v.X!=0)||((mask&2)!=0&&v.Y!=0)||((mask&4)!=0&&v.Z!=0)||((mask&8)!=0&&w.X!=0)||((mask&16)!=0&&w.Y!=0)||((mask&32)!=0&&w.Z!=0))throw new ArgumentException("Kinematic target conflicts with a locked world axis.");
        }
    }
    public void BeginBodyControls(float dt)
    {
        if (!Core.Controls.Active) return;
        var bodies = Core.Bodies;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b)) continue;
            var s = Core.Controls.Get(b);
            if ((s.Overrides & 16) != 0)
            {
                var v = bodies.LinearVelocityOf(b); var w = bodies.AngularVelocityOf(b); v.Clear(); w.Clear();
                if ((s.Overrides & 8) != 0)
                {
                    TargetMotion(b,s,dt,out var linear,out var angular);
                    v[0]=linear.X;v[1]=linear.Y;w[0]=angular.Z;
                }
            }
            Core.ProjectBodyVelocity(b);

        }
    }
    public void EndBodyControls()
    {
        if (!Core.Controls.Active) return;
        for (int b = 0; b < Core.Bodies.HighWater; b++)
        {
            if (!Core.Bodies.IsLive(b)) continue;
            var s = Core.Controls.Get(b); if ((s.Overrides & 8) == 0) continue;
            var r = Core.Bodies.RotationOf(b); var p = Core.Bodies.PositionOf(b);
            r[0] = s.TargetRotation.W; r[1] = s.TargetRotation.Z; float lx = LocalCenter[2*b], ly = LocalCenter[2*b+1]; p[0] = s.TargetPosition.X + (double)(r[0]*lx-r[1]*ly); p[1] = s.TargetPosition.Y + (double)(r[1]*lx+r[0]*ly);
            for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh]) { Bp.MoveProxy(Proxy[sh], TightBox(b, in Geom[sh]), 0, 0); }
            s.Overrides &= unchecked((byte)~8); s.TargetPosition = default; s.TargetRotation = default; Core.Controls.Set(b, s);
        }
    }
    private static void LimitVector(Span<float> v, float limit)
    {
        double length2 = 0; for (int k = 0; k < v.Length; k++) length2 += (double)v[k] * v[k];
        if (length2 > (double)limit * limit) { float scale = (float)(limit / double.Sqrt(length2)); if (scale > 0) scale = float.BitDecrement(scale); for (int k = 0; k < v.Length; k++) v[k] *= scale; }
    }
    private void LimitBodyControls(int b)
    {
        var s = Core.Controls.Get(b).Settings; var st = BodyState.AsSpan((b+1)*BS2D.Stride, BS2D.Stride);
        int m = (int)s.Constraints;
        if ((m & 1) != 0) st[0] = 0; if ((m & 2) != 0) st[1] = 0; if ((m & 32) != 0) st[2] = 0;
        LimitVector(st.Slice(0, 2), Math.Min(Opt.MaxLinearSpeed, s.MaxLinearSpeed));
        LimitVector(st.Slice(2, 1), Math.Min(Core.Controls.DefaultAngularSpeed, s.MaxAngularSpeed));
    }
}
