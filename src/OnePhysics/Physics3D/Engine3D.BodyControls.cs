using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    private int ContactGroupFloats => Core.Controls.Active ? W3D.ControlGroupFloats : W3D.GroupFloats;
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
    public void SetCenterOverride(int b, Vector3F value)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || Math.Abs(value.X) > MaxCenterOffset || Math.Abs(value.Y) > MaxCenterOffset || !float.IsFinite(value.Z) || Math.Abs(value.Z) > MaxCenterOffset) throw new ArgumentOutOfRangeException(nameof(value), value, "Center must be finite within ±2097152.");
        var s = Core.Controls.Get(b); s.Center = value; s.Overrides |= 2; CommitMassControl(b, s);
    }
    public void SetInertiaOverride(int b, Vector3F value, QuaternionF rotation)
    {
        CheckMass(value.X); CheckMass(value.Y); CheckMass(value.Z); rotation = Normalize(rotation, nameof(rotation));
        var s = Core.Controls.Get(b); s.Principal = value; s.InertiaRotation = rotation; s.Overrides |= 4; CommitMassControl(b, s);
    }
    public void CancelKinematicDrive(int b)
    {
        if (Core.Bodies.TypeOf(b) != BodyType.Kinematic) return;
        var state = Core.Controls.Get(b); if ((state.Overrides & 24) == 0) return;
        state.Overrides &= unchecked((byte)~24); state.TargetPosition = default; state.TargetRotation = default; Core.Controls.Set(b, state);
    }
    private void RequireKinematic(int b) { if (Core.Bodies.TypeOf(b) != BodyType.Kinematic) throw new ArgumentException("MoveKinematic requires a kinematic body; dynamic drives are not supported.", nameof(b)); }
    public void SetKinematicTarget(int b, Vector3D position, QuaternionF rotation)
    {
        RequireKinematic(b);
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || Math.Abs(position.X) > 2097152 || Math.Abs(position.Y) > 2097152 || !double.IsFinite(position.Z) || Math.Abs(position.Z) > 2097152) throw new ArgumentOutOfRangeException(nameof(position), position, "Position must be finite within ±2097152.");
        rotation = Normalize(rotation, nameof(rotation));
        var s = Core.Controls.Get(b); s.TargetPosition = new(WorldCore.Grid(position.X), WorldCore.Grid(position.Y), WorldCore.Grid(position.Z)); s.TargetRotation = rotation; s.Overrides |= 24; Core.Controls.Set(b, s);
    }
    public void SetKinematicPosition(int b, Vector3D position)
    {
        var s = Core.Controls.Get(b);
        var savedRotation = (s.Overrides & 8) != 0 ? s.TargetRotation : RotationOfBody(b);
        SetKinematicTarget(b, position, (s.Overrides & 8) != 0 ? s.TargetRotation : RotationOfBody(b));
        s = Core.Controls.Get(b); s.TargetRotation = savedRotation; Core.Controls.Set(b, s);
    }
    public void SetKinematicRotation(int b, QuaternionF rotation)
    {
        var s = Core.Controls.Get(b); var p = (s.Overrides & 8) != 0 ? s.TargetPosition : new Body(World!, Core.HandleOfBody(b)).Position;
        SetKinematicTarget(b, p, rotation);
    }
    private void TargetMotion(int b, BodyControlState s, float dt, out Vector3F linear, out Vector3F angular)
    {
        var cp = Core.Bodies.PositionOf(b);
        var off=s.TargetRotation.Rotate(LocalCenterOf(b));
        linear=new((float)((s.TargetPosition.X+off.X-cp[0])/dt),(float)((s.TargetPosition.Y+off.Y-cp[1])/dt),(float)((s.TargetPosition.Z+off.Z-cp[2])/dt));
        var q=s.TargetRotation*RotationOfBody(b).Conjugated(); if(q.W<0)q=new(-q.X,-q.Y,-q.Z,-q.W);
        double n=double.Sqrt((double)q.X*q.X+(double)q.Y*q.Y+(double)q.Z*q.Z); double scale=n>0 ? 2*Math64.Atan2(n,q.W)/dt/n : 0;
        angular=new((float)(q.X*scale),(float)(q.Y*scale),(float)(q.Z*scale));
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
                    v[0]=linear.X;v[1]=linear.Y;v[2]=linear.Z;w[0]=angular.X;w[1]=angular.Y;w[2]=angular.Z;
                }
            }
            Core.ProjectBodyVelocity(b);
            RefreshWorldInertia(b);
        }
    }
    public void EndBodyControls()
    {
        if (Core.Controls.Active || Opt.MaxLinearSpeed != 400f) foreach (int b in Core.AwakeBodies) LimitBodyControls(b);
        if (!Core.Controls.Active) return;
        for (int b = 0; b < Core.Bodies.HighWater; b++)
        {
            if (!Core.Bodies.IsLive(b)) continue;
            var s = Core.Controls.Get(b); if ((s.Overrides & 8) == 0) continue;
            var r = Core.Bodies.RotationOf(b); var p = Core.Bodies.PositionOf(b);
            r[0] = s.TargetRotation.X; r[1] = s.TargetRotation.Y; r[2] = s.TargetRotation.Z; r[3] = s.TargetRotation.W; var off = s.TargetRotation.Rotate(LocalCenterOf(b)); p[0] = s.TargetPosition.X + off.X; p[1] = s.TargetPosition.Y + off.Y; p[2] = s.TargetPosition.Z + off.Z;
            for (int sh = ShapeHead[b]; sh >= 0; sh = NextShape[sh]) { Bp.MoveProxy(Proxy[sh], TightBox(b, sh), 0, 0, 0); }
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
        Core.ProjectBodyVelocity(b); var s = Core.Controls.Get(b).Settings;
        LimitVector(Core.Bodies.LinearVelocityOf(b), Math.Min(Opt.MaxLinearSpeed, s.MaxLinearSpeed));
        LimitVector(Core.Bodies.AngularVelocityOf(b), Math.Min(Core.Controls.DefaultAngularSpeed, s.MaxAngularSpeed));
    }
    private bool JointIsLocked(int c) => Core.Controls.HasLocks && (Core.Controls.ConstraintsOf(Core.Constraints.BodyA[c]) | Core.Controls.ConstraintsOf(Core.Constraints.BodyB[c])) != 0;
    private void ProjectInertia(int b)
    {
        int m=(int)Core.Controls.Get(b).Settings.Constraints & 56; if(m==0)return;
        var output=WInvI.AsSpan(6*b,6); output.Clear(); if(m==56 || (BodyFlags[b]&1)!=0 || Core.Bodies.TypeOf(b)!=BodyType.Dynamic)return;
        var local=Inertia.AsSpan(6*b,6);
        var tensor=InertiaF.Rotate(new Matrix3x3F(local[0],local[1],local[2],local[1],local[3],local[4],local[2],local[4],local[5]),RotationOfBody(b));
        // Restrict the kinetic-energy quadratic form to allowed angular coordinates, then invert.
        // This is the Schur-complement response of ideal angular lock reaction torques.
        int first=(m&8)==0 ? 0 : (m&16)==0 ? 1 : 2;
        int second=first==0 ? ((m&16)==0 ? 1 : (m&32)==0 ? 2 : -1) : first==1 && (m&32)==0 ? 2 : -1;
        float aa=first==0 ? tensor.M11 : first==1 ? tensor.M22 : tensor.M33;
        int da=first==0 ? 0 : first==1 ? 3 : 5;
        if(second<0){if(aa>0)output[da]=1/aa;return;}
        float bb=second==1 ? tensor.M22 : tensor.M33;
        float ab=first==0 ? (second==1 ? tensor.M12 : tensor.M13) : tensor.M23;
        double determinant=(double)aa*bb-(double)ab*ab;
        if(determinant<=0)return;
        output[da]=(float)(bb/determinant);output[second==1 ? 3 : 5]=(float)(aa/determinant);output[first==0 ? (second==1 ? 1 : 2) : 4]=(float)(-ab/determinant);
    }
}
