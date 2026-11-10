using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

/// <summary>Raw rod particle and segment storage. Mutable spans are single-writer, unvalidated access; do not retain them across restore. One frame per segment, tangent along its local X axis.</summary>
public sealed partial class RodParticles
{
    internal Vector3D[] PositionData, Previous, PinTarget, PinStart;
    internal Vector3F[] VelocityData, AngularVelocity;
    internal QuaternionF[] FrameData, OldFrame;
    internal float[] Length, InverseMass, InverseBendInertia, InverseTwistInertia;
    internal Vector3F[] RestBend;
    internal bool[] Pinned, Clamped;
    internal int[] AttachmentBody, AttachmentGeneration;
    internal Vector3F[] AttachmentLocal, AttachmentForce, AttachmentLambda;
    internal float[] AttachmentBreak;
    internal bool[] AttachmentBroken;
    internal float Friction = 0.4f, Restitution;
    internal bool CollisionEnabled = true, SelfCollision = true, CollideWithAttachedBodies;
    internal uint Category = 1, Mask = uint.MaxValue;
    internal int Layer, Group;
    internal readonly RodMaterial Material;
    internal readonly bool Planar;
    internal readonly bool ClampStart;
    internal QuaternionF RootFrame;
    internal int Substeps, Iterations;
    internal float Damping, GravityScale;
    private readonly Row[] _rows;
    private readonly double[] _matrix, _rhs;
    internal readonly double[] LambdaData;
    private const int Width = 12;

    /// <summary>Particle world positions, metres. Same count for the lifetime of this storage.</summary>
    public Span<Vector3D> Positions => PositionData;
    /// <summary>Particle linear velocities, m/s.</summary>
    public Span<Vector3F> Velocities => VelocityData;
    /// <summary>Segment material frames, unit quaternions. A planar frame rotates only about Z.</summary>
    public Span<QuaternionF> Frames => FrameData;
    /// <summary>Segment angular velocities in world coordinates, rad/s.</summary>
    public Span<Vector3F> AngularVelocities => AngularVelocity;
    /// <summary>Undeformed segment lengths, metres.</summary>
    public ReadOnlySpan<float> RestLengths => Length;
    /// <summary>Undeformed discrete Darboux vectors, twice the vector part of the relative frame quaternion.</summary>
    public ReadOnlySpan<Vector3F> RestDarbouxVectors => RestBend;
    /// <summary>Last substep's XPBD multipliers, in six-row groups (boundary first, then segment stretch XYZ, twist X, bend YZ). Read-only.</summary>
    public ReadOnlySpan<double> Multipliers => LambdaData;
    /// <summary>Current elastic strain energy in joules, including stretch, shear, bend, twist and the half-cell root clamp. Derived in fixed constraint-row order; hard pins/clamps store no spring energy. Single-writer access only.</summary>
    public double ElasticEnergy
    {
        get
        {
            BuildRows(1);double energy=0;
            foreach(var row in _rows)energy+=.5*row.C*row.C/row.Alpha;
            return energy;
        }
    }
    /// <summary>Maximum absolute relative length error, dimensionless; zero for an undeformed straight rod.</summary>
    public double MaximumStretch
    {
        get
        {
            double error = 0;
            for (int i = 0; i < Length.Length; i++)
            {
                var d = PositionData[i+1] - PositionData[i];
                error = Math.Max(error, Math.Abs(Math.Sqrt(d.X*d.X+d.Y*d.Y+d.Z*d.Z)/Length[i]-1));
            }
            return error;
        }
    }

    internal RodParticles(Vector3D[] points, RodMaterial material, bool planar, int[] pins, int[] clamps, Vector3F velocity, int substeps, int iterations, float damping, float gravityScale, bool clampStart = false)
    {
        Material = material; Planar = planar; ClampStart = clampStart; Substeps = substeps; Iterations = iterations; Damping = damping; GravityScale = gravityScale;
        PositionData = (Vector3D[])points.Clone(); Previous = (Vector3D[])points.Clone();
        for (int i = 0; i < PositionData.Length; i++)
        {
            var p = PositionData[i]; PositionData[i] = new Vector3D(WorldCore.Grid(p.X),WorldCore.Grid(p.Y),WorldCore.Grid(p.Z));
        }
        Array.Copy(PositionData,Previous,PositionData.Length);
        PinTarget = (Vector3D[])PositionData.Clone(); PinStart = (Vector3D[])PositionData.Clone();
        int n = points.Length, m = n-1;
        AttachmentBody = new int[n]; Array.Fill(AttachmentBody,-1); AttachmentGeneration = new int[n];
        AttachmentLocal = new Vector3F[n]; AttachmentForce = new Vector3F[n]; AttachmentLambda = new Vector3F[n]; AttachmentBreak = new float[n]; Array.Fill(AttachmentBreak,float.MaxValue); AttachmentBroken = new bool[n];
        VelocityData = new Vector3F[n]; Array.Fill(VelocityData, velocity);
        AngularVelocity = new Vector3F[m]; FrameData = new QuaternionF[m]; OldFrame = new QuaternionF[m];
        Length = new float[m]; InverseMass = new float[n]; InverseBendInertia = new float[m]; InverseTwistInertia = new float[m];
        RestBend = new Vector3F[m]; Pinned = new bool[n]; Clamped = new bool[m];
        foreach (int i in pins) Pinned[i] = true;
        foreach (int i in clamps) Clamped[i] = true;
        double area = 3.141592653589793 * material.Radius * material.Radius;
        for (int i = 0; i < m; i++)
        {
            var d = points[i+1]-points[i]; float l = (float)Math.Sqrt(d.X*d.X+d.Y*d.Y+d.Z*d.Z); Length[i] = l;
            var t = new Vector3F((float)(d.X/l), (float)(d.Y/l), (float)(d.Z/l));
            FrameData[i] = t.X < -0.999999f ? new QuaternionF(0,0,1,0) : Unit(new QuaternionF(0,-t.Z,t.Y,1+t.X));
            float mass = (float)(area*material.Density*l);
            InverseMass[i] += mass*0.5f; InverseMass[i+1] += mass*0.5f;
            InverseBendInertia[i] = Clamped[i] ? 0 : 1/(mass*(l*l/12+material.Radius*material.Radius/4));
            InverseTwistInertia[i] = Clamped[i] ? 0 : 2/(mass*material.Radius*material.Radius);
        }
        for (int i = 0; i < n; i++) InverseMass[i] = Pinned[i] ? 0 : 1/InverseMass[i];
        for (int i = 0; i < m-1; i++)
        {
            var q = Conjugate(FrameData[i])*FrameData[i+1]; if (q.W < 0) q = Negate(q);
            RestBend[i] = new Vector3F(2*q.X,2*q.Y,2*q.Z);
        }
        RootFrame = FrameData[0];
        FrameData.CopyTo(OldFrame,0);
        _rows = new Row[(m+1)*6]; _matrix = new double[(m+1)*6*Width]; _rhs = new double[(m+1)*6]; LambdaData = new double[(m+1)*6];
    }

    private struct Row
    {
        internal Vector3F PA, PB, QA, QB;
        internal Vector3D WQA, WQB;
        internal double C, Alpha;
    }
    internal static QuaternionF Conjugate(QuaternionF q) => new(-q.X,-q.Y,-q.Z,q.W);
    private static QuaternionF Negate(QuaternionF q) => new(-q.X,-q.Y,-q.Z,-q.W);
    internal static QuaternionF Unit(QuaternionF q)
    {
        float s = 1/Sqrt(q.X*q.X+q.Y*q.Y+q.Z*q.Z+q.W*q.W);
        return new(q.X*s,q.Y*s,q.Z*s,q.W*s);
    }
    private static float Sqrt(float v) => (float)Math.Sqrt(v);
    internal static Vector3F Rotate(QuaternionF q, Vector3F v)
    {
        var u = new Vector3F(q.X,q.Y,q.Z);
        var t = Cross(u,v)*2;
        return v + t*q.W + Cross(u,t);
    }
    internal static Vector3F Cross(Vector3F a, Vector3F b) => new(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
    private static double Dot(Vector3F a, Vector3F b) => (double)a.X*b.X+(double)a.Y*b.Y+(double)a.Z*b.Z;
    private static double Dot(Vector3F a, Vector3D b) => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
    private static Vector3F Axis(int i) => i == 0 ? new(1,0,0) : i == 1 ? new(0,1,0) : new(0,0,1);
    private static float Component(QuaternionF q, int i) => i == 0 ? q.X : i == 1 ? q.Y : q.Z;
    private Vector3D AngularMass(int i, Vector3F v)
    {
        if (Clamped[i]) return default;
        if (Planar) return new(0,0,(double)v.Z*InverseBendInertia[i]);
        var tangent = Rotate(FrameData[i],new(1,0,0));
        double transverse = InverseBendInertia[i];
        double axial = Dot(tangent,v)*((double)InverseTwistInertia[i]-transverse);
        return new Vector3D(v.X*transverse+tangent.X*axial,v.Y*transverse+tangent.Y*axial,v.Z*transverse+tangent.Z*axial);
    }
    internal Vector3F InverseAngularImpulse(int segment,Vector3F impulse)
    {var x=AngularMass(segment,impulse);return new((float)x.X,(float)x.Y,(float)x.Z);}
    internal static QuaternionF Turn(QuaternionF q, Vector3F theta)
    {
        var d = new QuaternionF(theta.X*0.5f,theta.Y*0.5f,theta.Z*0.5f,0)*q;
        return Unit(new QuaternionF(q.X+d.X,q.Y+d.Y,q.Z+d.Z,q.W+d.W));
    }

    // Kugelstadt & Schömer 2016, strain measures, with XPBD compliance (Macklin et al. 2016).
    // Each group touches two points and two consecutive frames: J W J^T is block tridiagonal.
    // A fixed-order band Cholesky solve avoids the length-dependent propagation of local projections.
    private void BuildRows(float h)
    {
        double r2 = (double)Material.Radius*Material.Radius, area = Math.PI*r2, second = area*r2/4;
        double h2 = (double)h*h;
        int n = Length.Length;
        for (int k = 0; k < 6; k++) { _rows[k] = default; _rows[k].Alpha = 1; }
        if (ClampStart)
        {
            var relative = Conjugate(RootFrame)*FrameData[0]; float sign = relative.W < 0 ? -1 : 1;
            for (int k = 0; k < 3; k++)
            {
                if (Planar && k != 2) continue;
                ref var row = ref _rows[3+k]; row = default;
                var bx = Conjugate(RootFrame)*new QuaternionF(1,0,0,0)*FrameData[0];
                var by = Conjugate(RootFrame)*new QuaternionF(0,1,0,0)*FrameData[0];
                var bz = Conjugate(RootFrame)*new QuaternionF(0,0,1,0)*FrameData[0];
                row.QB = new Vector3F(sign*Component(bx,k),sign*Component(by,k),sign*Component(bz,k));
                row.WQB = AngularMass(0,row.QB); row.C = 2*sign*Component(relative,k);
                row.Alpha = Length[0]*0.5/((k == 0 ? 2*Material.ShearModulus : Material.YoungModulus)*second*h2);
            }
        }
        for (int i = 0; i < n; i++)
        {
            var q = FrameData[i]; var eD = PositionData[i+1]-PositionData[i];
            var e = new Vector3F((float)eD.X,(float)eD.Y,(float)eD.Z);
            for (int k = 0; k < 3; k++)
            {
                ref var row = ref _rows[(i+1)*6+k]; row = default;
                if (Planar && k == 2) { row.Alpha = 1; continue; }
                var d = Rotate(q,Axis(k));
                row.PA = d*-1; row.PB = d; row.QA = Cross(d,e);
                row.WQA = AngularMass(i,row.QA);
                row.C = Dot(d,e)-(k == 0 ? Length[i] : 0);
                row.Alpha = Length[i]/((k == 0 ? Material.YoungModulus : Material.ShearModulus)*area*h2);
            }
            var relative = i+1 < n ? Conjugate(q)*FrameData[i+1] : QuaternionF.Identity;
            float sign = relative.W < 0 ? -1 : 1;
            for (int k = 0; k < 3; k++)
            {
                ref var row = ref _rows[(i+1)*6+3+k]; row = default;
                if (i+1 == n || (Planar && k != 2)) { row.Alpha = 1; continue; }
                // d(2 Im(conj(qA) qB))/d thetaB = Im(conj(qA) (thetaB,0) qB).
                var bx = Conjugate(q)*new QuaternionF(1,0,0,0)*FrameData[i+1];
                var by = Conjugate(q)*new QuaternionF(0,1,0,0)*FrameData[i+1];
                var bz = Conjugate(q)*new QuaternionF(0,0,1,0)*FrameData[i+1];
                row.QB = new Vector3F(sign*Component(bx,k),sign*Component(by,k),sign*Component(bz,k));
                row.QA = row.QB*-1;
                row.WQA = AngularMass(i,row.QA); row.WQB = AngularMass(i+1,row.QB);
                var rest = RestBend[i]; float restK = k == 0 ? rest.X : k == 1 ? rest.Y : rest.Z;
                row.C = 2*sign*Component(relative,k)-restK;
                row.Alpha = (Length[i]+Length[i+1])*0.5/((k == 0 ? 2*Material.ShearModulus : Material.YoungModulus)*second*h2);
            }
        }
    }

    private void Solve(float h)
    {
        BuildRows(h); Array.Clear(_matrix);
        // Implicit drag scales the response to constraint forces as well as gravity.
        // Scaling only the prediction would bias static compliance by 1/(1+damping*h).
        double response=1/(1+(double)Damping*h);
        int groups = Length.Length+1, n = groups*6;
        for (int g = 0; g < groups; g++)
        {
            for (int a = 0; a < 6; a++)
            {
                int i = g*6+a; ref var x = ref _rows[i];
                _rhs[i] = -x.C-x.Alpha*LambdaData[i];
                for (int b = 0; b <= a; b++)
                {
                    int j = g*6+b; ref var y = ref _rows[j];
                    _matrix[i*Width+i-j] = ((g > 0 ? InverseMass[g-1]*Dot(x.PA,y.PA)+InverseMass[g]*Dot(x.PB,y.PB) : 0)+Dot(x.QA,y.WQA)+Dot(x.QB,y.WQB))*response+(a == b ? x.Alpha : 0);
                }
                if (g == 0) continue;
                for (int b = 0; b < 6; b++)
                {
                    int j = (g-1)*6+b; ref var y = ref _rows[j];
                    _matrix[i*Width+i-j] = ((g > 1 ? InverseMass[g-1]*Dot(x.PA,y.PB) : 0)+Dot(x.QA,y.WQB))*response;
                }
            }
        }
        // Lower-band Cholesky; bandwidth 11 is a consequence of the six-row chain graph, not input depth.
        for (int i = 0; i < n; i++)
        {
            int first = Math.Max(0,i-Width+1);
            for (int j = first; j <= i; j++)
            {
                double sum = _matrix[i*Width+i-j];
                for (int k = first; k < j; k++) sum -= _matrix[i*Width+i-k]*_matrix[j*Width+j-k];
                _matrix[i*Width+i-j] = i == j ? Math.Sqrt(Math.Max(sum,1e-30)) : sum/_matrix[j*Width];
            }
            for (int j = first; j < i; j++) _rhs[i] -= _matrix[i*Width+i-j]*_rhs[j];
            _rhs[i] /= _matrix[i*Width];
        }
        for (int i = n-1; i >= 0; i--)
        {
            for (int j = i+1; j < Math.Min(n,i+Width); j++) _rhs[i] -= _matrix[j*Width+j-i]*_rhs[j];
            _rhs[i] /= _matrix[i*Width]; LambdaData[i] += _rhs[i];
        }
        // Accumulate corrections per degree of freedom in row order, then apply once.
        for (int p = 0; p < groups; p++)
        {
            Vector3D dx = default;
            for (int g = Math.Max(1,p); g <= Math.Min(p+1,groups-1); g++)
                for (int k = 0; k < 6; k++)
                {
                    int j = g*6+k; var grad = g == p+1 ? _rows[j].PA : _rows[j].PB; double s = InverseMass[p]*response*_rhs[j];
                    dx += new Vector3D(grad.X*s,grad.Y*s,grad.Z*s);
                }
            PositionData[p] += new Vector3D(WorldCore.Grid(dx.X),WorldCore.Grid(dx.Y),WorldCore.Grid(dx.Z));
        }
        for (int p = 0; p < groups-1; p++)
        {
            Vector3D dt = default;
            for (int g = p; g <= p+1; g++)
                for (int k = 0; k < 6; k++)
                {
                    int j = g*6+k; var grad = g == p+1 ? _rows[j].WQA : _rows[j].WQB; double s = response*_rhs[j];
                    dt += new Vector3D(grad.X*s,grad.Y*s,grad.Z*s);
                }
            FrameData[p] = Turn(FrameData[p],new Vector3F((float)dt.X,(float)dt.Y,(float)dt.Z));
        }
    }

    internal void SetPinned(int particle, bool pinned)
    {
        AttachmentBody[particle]=-1;AttachmentGeneration[particle]=0;AttachmentBroken[particle]=false;AttachmentForce[particle]=default;
        Pinned[particle] = pinned; PinTarget[particle] = PositionData[particle]; PinStart[particle] = PositionData[particle];
        double area = Math.PI*Material.Radius*Material.Radius;
        float mass = 0;
        if (particle > 0) mass += (float)(area*Material.Density*Length[particle-1])*0.5f;
        if (particle < Length.Length) mass += (float)(area*Material.Density*Length[particle])*0.5f;
        InverseMass[particle] = pinned ? 0 : 1/mass;
    }

    internal void Predict(float h, Vector3F gravity, float fraction)
    {
        Array.Clear(LambdaData);Array.Clear(AttachmentLambda);
        for (int i = 0; i < PositionData.Length; i++)
        {
            Previous[i] = PositionData[i];
            if (Pinned[i])
            {
                var target = fraction == 1 ? PinTarget[i] : PinStart[i]+(PinTarget[i]-PinStart[i])*fraction;
                PositionData[i] = new Vector3D(WorldCore.Grid(target.X),WorldCore.Grid(target.Y),WorldCore.Grid(target.Z));
                continue;
            }
            var v = (VelocityData[i]+gravity*(GravityScale*h))/(1+Damping*h); VelocityData[i] = v;
            PositionData[i] += new Vector3D(WorldCore.Grid((double)v.X*h),WorldCore.Grid((double)v.Y*h),WorldCore.Grid((double)v.Z*h));
        }
        for (int i = 0; i < FrameData.Length; i++)
        {
            OldFrame[i] = FrameData[i];
            if (Clamped[i]) { AngularVelocity[i] = default; continue; }
            AngularVelocity[i] /= 1+Damping*h;
            FrameData[i] = Turn(FrameData[i],AngularVelocity[i]*h);
        }
    }
    internal void Project(float h) => Solve(h);
    internal void Finish(float h)
    {
        for (int i = 0; i < PositionData.Length; i++)
        {
            var d = (PositionData[i]-Previous[i])/h;
            VelocityData[i] = new Vector3F((float)d.X,(float)d.Y,(float)d.Z);
        }
        for (int i = 0; i < FrameData.Length; i++)
        {
            var dq = FrameData[i]*Conjugate(OldFrame[i]); double scale = (dq.W < 0 ? -2.0 : 2.0)/h;
            AngularVelocity[i] = Clamped[i] ? default : new Vector3F((float)(dq.X*scale),(float)(dq.Y*scale),(float)(dq.Z*scale));
        }
    }
}
