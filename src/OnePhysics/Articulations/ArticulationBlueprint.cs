using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Articulations;

/// <summary>One link as the dimension-specific definitions describe it, in doubles, before validation: the common ground of the 2D and 3D definitions.</summary>
internal sealed class LinkBlueprint
{
    public int Parent = -1;
    public ArticulationJointKind Joint;
    public D3 Axis = new(1, 0, 0);
    public D3 Origin;
    public Q4 OriginRotation = Q4.Identity;
    public double Mass;
    public D3 Com;
    public M3 Inertia;
    public bool HasLimits;
    public double Lower, Upper, ConeLimit;
    public double Friction, Armature, Damping, MaxVelocity;
    public double Stiffness, DriveDamping, MaxForce, TargetPosition, TargetVelocity;
    public double InitialPosition;
    public Q4 InitialRotation = Q4.Identity;
    public Q4 TargetRotation = Q4.Identity;
}

internal sealed class ArticulationBlueprint
{
    public LinkBlueprint[] Links = Array.Empty<LinkBlueprint>();
    public bool FixedBase;
    public D3 BasePosition;
    public Q4 BaseRotation = Q4.Identity;
    public int Substeps = 4, Iterations = 8;
    public bool SelfCollision, GravityCompensation;
    public double GravityScale = 1.0;

    // The public entry points take a definition named "def"; the name is carried in a field so the argument checks name it.
    private static readonly string DefParam = "def";

    private static ArgumentOutOfRangeException Bad(int link, string member, object value, string rule) =>
        new(DefParam, value, "Link " + link.ToString(CultureInfo.InvariantCulture) + " " + member + " = " + Convert.ToString(value, CultureInfo.InvariantCulture) + ": " + rule);

    private static bool Fin(double v, double limit) => Math.Abs(v) <= limit;   // false for NaN

    /// <summary>Checks everything the model needs and normalizes axes and rotations; throws <see cref="ArgumentOutOfRangeException"/> or <see cref="ArgumentException"/> naming the link and the member.</summary>
    public void Validate(bool planar, int maxLinks)
    {
        if (Links.Length < 1 || Links.Length > maxLinks) { throw new ArgumentException("An articulation needs 1 to " + maxLinks.ToString(CultureInfo.InvariantCulture) + " links; the definition has " + Links.Length.ToString(CultureInfo.InvariantCulture) + ".", DefParam); }
        if (Substeps < 1 || Substeps > 64) { throw new ArgumentOutOfRangeException(DefParam, Substeps, "Substeps must be within 1..64."); }
        if (Iterations < 0 || Iterations > 256) { throw new ArgumentOutOfRangeException(DefParam, Iterations, "Iterations must be within 0..256."); }
        if (!Fin(GravityScale, 1e6)) { throw new ArgumentOutOfRangeException(DefParam, GravityScale, "GravityScale must be finite."); }
        if (!Fin(BasePosition.X, 2097152) || !Fin(BasePosition.Y, 2097152) || !Fin(BasePosition.Z, 2097152)) { throw new ArgumentOutOfRangeException(DefParam, BasePosition.X, "The base position must be finite and within ±2^21."); }
        BaseRotation = NormalizeRotation(BaseRotation, -1, "base rotation");
        for (int i = 0; i < Links.Length; i++)
        {
            var l = Links[i] ?? throw new ArgumentException("Link " + i.ToString(CultureInfo.InvariantCulture) + " is null.", DefParam);
            if (i == 0 ? l.Parent != -1 : (l.Parent < 0 || l.Parent >= i)) { throw Bad(i, "Parent", l.Parent, i == 0 ? "the first link is the root and has parent -1." : "a link's parent must be an earlier link."); }
            if (i > 0 && !Enum.IsDefined(l.Joint)) { throw Bad(i, "Joint", (int)l.Joint, "unknown joint kind."); }
            if (i > 0 && planar && l.Joint == ArticulationJointKind.Spherical) { throw Bad(i, "Joint", "Spherical", "spherical joints exist only in 3D."); }
            if (!Fin(l.Origin.X, 1e6) || !Fin(l.Origin.Y, 1e6) || !Fin(l.Origin.Z, 1e6)) { throw Bad(i, "Origin", l.Origin.X, "must be finite and within ±1e6."); }
            if (planar && l.Origin.Z != 0.0) { throw Bad(i, "Origin.Z", l.Origin.Z, "a planar articulation lives in the plane."); }
            l.OriginRotation = NormalizeRotation(l.OriginRotation, i, "OriginRotation");
            if (!Fin(l.Mass, 1e12) || l.Mass < 0.0) { throw Bad(i, "Mass", l.Mass, "must be finite and at least 0 (0 derives it from the shapes)."); }
            if (!Fin(l.Com.X, 1e6) || !Fin(l.Com.Y, 1e6) || !Fin(l.Com.Z, 1e6)) { throw Bad(i, "CenterOfMass", l.Com.X, "must be finite."); }
            CheckInertia(i, l);
            if (l.Joint == ArticulationJointKind.Revolute || l.Joint == ArticulationJointKind.Prismatic)
            {
                double len = l.Axis.Length;
                if (!(len > 1e-9) || !Fin(len, 1e9)) { throw Bad(i, "Axis", len, "the joint axis must be finite and non-zero."); }
                l.Axis = l.Axis * (1.0 / len);
                if (planar && l.Joint == ArticulationJointKind.Revolute && (Math.Abs(l.Axis.X) > 1e-6 || Math.Abs(l.Axis.Y) > 1e-6)) { throw Bad(i, "Axis", l.Axis.X, "a planar revolute joint turns about z."); }
                if (planar && l.Joint == ArticulationJointKind.Prismatic && Math.Abs(l.Axis.Z) > 1e-6) { throw Bad(i, "Axis", l.Axis.Z, "a planar prismatic joint slides in the plane."); }
            }

            if (l.HasLimits && !(l.Lower <= l.Upper && Fin(l.Lower, 1e9) && Fin(l.Upper, 1e9))) { throw Bad(i, "Limits", l.Lower, "lower must not exceed upper, both finite."); }
            if (!Fin(l.ConeLimit, 6.3) || l.ConeLimit < 0.0) { throw Bad(i, "SwingLimit", l.ConeLimit, "must be within 0..2π."); }
            foreach (var (name, v) in new[] { ("Friction", l.Friction), ("Armature", l.Armature), ("Damping", l.Damping), ("MaxVelocity", l.MaxVelocity), ("Drive.Stiffness", l.Stiffness), ("Drive.Damping", l.DriveDamping), ("Drive.MaxForce", l.MaxForce) })
            {
                if (!(v >= 0.0) || !Fin(v, 1e15)) { throw Bad(i, name, v, "must be finite and at least 0."); }
            }

            if (!Fin(l.TargetPosition, 1e9) || !Fin(l.TargetVelocity, 1e9) || !Fin(l.InitialPosition, 1e9)) { throw Bad(i, "target or initial position", l.TargetPosition, "must be finite."); }
            l.InitialRotation = NormalizeRotation(l.InitialRotation, i, "InitialRotation");
            l.TargetRotation = NormalizeRotation(l.TargetRotation, i, "TargetRotation");
            if (planar && (Math.Abs(l.OriginRotation.X) > 1e-6 || Math.Abs(l.OriginRotation.Y) > 1e-6)) { throw Bad(i, "OriginRotation", l.OriginRotation.X, "a planar articulation rotates about z only."); }
        }

        if (planar && (Math.Abs(BaseRotation.X) > 1e-6 || Math.Abs(BaseRotation.Y) > 1e-6)) { throw new ArgumentOutOfRangeException(DefParam, BaseRotation.X, "A planar articulation rotates about z only."); }
    }

    private static Q4 NormalizeRotation(Q4 q, int link, string what)
    {
        double l2 = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        if (!(l2 > 1e-24) || !Fin(l2, 1e12)) { throw new ArgumentOutOfRangeException(DefParam, q.W, (link < 0 ? "The " : "Link " + link.ToString(CultureInfo.InvariantCulture) + " ") + what + " must be a finite, non-zero quaternion."); }
        return q.Normalized();
    }

    // The tensor about the centre of mass must be symmetric positive semi-definite (Sylvester's criterion with a small slack for rounding); a zero tensor is a point mass and is allowed.
    private static void CheckInertia(int i, LinkBlueprint l)
    {
        var t = l.Inertia;
        double scale = Math.Max(Math.Max(Math.Abs(t.A), Math.Abs(t.E)), Math.Abs(t.I));
        if (!Fin(t.A, 1e12) || !Fin(t.B, 1e12) || !Fin(t.C, 1e12) || !Fin(t.E, 1e12) || !Fin(t.F, 1e12) || !Fin(t.I, 1e12)) { throw Bad(i, "InertiaTensor", t.A, "must be finite."); }
        double slack = 1e-6 * scale;
        bool ok = t.A >= -slack && t.E >= -slack && t.I >= -slack
            && t.A * t.E - t.B * t.B >= -slack * scale && t.A * t.I - t.C * t.C >= -slack * scale && t.E * t.I - t.F * t.F >= -slack * scale
            && t.A * (t.E * t.I - t.F * t.F) - t.B * (t.B * t.I - t.F * t.C) + t.C * (t.B * t.F - t.E * t.C) >= -slack * scale * scale;
        if (!ok) { throw Bad(i, "InertiaTensor", t.A, "must be symmetric positive semi-definite."); }
        l.Inertia = new M3(t.A, t.B, t.C, t.B, t.E, t.F, t.C, t.F, t.I);
    }

    /// <summary>Builds the model: specifications, parameters and the initial joint positions. Throws <see cref="ArgumentException"/> when the joint-space mass matrix is not positive definite.</summary>
    public ArticulationModel BuildModel(bool planar)
    {
        int n = Links.Length;
        var specs = new LinkSpec[n];
        for (int i = 0; i < n; i++)
        {
            var l = Links[i];
            var s = new LinkSpec { Parent = l.Parent, Mass = l.Mass, Com = l.Com, Inertia = l.Inertia, Axis = l.Axis, PlacementRot = l.OriginRotation, PlacementPos = l.Origin };
            if (i == 0)
            {
                s.Kind = FixedBase ? JointKind.Fixed : planar ? JointKind.FreePlanar : JointKind.Free;
                s.PlacementRot = BaseRotation; s.PlacementPos = BasePosition;
            }
            else { s.Kind = (JointKind)(byte)l.Joint; }
            specs[i] = s;
        }

        var m = new ArticulationModel(specs);
        for (int i = 0; i < n; i++)
        {
            var l = Links[i];
            m.HasLimit[i] = l.HasLimits && i > 0; m.Lower[i] = l.Lower; m.Upper[i] = l.Upper; m.ConeLimit[i] = l.ConeLimit;
            m.Friction[i] = l.Friction; m.Armature[i] = l.Armature; m.JointDamping[i] = l.Damping; m.MaxVel[i] = l.MaxVelocity;
            m.Stiff[i] = l.Stiffness; m.Damp[i] = l.DriveDamping; m.MaxForce[i] = l.MaxForce; m.Target[i] = l.TargetPosition; m.TargetVel[i] = l.TargetVelocity; m.TargetRot[i] = l.TargetRotation;
            int po = m.PosOff[i];
            if (i == 0)
            {
                if (!FixedBase)
                {
                    m.Q[po] = BasePosition.X; m.Q[po + 1] = BasePosition.Y; m.Q[po + 2] = BasePosition.Z;
                    m.Q[po + 3] = BaseRotation.X; m.Q[po + 4] = BaseRotation.Y; m.Q[po + 5] = BaseRotation.Z; m.Q[po + 6] = BaseRotation.W;
                }
            }
            else if (m.Kind[i] == JointKind.Revolute || m.Kind[i] == JointKind.Prismatic) { m.Q[po] = l.InitialPosition; }
            else if (m.Kind[i] == JointKind.Spherical) { m.Q[po] = l.InitialRotation.X; m.Q[po + 1] = l.InitialRotation.Y; m.Q[po + 2] = l.InitialRotation.Z; m.Q[po + 3] = l.InitialRotation.W; }
        }

        m.ForwardKinematics();
        // The joint-space mass matrix must be positive definite or the dynamics cannot be solved: a link of no mass, or a point mass sitting on the axis it is meant to turn about, would make it singular.
        int dofs = m.TotalDof;
        var mm = new double[Math.Max(dofs * dofs, 1)];
        m.MassMatrix(mm);
        for (int d = 0; d < dofs; d++) { mm[d * dofs + d] += 1e-12 + 1e-12 * Math.Abs(mm[d * dofs + d]); }
        if (!Dense.Cholesky(mm, dofs)) { throw new ArgumentException("The links' masses and inertias leave the joint-space mass matrix singular (a link with no inertia about its own joint axis, or masses that are not positive); give the links positive masses and inertia about each axis they turn on.", DefParam); }
        return m;
    }
}
