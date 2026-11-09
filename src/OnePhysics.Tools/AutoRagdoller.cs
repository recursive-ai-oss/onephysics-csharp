using System;
using RecursiveAI.OneMath;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// Makes a ragdoll definition from a skeleton: a bone hierarchy with bind-pose joint positions and, optionally, a skinned mesh. The skeleton is in the engine's frame: y up, a person faces +z in 3D (their left is +x)
/// and faces <see cref="AutoRagdollOptions.Facing"/> along x in 2D.
/// </summary>
/// <remarks>
/// <para><b>Roles.</b> Each bone gets a <see cref="BoneRole"/> from its name (the last component after ':' or '|', split into words at separators and case changes, so "mixamorig:LeftUpLeg", "UpperArm_L" and
/// "thigh.R" all read): twist, roll, finger, clavicle, end-marker and root bones fold into their parent; "forearm"/"lowerarm", "upperarm"/"arm", "thigh"/"upleg", "calf"/"shin", "foot", "toe", "hand", "head", "neck",
/// "spine"/"chest"/"abdomen" and "hips"/"pelvis" name the person's parts; a bare "leg" is a thigh under the hips and a shin under a thigh. <see cref="AutoRagdollOptions.RoleOverrides"/> overrides any name.
/// A skeleton with a pelvis, a torso, two legs (thigh, shin, foot) and optionally a head and two arms is a humanoid: 13 parts (15 with hands, one more with a split torso), the left leg first, parts parents-first, and
/// <see cref="AutoRagdoll3D.Biped"/> names the bones the balance and walking controllers need. Any other skeleton gets one capsule per bone.</para>
/// <para><b>Shapes.</b> With skin points, each point belongs to the part whose bones weigh most on it (ties to the lower part). Limbs are capsules along the bone: the radius is the median distance of the part's
/// points from the axis and the ends are the extreme points along it; pelvis, torso, hands and feet are boxes on the world axes around their points (feet stay axis-aligned, which the standing controller assumes) and
/// the head a sphere at the centre of its points with the median distance as radius. A generic part whose cross-section is flat (principal moments across the bone differing by more than 4 to 1) becomes a box on the
/// principal axes of its points (onemath's symmetric eigen decomposition). Without skin, proportions of an adult scaled to the skeleton's height are used.</para>
/// <para><b>Mass.</b> A humanoid shares <see cref="AutoRagdollOptions.TotalMass"/> by Winter's segment table (Dempster): pelvis 14.2 %, trunk 35.5 % (abdomen 13.9 % and chest 21.6 % when split), head and neck 8.1 %,
/// each upper arm 2.8 %, forearm 1.6 % (plus the hand's 0.6 %), thigh 10 %, shank 4.65 %, foot 1.45 %. Any other skeleton shares it in proportion to shape volume times density.</para>
/// <para><b>Joints.</b> Limits and drive gains follow the role (ranges of motion after the AAOS and Kapandji tables; the drive gains of the engine's own humanoid, scaled by mass and height): spine ±0.4 twist and 0.5 swing,
/// head 0.6 twist and swing, upper arm 1.0 twist and 1.6 swing, elbow 0.3 twist, 2.4 flexion, 0.1 abduction, wrist 1.3 flexion, thigh 0.6, 1.2, 1.0, knee 0.1, 2.3, 0.08, ankle 0.3, 0.6, 0.4 (radians). In 2D the revolute
/// limits are the same ranges, flipped for a figure facing -x.</para>
/// <para>The result is a pure function of the input: the same bits on every platform and thread count.</para>
/// </remarks>
public static class AutoRagdoller
{
    // The engine's own humanoid is 1.66 m from heel to crown whatever height it is asked for; its drive gains (per kilogram and metre) are the reference.
    private const double ReferenceHeight = 1.66;

    private readonly struct Limits
    {
        public Limits(float twist, float swingY, float swingZ, float k, float d, float max) { Twist = twist; SwingY = swingY; SwingZ = swingZ; K = k; D = d; Max = max; }
        public float Twist { get; }
        public float SwingY { get; }
        public float SwingZ { get; }
        public float K { get; }
        public float D { get; }
        public float Max { get; }
    }

    private static Limits Table3D(PartFit p, bool humanoid)
    {
        if (!humanoid) { return new Limits(0.5f, 0.8f, 0.8f, 800f, 60f, 200f); }
        return p.Kind switch
        {
            BoneRole.Chest => p.CombinedTorso ? new Limits(0.4f, 0.5f, 0.5f, 6000f, 400f, 900f) : new Limits(0.3f, 0.4f, 0.4f, 6000f, 400f, 900f),
            BoneRole.Spine => new Limits(0.3f, 0.4f, 0.4f, 6000f, 400f, 900f),
            BoneRole.Head => new Limits(0.6f, 0.6f, 0.6f, 600f, 40f, 150f),
            BoneRole.UpperArm => new Limits(1.0f, 1.6f, 1.6f, 800f, 60f, 200f),
            BoneRole.Forearm => new Limits(0.3f, 2.4f, 0.1f, 500f, 40f, 120f),
            BoneRole.Hand => new Limits(0.3f, 1.3f, 0.4f, 200f, 15f, 40f),
            BoneRole.Thigh => new Limits(0.6f, 1.2f, 1.0f, 12000f, 800f, 1500f),
            BoneRole.Shin => new Limits(0.1f, 2.3f, 0.08f, 9000f, 600f, 1200f),
            BoneRole.Foot => new Limits(0.3f, 0.6f, 0.4f, 5000f, 300f, 600f),
            _ => new Limits(0.5f, 0.8f, 0.8f, 800f, 60f, 200f),
        };
    }

    // Revolute ranges for 2D (radians, counter-clockwise positive, a figure facing +x: forward bends are positive) and the largest drive torque.
    private static (float Lower, float Upper, float Max) Table2D(PartFit p, bool humanoid)
    {
        if (!humanoid) { return (-1.5f, 1.5f, 200f); }
        return p.Kind switch
        {
            BoneRole.Chest => (-0.5f, 0.5f, 900f),
            BoneRole.Spine => (-0.4f, 0.4f, 900f),
            BoneRole.Head => (-0.6f, 0.6f, 150f),
            BoneRole.UpperArm => (-1.2f, 3.0f, 200f),
            BoneRole.Forearm => (-0.05f, 2.5f, 120f),
            BoneRole.Hand => (-1.0f, 1.0f, 40f),
            BoneRole.Thigh => (-0.7f, 1.5f, 1500f),
            BoneRole.Shin => (-2.4f, 0.05f, 1200f),
            BoneRole.Foot => (-0.7f, 0.4f, 600f),
            _ => (-1.5f, 1.5f, 200f),
        };
    }

    private static bool UprightFrame(PartFit p, bool humanoid) => humanoid && p.Kind != BoneRole.UpperArm && p.Kind != BoneRole.Forearm && p.Kind != BoneRole.Hand;

    private static double Len(double x, double y, double z) => Math.Sqrt(x * x + y * y + z * z);

    // The perpendicular to d that is closest to +z (the way the figure faces).
    private static void Forward(double dx, double dy, double dz, out double fx, out double fy, out double fz)
    {
        double ax = 0, ay = 0, az = 1;
        double t = az * dz;
        fx = ax - t * dx; fy = ay - t * dy; fz = az - t * dz;
        double l = Len(fx, fy, fz);
        if (l < 1e-3)
        {
            ax = 0; ay = 1; az = 0;
            t = ay * dy;
            fx = ax - t * dx; fy = ay - t * dy; fz = az - t * dz;
            l = Len(fx, fy, fz);
        }

        fx /= l; fy /= l; fz /= l;
    }

    private static QuaternionF FromColumns(double x0, double x1, double x2, double y0, double y1, double y2, double z0, double z1, double z2)
    {
        var m = Matrix3x3F.FromColumns(new Vector3F((float)x0, (float)x1, (float)x2), new Vector3F((float)y0, (float)y1, (float)y2), new Vector3F((float)z0, (float)z1, (float)z2));
        return QuaternionF.CreateFromRotationMatrix(m).Normalized();
    }

    /// <summary>Makes a 3D ragdoll from a skeleton.</summary>
    /// <param name="boneNames">One name per bone, used for the role heuristics and as the part names.</param>
    /// <param name="parents">For each bone the index of its parent, or -1 for a root (at least one; several are allowed). No cycles.</param>
    /// <param name="bindPositions">Each bone's joint position in the bind pose, m, finite and within ±2^20, in the frame described on <see cref="AutoRagdoller"/>.</param>
    /// <param name="skin">Skinned mesh points to fit shapes to, or null to use proportions.</param>
    /// <param name="options">Total mass, mass model, hands, torso split, friction, role overrides; the default is a 75 kg person.</param>
    /// <returns>The ragdoll definition, the controller bone mapping when it is a biped, and the bone-to-part map.</returns>
    /// <exception cref="GeometryToolException">An input is empty, non-finite, out of range, inconsistent (array lengths, parents, cycles, skin indices) or an option is invalid; the message names the input and the position.</exception>
    public static AutoRagdoll3D Create(ReadOnlySpan<string> boneNames, ReadOnlySpan<int> parents, ReadOnlySpan<Vector3F> bindPositions, SkinnedPoints3D? skin = null, AutoRagdollOptions? options = null)
    {
        var opt = options ?? new AutoRagdollOptions();
        int n = boneNames.Length;
        var names = new string?[n];
        for (int i = 0; i < n; i++) { names[i] = boneNames[i]; }
        var par = parents.ToArray();
        if (bindPositions.Length != n) { throw new GeometryToolException(GeometryToolError.BadIndex, "bindPositions has " + MeshInput.Inv(bindPositions.Length) + " entries for " + MeshInput.Inv(n) + " bones", nameof(bindPositions)); }
        var x = new double[n]; var y = new double[n]; var z = new double[n];
        for (int i = 0; i < n; i++) { x[i] = bindPositions[i].X; y[i] = bindPositions[i].Y; z[i] = bindPositions[i].Z; }
        MeshData? mesh = null;
        if (skin is not null)
        {
            int nv = skin.Vertices.Length;
            mesh = new MeshData { X = new double[nv], Y = new double[nv], Z = new double[nv], Bones = skin.BoneIndices, Weights = skin.BoneWeights, K = skin.InfluencesPerVertex };
            for (int i = 0; i < nv; i++) { mesh.X[i] = skin.Vertices[i].X; mesh.Y[i] = skin.Vertices[i].Y; mesh.Z[i] = skin.Vertices[i].Z; }
        }

        var model = SkeletonFitter.Run(names, par, x, y, z, mesh, opt, planar: false);
        float g = (float)((opt.TotalMass / 75.0) * (model.Height / ReferenceHeight));
        var parts = new P3.RagdollPartDef[model.Parts.Count];
        for (int i = 0; i < parts.Length; i++)
        {
            var p = model.Parts[i];
            var lim = Table3D(p, model.Humanoid);
            var def = new P3.RagdollPartDef
            {
                Name = names[p.FirstBone] ?? ("part " + MeshInput.Inv(i)),
                Parent = p.Parent,
                Mass = (float)p.Mass,
                Friction = model.Humanoid && p.Kind == BoneRole.Foot ? Math.Max(opt.Friction, 1f) : opt.Friction,
                Position = new Vector3F((float)p.Cx, (float)p.Cy, (float)p.Cz),
                JointPosition = new Vector3F((float)p.Sx, (float)p.Sy, (float)p.Sz),
                LowerTwist = -lim.Twist, UpperTwist = lim.Twist, SwingYHalfAngle = Math.Max(lim.SwingY, opt.MinSwingHalfAngle), SwingZHalfAngle = Math.Max(lim.SwingZ, opt.MinSwingHalfAngle),
                DriveStiffness = lim.K * g, DriveDamping = lim.D * g, MaxTorque = lim.Max * g,
            };

            if (i == 0) { def.DriveStiffness = 0f; def.DriveDamping = 0f; def.MaxTorque = 0f; }
            switch (p.Shape)
            {
                case FitShape.Sphere:
                    def.Shape = P3.RagdollShapeKind.Sphere; def.ShapeHalfSize = new Vector3F((float)p.A, 0f, 0f);
                    break;
                case FitShape.Box:
                    def.Shape = P3.RagdollShapeKind.Box; def.ShapeHalfSize = new Vector3F((float)p.A, (float)p.B, (float)p.C);
                    if (p.Oriented) { def.Rotation = FromColumns(p.Ax0, p.Ay0, p.Az0, p.Ax1, p.Ay1, p.Az1, p.Ax2, p.Ay2, p.Az2); }
                    break;
                default:
                {
                    def.Shape = P3.RagdollShapeKind.Capsule; def.ShapeHalfSize = new Vector3F((float)p.A, (float)p.B, 0f);
                    // The capsule lies along the body's y axis; a part pointing downwards uses the upward end of its axis, so a hanging limb has the identity orientation.
                    double ax = p.Dx, ay = p.Dy, az = p.Dz;
                    if (ay < -1e-9 || (Math.Abs(ay) <= 1e-9 && (ax < -1e-9 || (Math.Abs(ax) <= 1e-9 && az < 0)))) { ax = -ax; ay = -ay; az = -az; }
                    Forward(ax, ay, az, out double fx, out double fy, out double fz);
                    // x = y cross z
                    double xx = ay * fz - az * fy, xy = az * fx - ax * fz, xz = ax * fy - ay * fx;
                    def.Rotation = FromColumns(xx, xy, xz, ax, ay, az, fx, fy, fz);
                    break;
                }
            }

            if (UprightFrame(p, model.Humanoid))
            {
                // The engine's humanoid frame: twist axis up, swing about the lateral axis (-x) and the forward axis.
                def.JointRotation = new QuaternionF(0f, 0f, 0.70710678f, 0.70710678f);
            }
            else
            {
                // Twist along the bone, taking the end of the axis that points up (the engine's own humanoid does, and a twist axis pointing down made an elbow unstable in testing);
                // the first swing axis is the lateral one, so flexion happens about y whichever way the limb points.
                double tx = p.Dx, ty = p.Dy, tz = p.Dz;
                if (ty < -1e-9 || (Math.Abs(ty) <= 1e-9 && (tx < -1e-9 || (Math.Abs(tx) <= 1e-9 && tz < 0)))) { tx = -tx; ty = -ty; tz = -tz; }
                Forward(tx, ty, tz, out double fx, out double fy, out double fz);
                double yx = fy * tz - fz * ty, yy = fz * tx - fx * tz, yz = fx * ty - fy * tx;
                def.JointRotation = FromColumns(tx, ty, tz, yx, yy, yz, fx, fy, fz);
            }

            parts[i] = def;
        }

        var ragdoll = new P3.RagdollDef { Parts = parts, SelfCollision = opt.SelfCollision, AngularDamping = 0.05f };
        BipedBones? biped = null;
        if (model.Humanoid && !(opt.SplitTorso && model.Parts[model.Torso].Kind == BoneRole.Spine))
        {
            biped = new BipedBones
            {
                Pelvis = model.Pelvis, Torso = model.Torso, Head = model.Head,
                LeftThigh = model.LeftThigh, LeftShin = model.LeftShin, LeftFoot = model.LeftFoot,
                RightThigh = model.RightThigh, RightShin = model.RightShin, RightFoot = model.RightFoot,
            };
        }

        return new AutoRagdoll3D(ragdoll, biped, model.PartOfBone, model.Roles, model.Humanoid, (float)model.Height);
    }

    /// <summary>Makes a 2D ragdoll from a skeleton in a side view (x along the ground, y up).</summary>
    /// <param name="boneNames">One name per bone.</param>
    /// <param name="parents">For each bone the index of its parent, or -1 for a root.</param>
    /// <param name="bindPositions">Each bone's joint position in the bind pose, m, finite and within ±2^20.</param>
    /// <param name="skin">Skinned points to fit shapes to, or null.</param>
    /// <param name="options">As for the 3D form; <see cref="AutoRagdollOptions.Facing"/> gives the direction the figure faces and the self-collision mode defaults to no collisions between parts (the arms and legs overlap in a side view).</param>
    /// <returns>The 2D ragdoll definition (see <see cref="Create"/> for the rest).</returns>
    /// <exception cref="GeometryToolException">As for the 3D form.</exception>
    public static AutoRagdoll2D Create2D(ReadOnlySpan<string> boneNames, ReadOnlySpan<int> parents, ReadOnlySpan<Vector2F> bindPositions, SkinnedPoints2D? skin = null, AutoRagdollOptions? options = null)
    {
        var opt = options ?? new AutoRagdollOptions();
        int n = boneNames.Length;
        var names = new string?[n];
        for (int i = 0; i < n; i++) { names[i] = boneNames[i]; }
        var par = parents.ToArray();
        if (bindPositions.Length != n) { throw new GeometryToolException(GeometryToolError.BadIndex, "bindPositions has " + MeshInput.Inv(bindPositions.Length) + " entries for " + MeshInput.Inv(n) + " bones", nameof(bindPositions)); }
        var x = new double[n]; var y = new double[n]; var z = new double[n];
        for (int i = 0; i < n; i++) { x[i] = bindPositions[i].X; y[i] = bindPositions[i].Y; }
        MeshData? mesh = null;
        if (skin is not null)
        {
            int nv = skin.Vertices.Length;
            mesh = new MeshData { X = new double[nv], Y = new double[nv], Z = new double[nv], Bones = skin.BoneIndices, Weights = skin.BoneWeights, K = skin.InfluencesPerVertex };
            for (int i = 0; i < nv; i++) { mesh.X[i] = skin.Vertices[i].X; mesh.Y[i] = skin.Vertices[i].Y; }
        }

        var model = SkeletonFitter.Run(names, par, x, y, z, mesh, opt, planar: true);
        float g = (float)((opt.TotalMass / 75.0) * (model.Height / ReferenceHeight));
        var parts = new P2.RagdollPartDef[model.Parts.Count];
        for (int i = 0; i < parts.Length; i++)
        {
            var p = model.Parts[i];
            var (lo, hi, max) = Table2D(p, model.Humanoid);
            if (opt.Facing < 0) { (lo, hi) = (-hi, -lo); }
            var def = new P2.RagdollPartDef
            {
                Name = names[p.FirstBone] ?? ("part " + MeshInput.Inv(i)),
                Parent = p.Parent,
                Mass = (float)p.Mass,
                Friction = model.Humanoid && p.Kind == BoneRole.Foot ? Math.Max(opt.Friction, 1f) : opt.Friction,
                Position = new Vector2F((float)p.Cx, (float)p.Cy),
                JointPosition = new Vector2F((float)p.Sx, (float)p.Sy),
                LowerAngle = lo, UpperAngle = hi, MaxTorque = max * g,
            };

            switch (p.Shape)
            {
                case FitShape.Sphere:
                    def.Shape = P2.RagdollShapeKind.Circle; def.ShapeHalfSize = new Vector2F((float)p.A, 0f);
                    break;
                case FitShape.Box:
                    def.Shape = P2.RagdollShapeKind.Box; def.ShapeHalfSize = new Vector2F((float)p.A, (float)p.B);
                    break;
                default:
                {
                    def.Shape = P2.RagdollShapeKind.Capsule; def.ShapeHalfSize = new Vector2F((float)p.A, (float)p.B);
                    double ax = p.Dx, ay = p.Dy;
                    if (ay < -1e-9 || (Math.Abs(ay) <= 1e-9 && ax < 0)) { ax = -ax; ay = -ay; }
                    def.Angle = (float)Math64.Atan2(ay, ax);
                    break;
                }
            }

            parts[i] = def;
        }

        // The arms and legs overlap in a side view, so a recognised person's parts never collide with each other unless asked to.
        var selfMode = model.Humanoid && opt.SelfCollision != P3.RagdollSelfCollision.All ? P2.RagdollSelfCollision.None : (P2.RagdollSelfCollision)(byte)opt.SelfCollision;
        var ragdoll = new P2.RagdollDef { Parts = parts, SelfCollision = selfMode, AngularDamping = 0.05f };
        BipedBones? biped = null;
        if (model.Humanoid && !(opt.SplitTorso && model.Parts[model.Torso].Kind == BoneRole.Spine))
        {
            biped = new BipedBones
            {
                Pelvis = model.Pelvis, Torso = model.Torso, Head = model.Head,
                LeftThigh = model.LeftThigh, LeftShin = model.LeftShin, LeftFoot = model.LeftFoot,
                RightThigh = model.RightThigh, RightShin = model.RightShin, RightFoot = model.RightFoot,
            };
        }

        return new AutoRagdoll2D(ragdoll, biped, model.PartOfBone, model.Roles, model.Humanoid, (float)model.Height);
    }
}
