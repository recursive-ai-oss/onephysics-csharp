using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>What a bone is for, as far as a ragdoll is concerned. <see cref="Auto"/> asks the name heuristics.</summary>
public enum BoneRole : byte
{
    /// <summary>Decided from the bone's name (see <see cref="AutoRagdoller"/>).</summary>
    Auto = 0,

    /// <summary>The hips; the ragdoll's root part.</summary>
    Pelvis,

    /// <summary>A lower spine bone (abdomen, waist).</summary>
    Spine,

    /// <summary>An upper spine bone (chest, thorax).</summary>
    Chest,

    /// <summary>A neck bone; it joins the head's part.</summary>
    Neck,

    /// <summary>The head.</summary>
    Head,

    /// <summary>The upper arm.</summary>
    UpperArm,

    /// <summary>The forearm.</summary>
    Forearm,

    /// <summary>The hand.</summary>
    Hand,

    /// <summary>The thigh.</summary>
    Thigh,

    /// <summary>The shin (calf).</summary>
    Shin,

    /// <summary>The foot.</summary>
    Foot,

    /// <summary>A toe; it joins the foot's part.</summary>
    Toe,

    /// <summary>Any bone of a skeleton that is not a person (generic skeletons make a part per bone).</summary>
    Generic,

    /// <summary>Folded into the nearest ancestor that has a part: twist bones, fingers, clavicles, end markers.</summary>
    Merge,
}

/// <summary>How masses are shared among the parts.</summary>
public enum AutoRagdollMassModel : byte
{
    /// <summary>Winter's anthropometric segment table (Dempster's data) for a recognised humanoid; volume times density for any other skeleton. The default.</summary>
    Auto = 0,

    /// <summary>Always volume times <see cref="AutoRagdollOptions.Density"/>, scaled to the total mass.</summary>
    Volume = 1,
}

/// <summary>Options for <see cref="AutoRagdoller"/>.</summary>
public sealed class AutoRagdollOptions
{
    /// <summary>Total mass of the figure, kg; finite, above 0, at most 1e7. Default 75.</summary>
    public float TotalMass { get; init; } = 75f;

    /// <summary>How masses are shared. Default <see cref="AutoRagdollMassModel.Auto"/>.</summary>
    public AutoRagdollMassModel MassModel { get; init; }

    /// <summary>Density, kg/m³, for the volume model; finite, above 0. Default 985 (the human body's mean).</summary>
    public float Density { get; init; } = 985f;

    /// <summary>Humanoids: give each hand its own part (15 parts) instead of folding it into the forearm (13). Default false.</summary>
    public bool IncludeHands { get; init; }

    /// <summary>Humanoids: split the torso into an abdomen and a chest part (one more part) when the skeleton has at least two spine bones. Default false.</summary>
    public bool SplitTorso { get; init; }

    /// <summary>Friction of every part's shape; feet of a humanoid get at least 1. At least 0. Default 0.6.</summary>
    public float Friction { get; init; } = 0.6f;

    /// <summary>Which parts of the figure collide with each other. Default parts that are not joined.</summary>
    public RecursiveAI.OnePhysics.Physics3D.RagdollSelfCollision SelfCollision { get; init; } = RecursiveAI.OnePhysics.Physics3D.RagdollSelfCollision.NonAdjacent;

    /// <summary>Roles by bone name (exact, case-sensitive), overriding the name heuristics; null for none. Bones whose role is <see cref="BoneRole.Auto"/> here are still decided by name.</summary>
    public IReadOnlyDictionary<string, BoneRole>? RoleOverrides { get; init; }

    /// <summary>Generic skeletons: bones shorter than this, m, are folded into their parent. 0 chooses 2% of the skeleton's height.</summary>
    public float MinBoneLength { get; init; }

    /// <summary>3D: raises every swing half-angle to at least this many radians, 0 for the published ranges as they are (default). In tumbling drops of limp figures the engine's swing limits are unstable where a cone is narrow (a knee's 0.08 rad sideways swing, an elbow's 0.1): the
    /// engine's own humanoid shows it too, and half-angles of at least 0.6 rad removed it in testing. Use it for figures that will be thrown about rather than driven. From 0 to π.</summary>
    public float MinSwingHalfAngle { get; init; }

    /// <summary>2D only: the direction the figure faces, +1 for +x and -1 for -x. Joint limits that bend forward follow it. Default 1.</summary>
    public int Facing { get; init; } = 1;
}

/// <summary>Skinning data for fitting shapes: mesh points and, for each, the bones that move it. Each point has <see cref="InfluencesPerVertex"/> (bone index, weight) pairs; unused slots have weight 0.</summary>
public sealed class SkinnedPoints3D
{
    /// <summary>Creates the skin data (the arrays are used as given, not copied).</summary>
    public SkinnedPoints3D(Vector3F[] vertices, int[] boneIndices, float[] boneWeights, int influencesPerVertex = 4)
    {
        Vertices = vertices; BoneIndices = boneIndices; BoneWeights = boneWeights; InfluencesPerVertex = influencesPerVertex;
    }

    /// <summary>Mesh points in the skeleton's bind pose.</summary>
    public Vector3F[] Vertices { get; }

    /// <summary>Bone index per influence, <see cref="InfluencesPerVertex"/> per point.</summary>
    public int[] BoneIndices { get; }

    /// <summary>Weight per influence, the same layout.</summary>
    public float[] BoneWeights { get; }

    /// <summary>Influences stored per point, 1 to 16.</summary>
    public int InfluencesPerVertex { get; }
}

/// <summary>The 2D form of <see cref="SkinnedPoints3D"/>.</summary>
public sealed class SkinnedPoints2D
{
    /// <summary>Creates the skin data (the arrays are used as given, not copied).</summary>
    public SkinnedPoints2D(Vector2F[] vertices, int[] boneIndices, float[] boneWeights, int influencesPerVertex = 4)
    {
        Vertices = vertices; BoneIndices = boneIndices; BoneWeights = boneWeights; InfluencesPerVertex = influencesPerVertex;
    }

    /// <summary>Mesh points in the skeleton's bind pose.</summary>
    public Vector2F[] Vertices { get; }

    /// <summary>Bone index per influence.</summary>
    public int[] BoneIndices { get; }

    /// <summary>Weight per influence.</summary>
    public float[] BoneWeights { get; }

    /// <summary>Influences stored per point, 1 to 16.</summary>
    public int InfluencesPerVertex { get; }
}

/// <summary>A 3D ragdoll made from a skeleton.</summary>
public sealed class AutoRagdoll3D
{
    internal AutoRagdoll3D(RecursiveAI.OnePhysics.Physics3D.RagdollDef definition, BipedBones? biped, int[] partOfBone, BoneRole[] roles, bool humanoid, float height)
    {
        Definition = definition; Biped = biped; PartOfBone = partOfBone; Roles = roles; IsHumanoid = humanoid; Height = height;
    }

    /// <summary>The definition, ready for <c>PhysicsWorld3D.AddRagdoll</c>. Positions are in the skeleton's own space (the bind pose), parents before children, the pelvis (or the skeleton's first part) is part 0.</summary>
    public RecursiveAI.OnePhysics.Physics3D.RagdollDef Definition { get; }

    /// <summary>The part indices for the balance and walking controllers when the skeleton is a recognised biped (pelvis, torso, both legs; the head if present); null otherwise.</summary>
    public BipedBones? Biped { get; }

    /// <summary>For each input bone, the index of the part that carries it (bones folded into a neighbour map to that neighbour's part), or -1 for a bone above the root part.</summary>
    public int[] PartOfBone { get; }

    /// <summary>The role each bone was given.</summary>
    public BoneRole[] Roles { get; }

    /// <summary>True when the skeleton was recognised as a person (pelvis, two legs, a torso).</summary>
    public bool IsHumanoid { get; }

    /// <summary>The figure's height, m: from the mesh when there is one, else estimated from the bones.</summary>
    public float Height { get; }
}

/// <summary>A 2D ragdoll made from a skeleton.</summary>
public sealed class AutoRagdoll2D
{
    internal AutoRagdoll2D(RecursiveAI.OnePhysics.Physics2D.RagdollDef definition, BipedBones? biped, int[] partOfBone, BoneRole[] roles, bool humanoid, float height)
    {
        Definition = definition; Biped = biped; PartOfBone = partOfBone; Roles = roles; IsHumanoid = humanoid; Height = height;
    }

    /// <summary>The definition, ready for <c>PhysicsWorld2D.AddRagdoll</c>.</summary>
    public RecursiveAI.OnePhysics.Physics2D.RagdollDef Definition { get; }

    /// <summary>The part indices for the balance and walking controllers when the skeleton is a recognised biped; null otherwise.</summary>
    public BipedBones? Biped { get; }

    /// <summary>For each input bone, the index of the part that carries it, or -1.</summary>
    public int[] PartOfBone { get; }

    /// <summary>The role each bone was given.</summary>
    public BoneRole[] Roles { get; }

    /// <summary>True when the skeleton was recognised as a person.</summary>
    public bool IsHumanoid { get; }

    /// <summary>The figure's height, m.</summary>
    public float Height { get; }
}
