using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Number of seams and attachments in the world.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int ClothLinkCount => _engine.LinkHandles.Count;

    private SoftBodyData LinkParticle(SoftBody body, int particle, string bodyName, string particleName)
    {
        if (!body.IsValid) { throw new StaleHandleException(bodyName + " is stale or not a soft body of a world."); }
        if (body.WorldOrNull != this) { throw new ArgumentException(bodyName + " belongs to another world.", bodyName); }
        var d = _engine.SoftData[body.Slot]!;
        if ((uint)particle >= (uint)d.N) { throw new ArgumentOutOfRangeException(particleName, particle, "particle " + particle.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (d.N - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        return d;
    }

    /// <summary>Joins a particle of one soft body to a particle of another (or of the same body): a stitch. The two are held together by an equality constraint solved with the other contacts of every pass.</summary>
    /// <param name="a">The first body.</param>
    /// <param name="particleA">A particle of <paramref name="a"/>.</param>
    /// <param name="b">The second body (may be <paramref name="a"/>).</param>
    /// <param name="particleB">A particle of <paramref name="b"/>.</param>
    /// <param name="compliance">m/N; 0 (default) is as stiff as the solver can make it. Finite, 0 to <see cref="SoftBodyDef.MaxCompliance"/>.</param>
    /// <returns>The link's number, for <see cref="RemoveClothLink"/>. Numbers are reused, lowest first.</returns>
    /// <exception cref="StaleHandleException">A body was destroyed.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A particle or the compliance is out of range, or both ends are the same particle.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int AddSeam(SoftBody a, int particleA, SoftBody b, int particleB, float compliance = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddSeam");
        LinkParticle(a, particleA, nameof(a), nameof(particleA)); LinkParticle(b, particleB, nameof(b), nameof(particleB));
        if (!(compliance >= 0f && compliance <= SoftBodyDef.MaxCompliance)) { throw new ArgumentOutOfRangeException(nameof(compliance), compliance, "compliance must be finite and within 0 to " + SoftBodyDef.MaxCompliance.ToString("R", CultureInfo.InvariantCulture) + "."); }
        if (a.Slot == b.Slot && particleA == particleB) { throw new ArgumentOutOfRangeException(nameof(particleB), particleB, "a seam must join two different particles."); }
        return _engine.AddClothLink(new ClothLink { Kind = (int)ClothLinkKind.Seam, SoftA = a.Slot, PartA = particleA, SoftB = b.Slot, PartB = particleB, Compliance = compliance });
    }

    /// <summary>Stitches two rows of particles pairwise: <c>particlesA[i]</c> to <c>particlesB[i]</c>, the seam between two edges of cloth (see <see cref="AddSeam(SoftBody, int, SoftBody, int, float)"/>).</summary>
    /// <returns>The number of the first link made; the links are numbered consecutively unless numbers were being reused.</returns>
    /// <exception cref="ArgumentException">The two lists differ in length or are empty.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int AddSeam(SoftBody a, ReadOnlySpan<int> particlesA, SoftBody b, ReadOnlySpan<int> particlesB, float compliance = 0f)
    {
        if (particlesA.Length != particlesB.Length || particlesA.Length == 0) { throw new ArgumentException("the two particle lists must have the same, non-zero length (" + particlesA.Length.ToString(CultureInfo.InvariantCulture) + " and " + particlesB.Length.ToString(CultureInfo.InvariantCulture) + ").", nameof(particlesB)); }
        for (int i = 0; i < particlesA.Length; i++) { LinkParticle(a, particlesA[i], nameof(a), nameof(particlesA)); LinkParticle(b, particlesB[i], nameof(b), nameof(particlesB)); }
        int first = -1;
        for (int i = 0; i < particlesA.Length; i++)
        {
            int id = AddSeam(a, particlesA[i], b, particlesB[i], compliance);
            if (first < 0) { first = id; }
        }

        return first;
    }

    /// <summary>Holds a particle of a soft body to the point of a rigid body it is at now: the particle follows the body, and the body feels what pulls on the particle (momentum is conserved).</summary>
    /// <param name="cloth">The soft body.</param>
    /// <param name="particle">The particle to attach.</param>
    /// <param name="body">The rigid body.</param>
    /// <returns>The link's number, for <see cref="RemoveClothLink"/>. The link goes when the rigid body or the soft body is destroyed.</returns>
    /// <exception cref="StaleHandleException">A body was destroyed.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The particle is out of range.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int AttachCloth(SoftBody cloth, int particle, Body body)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AttachCloth");
        var d = LinkParticle(cloth, particle, nameof(cloth), nameof(particle));
        if (!body.IsValid) { throw new StaleHandleException("body is stale: the rigid body was destroyed."); }
        int b = body.Handle.Index;
        var p = _core.Bodies.PositionOf(b);
        var q = _engine.RotationOfBody(b);
        var r = q.Conjugated().Rotate(new Vector3F((float)(d.Pos[3 * particle] - p[0]), (float)(d.Pos[3 * particle + 1] - p[1]), (float)(d.Pos[3 * particle + 2] - p[2])));
        if (!float.IsFinite(r.X + r.Y + r.Z)) { throw new ArgumentOutOfRangeException(nameof(particle), particle, "the particle is too far from the body to attach."); }
        return _engine.AddClothLink(new ClothLink { Kind = (int)ClothLinkKind.Attachment, SoftA = cloth.Slot, PartA = particle, Rigid = b, RigidGen = body.Handle.Generation, Lx = r.X, Ly = r.Y, Lz = r.Z });
    }

    /// <summary>Attaches a row of particles (an edge of the cloth) to a rigid body, each where it is now (see <see cref="AttachCloth(SoftBody, int, Body)"/>).</summary>
    /// <returns>The number of the first link made.</returns>
    /// <exception cref="ArgumentException">The list is empty.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public int AttachClothEdge(SoftBody cloth, ReadOnlySpan<int> particles, Body body)
    {
        if (particles.Length == 0) { throw new ArgumentException("the particle list is empty.", nameof(particles)); }
        int first = -1;
        for (int i = 0; i < particles.Length; i++)
        {
            int id = AttachCloth(cloth, particles[i], body);
            if (first < 0) { first = id; }
        }

        return first;
    }

    /// <summary>Removes a seam or attachment.</summary>
    /// <param name="link">A number returned by one of the link methods.</param>
    /// <exception cref="ArgumentOutOfRangeException">The number is not that of a live link.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
    public void RemoveClothLink(int link)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.RemoveClothLink");
        if ((uint)link >= (uint)_engine.LinkHandles.HighWater || !_engine.LinkHandles.IsLive(link)) { throw new ArgumentOutOfRangeException(nameof(link), link, "link " + link.ToString(CultureInfo.InvariantCulture) + " is not a live cloth link."); }
        _engine.RemoveClothLink(link);
    }
}
