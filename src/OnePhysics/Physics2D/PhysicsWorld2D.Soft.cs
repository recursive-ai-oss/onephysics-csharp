using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Adds a soft body from a mesh with the starting values of a material (happy path). A triangle mesh makes a filled body (area constraints on every triangle); an outline from <see cref="SoftBodyMesh.Ring"/> becomes a pressurised balloon
    /// when the material is <see cref="SoftMaterial.Rubber"/>.
    /// </summary>
    /// <param name="mesh">The rest shape.</param>
    /// <param name="at">Where the mesh origin goes, world coordinates.</param>
    /// <param name="material">Starting values; default <see cref="SoftMaterial.Jelly"/>.</param>
    /// <returns>A handle to the body.</returns>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftBody(SoftBodyMesh mesh, Vector2D at = default, SoftMaterial material = SoftMaterial.Jelly)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSoftBody");
        ArgumentNullException.ThrowIfNull(mesh);
        return _engine.AddSoftBody(MaterialDef(mesh, at, material));
    }

    private static SoftBodyDef MaterialDef(SoftBodyMesh mesh, Vector2D at, SoftMaterial material)
    {
        var def = material switch
        {
            SoftMaterial.Rubber => SoftBodyDef.Rubber(mesh, at),
            SoftMaterial.Jelly => SoftBodyDef.Jelly(mesh, at),
            _ => throw new ArgumentOutOfRangeException(nameof(material), material, "material is not a SoftMaterial."),
        };
        if (mesh.IsClosed && mesh.TriangleCount == 0 && material == SoftMaterial.Rubber) { def.InternalPressure = 1f; }
        return def;
    }

    /// <summary>Adds a block of soft material made of triangles, centred on <paramref name="at"/>.</summary>
    /// <param name="size">Width and height, metres.</param>
    /// <param name="at">Centre of the block.</param>
    /// <param name="material">Starting values, <see cref="SoftMaterial.Jelly"/> by default.</param>
    /// <param name="cells">Cells along each side, 1 to 64. Default 4.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftBox(Vector2F size, Vector2D at = default, SoftMaterial material = SoftMaterial.Jelly, int cells = 4)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSoftBox");
        return _engine.AddSoftBody(MaterialDef(SoftBodyMesh.Box(size, cells, cells), at, material));
    }

    /// <summary>Adds a soft disc made of triangles, centred on <paramref name="at"/>.</summary>
    /// <param name="radius">Metres.</param>
    /// <param name="at">Centre of the disc.</param>
    /// <param name="material">Starting values, <see cref="SoftMaterial.Jelly"/> by default.</param>
    /// <param name="rings">Rings of vertices around the centre, 1 to 32. Default 3.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftCircle(float radius, Vector2D at = default, SoftMaterial material = SoftMaterial.Jelly, int rings = 3)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSoftCircle");
        return _engine.AddSoftBody(MaterialDef(SoftBodyMesh.Disc(radius, rings, Math.Max(6, 6 * rings)), at, material));
    }

    /// <summary>Adds a balloon: a closed skin that holds its enclosed area.</summary>
    /// <param name="radius">Metres.</param>
    /// <param name="at">Centre of the balloon.</param>
    /// <param name="segments">Particles around the skin, 3 to 4096. Default 24.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddBalloon(float radius, Vector2D at = default, int segments = 24)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddBalloon");
        return _engine.AddSoftBody(SoftBodyDef.Balloon(SoftBodyMesh.Ring(radius, segments), at));
    }

    /// <summary>Adds a shape-matching blob (a disc whose clusters pull it back to its rest shape).</summary>
    /// <param name="radius">Metres.</param>
    /// <param name="at">Centre of the blob.</param>
    /// <param name="rings">Rings of vertices around the centre, 1 to 32. Default 3.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddBlob(float radius, Vector2D at = default, int rings = 3)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddBlob");
        return _engine.AddSoftBody(SoftBodyDef.Blob(SoftBodyMesh.Disc(radius, rings, Math.Max(6, 6 * rings)), at));
    }

    /// <summary>Adds a soft body from a definition (middle tier). The definition is copied.</summary>
    /// <exception cref="SoftBodyException">A field is out of range or the mesh and definition do not fit (see <see cref="SoftBodyDef"/>).</exception>
    public SoftBody AddSoftBody(SoftBodyDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddSoftBody");
        ArgumentNullException.ThrowIfNull(def);
        return _engine.AddSoftBody(def);
    }

    /// <summary>The handle of the soft body in a slot (<see cref="SoftBody.Slot"/>). Use it to get at the bodies of a world after <c>Restore</c>, which replaces them: a handle from before the restore belongs to the old state.</summary>
    /// <param name="slot">The slot number, 0 or more.</param>
    /// <exception cref="ArgumentOutOfRangeException">The slot is negative or beyond every slot the world has used.</exception>
    /// <exception cref="StaleHandleException">The slot holds no soft body.</exception>
    public SoftBody GetSoftBody(int slot)
    {
        var e = _engine;
        if ((uint)slot >= (uint)e.SoftHandles.HighWater) { throw new ArgumentOutOfRangeException(nameof(slot), slot, "soft body slot " + slot.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (e.SoftHandles.HighWater - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        if (!e.SoftHandles.IsLive(slot)) { throw new StaleHandleException("Soft body slot " + slot.ToString(CultureInfo.InvariantCulture) + " holds no soft body."); }
        return new SoftBody(this, slot, e.SoftHandles.GenerationOf(slot));
    }

    /// <summary>Number of soft bodies in the world.</summary>
    public int SoftBodyCount => _engine.SoftHandles.Count;

    /// <summary>
    /// A stateless sensor against soft bodies: writes, in ascending slot order, the soft bodies with a particle disc that overlaps the shape placed at <paramref name="position"/> and <paramref name="angleRadians"/>
    /// (touching counts), and returns how many were written. When <paramref name="results"/> is too small the lowest slots are kept. Allocates nothing; sleeping bodies are included.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The position or angle is not finite or the position is beyond ±2^22.</exception>
    /// <exception cref="ArgumentException">The shape is the default (empty) value.</exception>
    public int OverlapSoftBodies(in Shape2D shape, Vector2D position, float angleRadians, Span<SoftBody> results)
    {
        if ((byte)shape.Kind == 0) { throw new ArgumentException("The shape is the default (empty) value.", nameof(shape)); }
        const double Bound = 4194304.0;
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || Math.Abs(position.X) > Bound || Math.Abs(position.Y) > Bound) { throw new ArgumentOutOfRangeException(nameof(position), position, "position must be finite and within ±2^22."); }
        if (!float.IsFinite(angleRadians)) { throw new ArgumentOutOfRangeException(nameof(angleRadians), angleRadians, "angleRadians must be finite."); }
        var (sn, cs) = Math64.SinCos(angleRadians);
        var q = new Rot2((float)cs, (float)sn);
        return _engine.OverlapSoft(in shape, position.X, position.Y, q, results, this);
    }
}
