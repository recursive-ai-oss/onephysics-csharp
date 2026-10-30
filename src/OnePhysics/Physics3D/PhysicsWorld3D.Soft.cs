using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Adds a soft body from a mesh with the starting values of a material (happy path). A triangle mesh makes a cloth or a shell, a tetrahedral mesh a volume; a closed sphere surface from
    /// <see cref="SoftBodyMesh.Sphere"/> becomes a pressurised ball when the material is <see cref="SoftMaterial.Rubber"/>.
    /// </summary>
    /// <param name="mesh">The rest shape.</param>
    /// <param name="at">Where the mesh origin goes, world coordinates.</param>
    /// <param name="material">Starting values; default <see cref="SoftMaterial.Jelly"/>.</param>
    /// <returns>A handle to the body.</returns>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftBody(SoftBodyMesh mesh, Vector3D at = default, SoftMaterial material = SoftMaterial.Jelly)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        return _engine.AddSoftBody(MaterialDef(mesh, at, material));
    }

    private static SoftBodyDef MaterialDef(SoftBodyMesh mesh, Vector3D at, SoftMaterial material)
    {
        var def = material switch
        {
            SoftMaterial.Cloth => SoftBodyDef.Cloth(mesh, at),
            SoftMaterial.Rubber => SoftBodyDef.Rubber(mesh, at),
            SoftMaterial.Jelly => SoftBodyDef.Jelly(mesh, at),
            _ => throw new ArgumentOutOfRangeException(nameof(material), material, "material is not a SoftMaterial."),
        };
        if (mesh.IsClosed && mesh.TetrahedronCount == 0 && material == SoftMaterial.Rubber) { def.InternalPressure = 1f; }
        return def;
    }

    /// <summary>Adds a square of cloth in the x–z plane, centred on <paramref name="at"/>, optionally held at its four corners.</summary>
    /// <param name="width">Size along x, metres.</param>
    /// <param name="depth">Size along z, metres.</param>
    /// <param name="segments">Cells along each side, 1 to 1024.</param>
    /// <param name="at">Centre of the sheet.</param>
    /// <param name="pinCorners">Hold the four corner particles in place.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddCloth(float width, float depth, int segments, Vector3D at = default, bool pinCorners = false)
    {
        var mesh = SoftBodyMesh.Cloth(width, depth, segments, segments);
        var def = SoftBodyDef.Cloth(mesh, at);
        if (pinCorners)
        {
            int n = segments + 1;
            def.PinnedParticles = new[] { 0, segments, segments * n, segments * n + segments };
        }

        return _engine.AddSoftBody(def);
    }

    /// <summary>Adds a block of soft material made of tetrahedra, centred on <paramref name="at"/>.</summary>
    /// <param name="size">Edge lengths, metres.</param>
    /// <param name="at">Centre of the block.</param>
    /// <param name="material">Starting values, <see cref="SoftMaterial.Jelly"/> by default.</param>
    /// <param name="cells">Cells along each edge, 1 to 64. Default 4.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftBox(Vector3F size, Vector3D at = default, SoftMaterial material = SoftMaterial.Jelly, int cells = 4) =>
        _engine.AddSoftBody(MaterialDef(SoftBodyMesh.Box(size, cells, cells, cells), at, material));

    /// <summary>Adds a pressurised ball.</summary>
    /// <param name="radius">Metres.</param>
    /// <param name="at">Centre of the ball.</param>
    /// <param name="material">Starting values, <see cref="SoftMaterial.Rubber"/> by default.</param>
    /// <param name="segments">Cells along each edge of the cube the sphere is made from, 1 to 128. Default 4.</param>
    /// <exception cref="SoftBodyException">A value is out of range.</exception>
    public SoftBody AddSoftBall(float radius, Vector3D at = default, SoftMaterial material = SoftMaterial.Rubber, int segments = 4)
    {
        var def = MaterialDef(SoftBodyMesh.Sphere(radius, segments), at, material);
        def.InternalPressure = 1f;
        return _engine.AddSoftBody(def);
    }

    /// <summary>Adds a soft body from a definition (middle tier). The definition is copied.</summary>
    /// <exception cref="SoftBodyException">A field is out of range or the mesh and definition do not fit (see <see cref="SoftBodyDef"/>).</exception>
    public SoftBody AddSoftBody(SoftBodyDef def)
    {
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
        if ((uint)slot >= (uint)e.SoftHandles.HighWater) { throw new ArgumentOutOfRangeException(nameof(slot), slot, "soft body slot " + slot.ToString(System.Globalization.CultureInfo.InvariantCulture) + " is outside 0 to " + (e.SoftHandles.HighWater - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + "."); }
        if (!e.SoftHandles.IsLive(slot)) { throw new StaleHandleException("Soft body slot " + slot.ToString(System.Globalization.CultureInfo.InvariantCulture) + " holds no soft body."); }
        return new SoftBody(this, slot, e.SoftHandles.GenerationOf(slot));
    }

    /// <summary>Number of soft bodies in the world.</summary>
    public int SoftBodyCount => _engine.SoftHandles.Count;
}
