using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Body
{
    /// <summary>Replaces this body by the Voronoi cells of its shapes now; see <see cref="PhysicsWorld3D.Fracture"/>. The body is destroyed and the fragments returned.</summary>
    /// <exception cref="FractureException">The body cannot be cut.</exception>
    public Body[] Fracture(FractureDef? def = null) { var (w, _) = Resolve(); return w.Fracture(this, def); }

    /// <summary>Makes the body fracture by itself when a contact impulse reaches the definition's threshold; see <see cref="PhysicsWorld3D.SetFracturable"/>.</summary>
    public void SetFracturable(FractureDef def) { var (w, _) = Resolve(); w.SetFracturable(this, def); }

    /// <summary>True when the body fractures by itself on impact.</summary>
    public bool IsFracturable { get { var (w, _) = Resolve(); return w.IsFracturable(this); } }

    /// <summary>Cuts the body with a plane through <paramref name="point"/> (world position) with <paramref name="normal"/> (world axes); see <see cref="PhysicsWorld3D.Slice"/>.</summary>
    /// <exception cref="FractureException">The plane misses the body or the body cannot be cut.</exception>
    public SliceResult Slice(Vector3D point, Vector3F normal) { var (w, _) = Resolve(); return w.Slice(this, point, normal); }
}
