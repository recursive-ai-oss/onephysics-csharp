using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Replaces this body by the Voronoi cells of its polygons now; see <see cref="PhysicsWorld2D.Fracture"/>. The body is destroyed and the fragments returned.</summary>
    /// <exception cref="FractureException">The body cannot be cut.</exception>
    public Body[] Fracture(FractureDef? def = null) { var (w, _) = Resolve(); return w.Fracture(this, def); }

    /// <summary>Makes the body fracture by itself when a contact impulse reaches the definition's threshold; see <see cref="PhysicsWorld2D.SetFracturable"/>.</summary>
    public void SetFracturable(FractureDef def) { var (w, _) = Resolve(); w.SetFracturable(this, def); }

    /// <summary>True when the body fractures by itself on impact.</summary>
    public bool IsFracturable { get { var (w, _) = Resolve(); return w.IsFracturable(this); } }

    /// <summary>Cuts the body with a line through <paramref name="point"/> (world position) with <paramref name="normal"/> (world axes); see <see cref="PhysicsWorld2D.Slice"/>.</summary>
    /// <exception cref="FractureException">The line misses the body or the body cannot be cut.</exception>
    public SliceResult Slice(Vector2D point, Vector2F normal) { var (w, _) = Resolve(); return w.Slice(this, point, normal); }
}
