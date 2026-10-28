using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>
    /// True for a bullet: a dynamic body that, besides the continuous collision every fast body gets against static shapes, is also swept against the dynamic and kinematic bodies that moved during the step,
    /// so it cannot tunnel through a thin body that moves across its path. Bullets do not sweep against each other. It costs a broadphase query per fast bullet per step, so keep it for small fast bodies.
    /// The flag is saved in snapshots. Default false (<see cref="BodyDef.IsBullet"/>).
    /// </summary>
    /// <exception cref="InvalidOperationException">Setting it on a static body.</exception>
    public bool IsBullet
    {
        get { var (w, b) = Resolve(); return w.Engine.IsBullet(b); }
        set
        {
            var (w, b) = Resolve();
            RequireMovable(w, b);
            ref byte f = ref w.Core.Bodies.Flags[b];
            if (value) { f |= (byte)BodyBits.Bullet; } else { f &= unchecked((byte)~(byte)BodyBits.Bullet); }
        }
    }
}
