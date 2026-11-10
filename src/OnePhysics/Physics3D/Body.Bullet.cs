using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Body
{
    /// <summary>
    /// True for a body marked as a bullet (<see cref="BodyDef.IsBullet"/>). Continuous collision runs for every fast dynamic body whether or not it is marked; the flag is saved in snapshots and shown by inspection.
    /// Default false.
    /// </summary>
    /// <exception cref="InvalidOperationException">Setting it on a static body.</exception>
    public bool IsBullet
    {
        get { var (w, b) = Resolve(); return (w.Core.Bodies.Flags[b] & (byte)BodyBits.Bullet) != 0; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Body.IsBullet");
            var (w, b) = Resolve();
            RequireMovable(w, b);
            ref byte f = ref w.Core.Bodies.Flags[b];
            if (value) { f |= (byte)BodyBits.Bullet; } else { f &= unchecked((byte)~(byte)BodyBits.Bullet); }
        }
    }
}
