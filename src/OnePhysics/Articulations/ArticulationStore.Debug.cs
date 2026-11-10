using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Articulations;

internal sealed partial class ArticulationStore
{
    /// <summary>Appends the joints of every link and the contacts of the last step, articulations by slot.</summary>
    internal void AddDebug(DebugGeometry g)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var s = States[i]!;
            var m = s.Model;
            m.ForwardKinematics();
            for (int k = 0; k < m.N; k++)
            {
                var kind = m.Kind[k];
                var axis = kind == JointKind.Revolute || kind == JointKind.Prismatic ? m.Rw[k] * m.Axis[k] : D3.Zero;
                float pos = 0f, vel = 0f;
                bool atLimit = false;
                if (kind == JointKind.Revolute || kind == JointKind.Prismatic)
                {
                    double q = m.Q[m.PosOff[k]];
                    pos = (float)q; vel = (float)m.Qd[m.VelOff[k]];
                    atLimit = m.HasLimit[k] && (q - m.Lower[k] < 1e-3 || m.Upper[k] - q < 1e-3);
                }

                var com = m.WorldPoint(k, m.Com[k]);
                var shown = kind == JointKind.Free || kind == JointKind.FreePlanar ? ArticulationJointKind.Fixed : (ArticulationJointKind)(byte)kind;
                g.Add(new DebugArticulationJoint(_host.Core.HandleOfBody(s.Body[k]), i, k, m.Parent[k], shown, new Vector3D(m.Pw[k].X, m.Pw[k].Y, m.Pw[k].Z), new Vector3F((float)axis.X, (float)axis.Y, (float)axis.Z), new Vector3D(com.X, com.Y, com.Z), pos, vel, atLimit));
            }

            for (int c = 0; c < s.LastContactCount; c++)
            {
                var d = s.LastContacts[c];
                g.Add(new DebugArticulationContact(i, d.Link, d.Point, d.Normal, d.Separation, d.NormalImpulse));
            }
        }
    }
}
