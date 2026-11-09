using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // Strips and membranes: strain limiting as a hard projection after the contacts of every pass (Provot 1995; Goldenthal et al. 2007) and wind on the boundary (the 2D side of the plane's flat-plate model).

    private static void StripLimits(SoftBodyData d, int pass, bool last)
    {
        var st = d.Def.Strip!;
        float k = 1f + st.StrainLimit;
        var x = d.X; var inv = d.InvMass;
        int ne = d.SEdgeA.Length;
        // Two sweeps, in opposite directions on alternate passes, so no end of the strip is always last.
        int sweeps = last ? 24 : 4;
        for (int sweep = 0; sweep < sweeps; sweep++)
        {
            bool rev = ((pass + sweep) & 1) != 0;
            bool any = false;
            for (int i = 0; i < ne; i++)
            {
                int e = rev ? ne - 1 - i : i;
                int a = d.SEdgeA[e], b = d.SEdgeB[e];
                float wa = inv[a], wb = inv[b], wsum = wa + wb;
                if (wsum == 0f) { continue; }
                float max = d.SEdgeRest[e] * k;
                int ka = 2 * a, kb = 2 * b;
                float dx = x[ka] - x[kb], dy = x[ka + 1] - x[kb + 1];
                float l2 = dx * dx + dy * dy;
                if (!(l2 > max * max)) { continue; }
                float len = float.Sqrt(l2);
                float f = (len - max) / (len * wsum);
                float sa = wa * f, sb = wb * f;
                x[ka] -= dx * sa; x[ka + 1] -= dy * sa;
                x[kb] += dx * sb; x[kb + 1] += dy * sb;
                any = true;
            }

            if (!any && sweep >= 3) { break; }
        }
    }

    // Wind on the boundary of a strip for one substep. Each boundary edge (material on its left, outward normal on its right) presents its length times a unit depth to the relative flow; the windward face takes the plate
    // force from its angle of attack, the lee face only skin friction along the edge. The velocity change is capped at the relative speed, as in 3D.
    private void StripAero(SoftBodyData d, float h)
    {
        var st = d.Def.Strip!;
        var c = AeroCoefficients.Plate(2f);
        c.LiftSlope = st.LiftSlope; c.PlateDrag = st.Drag; c.MinDrag = Math.Min(c.MinDrag, st.Drag);
        float rho = Actuators.AirDensity;
        var x = d.X; var vel = d.Vel; var inv = d.InvMass;
        for (int s = 0; s < d.SurfA.Length; s++)
        {
            int a = d.SurfA[s], b = d.SurfB[s];
            int ka = 2 * a, kb = 2 * b;
            float tx = x[kb] - x[ka], ty = x[kb + 1] - x[ka + 1];
            float len = float.Sqrt(tx * tx + ty * ty);
            if (!(len > 1e-9f)) { continue; }
            float il = 1f / len;
            tx *= il; ty *= il;
            // the outward normal, on the right of the direction a to b
            float nx = ty, ny = -tx;
            double cxw = d.Ox + 0.5 * (x[ka] + x[kb]), cyw = d.Oy + 0.5 * (x[ka + 1] + x[kb + 1]);
            var wind = Actuators.WindAt(new Vector3D(cxw, cyw, 0.0));
            float vx = 0.5f * (vel[ka] + vel[kb]), vy = 0.5f * (vel[ka + 1] + vel[kb + 1]);
            float ux = wind.X - vx, uy = wind.Y - vy;
            // flow into the face: along the inward normal
            float un = -(ux * nx + uy * ny);
            float ut = ux * tx + uy * ty;
            float fc, fn;
            if (un > 0f) { c.Force(float.Abs(ut), un, len, rho, out fc, out fn); }
            else { c.Force(float.Abs(ut), 0f, len, rho, out fc, out fn); fn = 0f; }
            float sgn = ut >= 0f ? 1f : -1f;
            // force: along the tangent (sign of the flow) and along the inward normal (pushing the face away from the flow)
            float fx = fc * sgn * tx - fn * nx, fy = fc * sgn * ty - fn * ny;
            float wa = inv[a], wb = inv[b];
            float m = (wa > 0f ? 1f / wa : 0f) + (wb > 0f ? 1f / wb : 0f);
            float fl = float.Sqrt(fx * fx + fy * fy), ul = float.Sqrt(ux * ux + uy * uy);
            if (fl * h > ul * m && fl > 0f) { float sc = ul * m / (fl * h); fx *= sc; fy *= sc; }
            if (wa > 0f) { vel[ka] += fx * wa * h * 0.5f; vel[ka + 1] += fy * wa * h * 0.5f; }
            if (wb > 0f) { vel[kb] += fx * wb * h * 0.5f; vel[kb + 1] += fy * wb * h * 0.5f; }
        }
    }
}
