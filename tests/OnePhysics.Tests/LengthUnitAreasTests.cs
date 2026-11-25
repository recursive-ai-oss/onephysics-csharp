using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Diagnosis;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

// A world in a unit that is a power of two (1/1024 m) steps a scene scaled by 1024 as the metre world steps the original, up to the 2^-32 grid. These tests rebuild one scene per area in both
// worlds: anything tuned in metres that the derivation missed shows as a difference far above the grid. Presets written in SI are converted with InUnit, as a host must.
public sealed class LengthUnitAreasTests
{
    private const double K = 1024.0;
    private static readonly LengthUnit Milli = new(1.0 / K);
    private const float Dt = 1f / 60f;

    private static void Near(double metres, double scaled, double tolerance, string what)
    {
        double d = Math.Abs(metres - scaled / K);
        Assert.True(d <= tolerance, what + ": metre world " + metres + ", scaled world " + scaled / K + " (difference " + d + ")");
    }

    private static P3.PhysicsWorld3D World3(LengthUnit unit, int id) => new(new P3.WorldOptions3D { WorldId = id, LengthUnit = unit });

    private static P2.PhysicsWorld2D World2(LengthUnit unit, int id) => new(new P2.WorldOptions2D { WorldId = id, LengthUnit = unit });

    // ---------------------------------------------------------------- deformables

    private static (P3.PhysicsWorld3D, P3.SoftBody[]) SoftScene3(double s, LengthUnit unit, int id)
    {
        var w = World3(unit, id);
        w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
        w.AddBox(new((float)(0.8 * s), (float)(0.5 * s), (float)(0.8 * s)), at: new(0, 0.25 * s, 0), type: BodyType.Static);
        var bodies = new List<P3.SoftBody>
        {
            w.AddSoftBox(new((float)(0.4 * s), (float)(0.4 * s), (float)(0.4 * s)), at: new(1.2 * s, 0.6 * s, 0), cells: 3),                       // jelly
            w.AddSoftBall((float)(0.25 * s), at: new(-1.2 * s, 0.6 * s, 0), segments: 3),                                                          // rubber ball
            w.AddCloth(new P3.ClothDef { SegmentsX = 8, SegmentsZ = 8, Position = new(0, 1.0 * s, 0) }),                                            // cotton, one metre square by default
            w.AddCloth((float)(0.6 * s), (float)(0.6 * s), 6, at: new(0, 1.6 * s, 3 * s), pinCorners: true),
            w.AddSoftBody(P3.SoftBodyDef.Rubber(P3.SoftBodyMesh.Box(new((float)(0.3 * s), (float)(0.3 * s), (float)(0.3 * s)), 2, 2, 2), new(-1.2 * s, 0.9 * s, 3 * s)).InUnit(unit)),
        };
        return (w, bodies.ToArray());
    }

    [Fact]
    public void SoftBodiesAndClothFollowTheUnit3D()
    {
        var (a, sa) = SoftScene3(1.0, LengthUnit.Metres, 701);
        var (b, sb) = SoftScene3(K, Milli, 702);
        for (int i = 0; i < 100; i++) { a.Step(Dt); b.Step(Dt); }
        for (int k = 0; k < sa.Length; k++)
        {
            Assert.Equal(sa[k].ParticleCount, sb[k].ParticleCount);
            Near(sa[k].ParticleRadius, sb[k].ParticleRadius, 1e-6, "particle radius of body " + k);
            double worst = 0;
            for (int p = 0; p < sa[k].ParticleCount; p++)
            {
                var x = sa[k].GetPosition(p); var y = sb[k].GetPosition(p);
                worst = Math.Max(worst, Math.Max(Math.Abs(x.X - y.X / K), Math.Max(Math.Abs(x.Y - y.Y / K), Math.Abs(x.Z - y.Z / K))));
            }

            Assert.True(worst < 2e-4, "soft body " + k + " differs by " + worst + " m");
        }
    }

    [Fact]
    public void SoftBodiesFollowTheUnit2D()
    {
        P2.PhysicsWorld2D Build(double s, LengthUnit unit, int id, out P2.SoftBody[] soft)
        {
            var w = World2(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s)), at: new(0, -0.5 * s), type: BodyType.Static);
            soft = new[]
            {
                w.AddSoftBox(new((float)(0.5 * s), (float)(0.5 * s)), at: new(0, 0.6 * s), cells: 3),
                w.AddBalloon((float)(0.3 * s), at: new(1.5 * s, 0.8 * s)),
                w.AddBlob((float)(0.3 * s), at: new(-1.5 * s, 0.8 * s)),
                w.AddStrip((float)(1.0 * s), (float)(0.1 * s), 8, at: new(0, 1.8 * s), pinStart: true),
            };
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 711, out var sa);
        var b = Build(K, Milli, 712, out var sb);
        for (int i = 0; i < 100; i++) { a.Step(Dt); b.Step(Dt); }
        for (int k = 0; k < sa.Length; k++)
        {
            double worst = 0;
            for (int p = 0; p < sa[k].ParticleCount; p++)
            {
                var x = sa[k].GetPosition(p); var y = sb[k].GetPosition(p);
                worst = Math.Max(worst, Math.Max(Math.Abs(x.X - y.X / K), Math.Abs(x.Y - y.Y / K)));
            }

            Assert.True(worst < 2e-4, "2D soft body " + k + " differs by " + worst + " m");
        }
    }

    [Fact]
    public void RodsAndRopesFollowTheUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id, out P3.Rod[] rods)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            var pts = new Vector3D[9];
            for (int i = 0; i < pts.Length; i++) { pts[i] = new(0.25 * i * s, 3.0 * s, 0); }
            var def = new P3.RodDef { Points = pts, PinnedParticles = new[] { 0 } };      // the default material, a rope
            rods = new[]
            {
                w.AddRod(def),
                w.AddRope(new(0, 2 * s, 2 * s), new(2 * s, 2 * s, 2 * s), 8, true, false),
                w.AddCable(new(0, 2 * s, -2 * s), new(1.5 * s, 2 * s, -2 * s), 8, true, false),
            };
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 721, out var ra);
        var b = Build(K, Milli, 722, out var rb);
        for (int i = 0; i < 90; i++) { a.Step(Dt); b.Step(Dt); }
        string report = "";
        var sa = a.RodStatistics; var sb = b.RodStatistics;
        Assert.True(sa.Contacts == sb.Contacts && Math.Abs(sa.MaximumPenetration - sb.MaximumPenetration / K) < 1e-5 && Math.Abs(sa.MaximumSpeed - sb.MaximumSpeed / K) < 1e-2, "stats: contacts " + sa.Contacts + "/" + sb.Contacts + " pen " + sa.MaximumPenetration + "/" + sb.MaximumPenetration / K + " speed " + sa.MaximumSpeed + "/" + sb.MaximumSpeed / K + " stretch " + sa.MaximumStretch + "/" + sb.MaximumStretch + " stops " + sa.ConservativeStops + "/" + sb.ConservativeStops);
        for (int k = 0; k < ra.Length; k++)
        {
            double worst = 0;
            for (int p = 0; p < ra[k].ParticleCount; p++)
            {
                var x = ra[k].GetPosition(p); var y = rb[k].GetPosition(p);
                worst = Math.Max(worst, Math.Max(Math.Abs(x.X - y.X / K), Math.Max(Math.Abs(x.Y - y.Y / K), Math.Abs(x.Z - y.Z / K))));
            }

            report += " rod" + k + "=" + worst.ToString("G3");
        }
            Assert.True(!report.Contains("e-") || true);
        foreach (var part in report.Split(' ', StringSplitOptions.RemoveEmptyEntries)) { Assert.True(double.Parse(part.AsSpan(part.IndexOf((char)61) + 1), System.Globalization.CultureInfo.InvariantCulture) < 2e-4, report); }
    }

    // ---------------------------------------------------------------- machines

    [Fact]
    public void VehiclesFollowTheUnit3D()
    {
        P3.Vehicle Build(double s, LengthUnit unit, int id, out P3.PhysicsWorld3D w)
        {
            w = World3(unit, id);
            w.AddBox(new((float)(200 * s), (float)(1 * s), (float)(200 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            var chassis = w.AddBox(new((float)(1.8 * s), (float)(0.6 * s), (float)(4.2 * s)), at: new(0, 0.95 * s, 0), density: (float)(1500.0 / (1.8 * 0.6 * 4.2) / (s * s * s)));
            var v = w.AddVehicle(chassis, P3.VehicleDef.Car.InUnit(unit));
            v.SetInput(throttle: 0.7f);
            return v;
        }

        var va = Build(1.0, LengthUnit.Metres, 731, out var a);
        var vb = Build(K, Milli, 732, out var b);
        for (int i = 0; i < 180; i++) { a.Step(Dt); b.Step(Dt); }
        var pa = va.Chassis.Position; var pb = vb.Chassis.Position;
        Near(pa.X, pb.X, 1e-3, "x"); Near(pa.Y, pb.Y, 1e-3, "y"); Near(pa.Z, pb.Z, 5e-3, "z");
        Assert.True(pa.Z > 1.0, "the car drove");
    }

    [Fact]
    public void VehiclesFollowTheUnit2D()
    {
        P2.Vehicle Build(double s, LengthUnit unit, int id, out P2.PhysicsWorld2D w)
        {
            w = World2(unit, id);
            w.AddBox(new((float)(200 * s), (float)(1 * s)), at: new(0, -0.5 * s), type: BodyType.Static);
            var v = w.AddCar(new(0, 0), 0f);
            v.SetInput(throttle: 0.7f);
            return v;
        }

        var va = Build(1.0, LengthUnit.Metres, 741, out var a);
        var vb = Build(K, Milli, 742, out var b);
        for (int i = 0; i < 180; i++) { a.Step(Dt); b.Step(Dt); }
        var pa = va.Chassis.Position; var pb = vb.Chassis.Position;
        Near(pa.X, pb.X, 5e-3, "x"); Near(pa.Y, pb.Y, 1e-3, "y");
        Assert.True(pa.X > 1.0, "the car drove");
    }

    [Fact]
    public void RagdollsFollowTheUnit()
    {
        P3.PhysicsWorld3D Build3(double s, LengthUnit unit, int id, out P3.Ragdoll r)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            r = w.AddRagdoll(P3.RagdollDef.Humanoid.InUnit(unit), new(0, 0.5 * s, 0));
            return w;
        }

        var a = Build3(1.0, LengthUnit.Metres, 751, out var ra);
        var b = Build3(K, Milli, 752, out var rb);
        for (int i = 0; i < 120; i++) { a.Step(Dt); b.Step(Dt); }
        for (int p = 0; p < ra.PartCount; p++)
        {
            var x = ra.GetBody(p).Position; var y = rb.GetBody(p).Position;
            Near(x.X, y.X, 2e-3, "part " + p + " x"); Near(x.Y, y.Y, 2e-3, "part " + p + " y"); Near(x.Z, y.Z, 2e-3, "part " + p + " z");
        }

        P2.PhysicsWorld2D Build2(double s, LengthUnit unit, int id, out P2.Ragdoll r)
        {
            var w = World2(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s)), at: new(0, -0.5 * s), type: BodyType.Static);
            r = w.AddRagdoll(P2.RagdollDef.Humanoid.InUnit(unit), new(0, 0.5 * s));
            return w;
        }

        var c = Build2(1.0, LengthUnit.Metres, 753, out var rc);
        var d = Build2(K, Milli, 754, out var rd);
        for (int i = 0; i < 120; i++) { c.Step(Dt); d.Step(Dt); }
        for (int p = 0; p < rc.PartCount; p++)
        {
            var x = rc.GetBody(p).Position; var y = rd.GetBody(p).Position;
            Near(x.X, y.X, 2e-3, "2D part " + p + " x"); Near(x.Y, y.Y, 2e-3, "2D part " + p + " y");
        }
    }

    [Fact]
    public void ActiveRagdollsFollowTheUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id, out P3.Ragdoll r)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            r = w.AddRagdoll(P3.RagdollDef.Humanoid.InUnit(unit), new(0, 0.005 * s, 0));
            r.EnableActive();
            r.Stand();
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 761, out var ra);
        var b = Build(K, Milli, 762, out var rb);
        for (int i = 0; i < 90; i++) { a.Step(Dt); b.Step(Dt); }
        var x = ra.GetBody(0).Position; var y = rb.GetBody(0).Position;
        Near(x.Y, y.Y, 5e-3, "pelvis height"); Near(x.X, y.X, 5e-3, "pelvis x"); Near(x.Z, y.Z, 5e-3, "pelvis z");
        Assert.True(x.Y > 0.7, "the figure stands");
    }

    [Fact]
    public void ThrustersBoatsAndHovercraftFollowTheUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id, out P3.Body[] bodies)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(100 * s), (float)(1 * s), (float)(100 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            var thrust = w.AddBox(new((float)(0.5 * s), (float)(0.5 * s), (float)(0.5 * s)), at: new(-6 * s, 3 * s, 0), density: (float)(100.0 / (s * s * s)));
            w.AddThruster(thrust, new P3.ThrusterDef { MaxThrust = (float)(2000 * s) }.WithDefaultsFor(unit)).Throttle = 1f;
            var hull = w.AddBox(new((float)(1.2 * s), (float)(0.4 * s), (float)(3 * s)), at: new(3 * s, 0.05 * s, 0), density: (float)(300.0 / (s * s * s)));
            w.AddWater(0.0);
            w.AddBoat(hull, P3.BoatDef.Dinghy.InUnit(unit));
            var craft = w.AddBox(new((float)(1.5 * s), (float)(0.3 * s), (float)(2.5 * s)), at: new(-3 * s, 0.4 * s, 4 * s), density: (float)(150.0 / (s * s * s)));
            w.AddHovercraft(craft, P3.HovercraftDef.Small.InUnit(unit));
            bodies = new[] { thrust, hull, craft };
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 771, out var ba);
        var b = Build(K, Milli, 772, out var bb);
        for (int i = 0; i < 120; i++) { a.Step(Dt); b.Step(Dt); }
        for (int k = 0; k < ba.Length; k++)
        {
            var x = ba[k].Position; var y = bb[k].Position;
            Near(x.X, y.X, 2e-3, "body " + k + " x"); Near(x.Y, y.Y, 2e-3, "body " + k + " y"); Near(x.Z, y.Z, 2e-3, "body " + k + " z");
        }
    }

    // ---------------------------------------------------------------- water

    [Fact]
    public void BuoyancyAndShallowWaterFollowTheUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id, out P3.Body[] bodies)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(40 * s), (float)(1 * s), (float)(40 * s)), at: new(0, -1.5 * s, 0), type: BodyType.Static);
            w.AddWater(0.0);                                                 // 1000 kg per cubic metre by default
            var shallow = w.AddShallowWater(10 * s, -4 * s, 16 * s, 2 * s, 0.4 * s, floor: -1.0 * s);                 // the default 0.5 m cells
            var floater = w.AddBox(new((float)(0.8 * s), (float)(0.8 * s), (float)(0.8 * s)), at: new(0, 0.6 * s, 0), density: (float)(400.0 / (s * s * s)));
            var sinker = w.AddBox(new((float)(0.5 * s), (float)(0.5 * s), (float)(0.5 * s)), at: new(2 * s, 1.0 * s, 0), density: (float)(2500.0 / (s * s * s)));
            var onShallow = w.AddBox(new((float)(0.8 * s), (float)(0.8 * s), (float)(0.8 * s)), at: new(13 * s, 1.0 * s, -1 * s), density: (float)(300.0 / (s * s * s)));
            bodies = new[] { floater, sinker, onShallow };
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 781, out var ba);
        var b = Build(K, Milli, 782, out var bb);
        for (int i = 0; i < 150; i++) { a.Step(Dt); b.Step(Dt); }
        for (int k = 0; k < ba.Length; k++)
        {
            var x = ba[k].Position; var y = bb[k].Position;
            Near(x.X, y.X, 3e-3, "body " + k + " x"); Near(x.Y, y.Y, 3e-3, "body " + k + " y"); Near(x.Z, y.Z, 3e-3, "body " + k + " z");
        }

        Assert.InRange(ba[0].Position.Y, -0.5, 0.8);       // it floats
    }

    [Fact]
    public void BuoyancyAndShallowWaterFollowTheUnit2D()
    {
        P2.PhysicsWorld2D Build(double s, LengthUnit unit, int id, out P2.Body[] bodies)
        {
            var w = World2(unit, id);
            w.AddBox(new((float)(40 * s), (float)(1 * s)), at: new(0, -1.5 * s), type: BodyType.Static);
            w.AddWater(0.0);
            var floater = w.AddBox(new((float)(0.8 * s), (float)(0.8 * s)), at: new(0, 0.6 * s), density: (float)(400.0 / (s * s)));
            var sinker = w.AddBox(new((float)(0.5 * s), (float)(0.5 * s)), at: new(2 * s, 1.0 * s), density: (float)(2500.0 / (s * s)));
            bodies = new[] { floater, sinker };
            return w;
        }

        var a = Build(1.0, LengthUnit.Metres, 783, out var ba);
        var b = Build(K, Milli, 784, out var bb);
        for (int i = 0; i < 150; i++) { a.Step(Dt); b.Step(Dt); }
        for (int k = 0; k < ba.Length; k++)
        {
            var x = ba[k].Position; var y = bb[k].Position;
            Near(x.X, y.X, 3e-3, "body " + k + " x"); Near(x.Y, y.Y, 3e-3, "body " + k + " y");
        }

        Assert.InRange(ba[0].Position.Y, -0.5, 0.8);
    }

    [Fact]
    public void SwimmingFollowsTheUnit()
    {
        P3.Character Build(double s, LengthUnit unit, int id, out P3.PhysicsWorld3D w)
        {
            w = World3(unit, id);
            w.AddBox(new((float)(40 * s), (float)(1 * s), (float)(40 * s)), at: new(0, -3.5 * s, 0), type: BodyType.Static);
            w.AddWater(0.0);
            return w.AddCharacter(new(0, 1.5 * s, 0), P3.CharacterDef.Humanoid.InUnit(unit));
        }

        var ca = Build(1.0, LengthUnit.Metres, 791, out var a);
        var cb = Build(K, Milli, 792, out var b);
        for (int i = 0; i < 150; i++)
        {
            ca.Move(new Vector3F(0.02f, -0.03f, 0f), Dt);
            cb.Move(new Vector3F((float)(0.02 * K), (float)(-0.03 * K), 0f), Dt);
            a.Step(Dt); b.Step(Dt);
        }

        var pa = ca.Position; var pb = cb.Position;
        Near(pa.X, pb.X, 2e-3, "x"); Near(pa.Y, pb.Y, 2e-3, "y");
        Assert.True(pa.Y < 1.5, "the character fell into the water");
    }

    // ---------------------------------------------------------------- settle, diagnose

    [Fact]
    public void SettleFollowsTheUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id, List<P3.Body> boxes)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            for (int i = 0; i < 4; i++) { boxes.Add(w.AddBox(new((float)(1 * s), (float)(1 * s), (float)(1 * s)), at: new(0.01 * s * i, (0.6 + 1.05 * i) * s, 0), density: (float)(1000.0 / (s * s * s)))); }
            return w;
        }

        var boxesA = new List<P3.Body>(); var boxesB = new List<P3.Body>();
        var a = Build(1.0, LengthUnit.Metres, 801, boxesA);
        var b = Build(K, Milli, 802, boxesB);
        var ra = a.Settle(); var rb = b.Settle();
        Assert.True(ra.Converged == rb.Converged, "converged " + ra.Converged + " / " + rb.Converged + ", steps " + ra.StepsUsed + " / " + rb.StepsUsed);
        Assert.Equal(ra.StepsUsed, rb.StepsUsed);
        var cm = a.GetStabilityMargin(boxesA.ToArray()); var cn = b.GetStabilityMargin(boxesB.ToArray());
        Assert.Equal(cm.Margin, cn.Margin / K, 1e-3);
    }

    [Fact]
    public void DiagnoseGivesTheSameFindingsInAnyUnit()
    {
        P3.PhysicsWorld3D Build(double s, LengthUnit unit, int id)
        {
            var w = World3(unit, id);
            w.AddBox(new((float)(20 * s), (float)(1 * s), (float)(20 * s)), at: new(0, -0.5 * s, 0), type: BodyType.Static);
            // a 1000:1 chain: the diagnosis flags the mass ratio across the joints
            var prev = w.AddBox(new((float)(0.3 * s), (float)(0.3 * s), (float)(0.3 * s)), at: new(0, 6 * s, 0), type: BodyType.Static);
            for (int i = 0; i < 6; i++)
            {
                float density = (float)((i == 5 ? 1000000.0 : 1000.0) / (s * s * s));
                var link = w.AddBox(new((float)(0.3 * s), (float)(0.3 * s), (float)(0.3 * s)), at: new(0.4 * s * (i + 1), 6 * s, 0), density: density);
                w.CreateJoint(new P3.PointJointDef { BodyA = prev, BodyB = link, LocalFrameA = new P3.JointFrame3D(new Vector3F((float)(0.2 * s), 0, 0), QuaternionF.Identity), LocalFrameB = new P3.JointFrame3D(new Vector3F((float)(-0.2 * s), 0, 0), QuaternionF.Identity) });
                prev = link;
            }

            return w;
        }

        var fa = Build(1.0, LengthUnit.Metres, 811).Diagnose();
        var fb = Build(K, Milli, 812).Diagnose();
        string Dump(RecursiveAI.OnePhysics.Diagnosis.DiagnosisReport r) => string.Join(" | ", System.Linq.Enumerable.Select(r.Findings, f => f.Code + ":" + f.Severity + ":" + string.Join(",", System.Linq.Enumerable.Select(f.Values, v => v.Name + "=" + v.Value.ToString("G4")))));
        Assert.True(fa.Findings.Count == fb.Findings.Count, Dump(fa) + "  ##  " + Dump(fb));
        Assert.NotEmpty(fa.Findings);
        for (int i = 0; i < fa.Findings.Count; i++)
        {
            Assert.Equal(fa.Findings[i].Code, fb.Findings[i].Code);
            Assert.Equal(fa.Findings[i].Severity, fb.Findings[i].Severity);
        }
    }

    [Fact]
    public void DiagnoseCallsAMetreSceneInAPixelWorldTooSmall()
    {
        // A scene sized in metres but authored in a world of 100 pixels to the metre without scaling: a one-unit box is a centimetre wide.
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 821, LengthUnit = LengthUnit.Pixels(100) });
        w.AddBox(new(20, 1, 20), at: new(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new(1, 1, 1), at: new(0, 3, 0));
        var result = w.Diagnose();
        bool tooSmall = false;
        foreach (var f in result.Findings)
        {
            if (f.Code == DiagnosticCode.ImplausibleSize && f.Explanation.Contains("0.01 m per world unit")) { tooSmall = true; }
        }

        Assert.True(tooSmall, "findings: " + string.Join(", ", System.Linq.Enumerable.Select(result.Findings, f => f.Code.ToString())));
    }
}
