using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationWhipTests
{
    /// <summary>
    /// A 12-link whip: masses fall off geometrically from 1 kg at the handle to a few grams at the tip, no gravity, the handle joint thrown at 12 rad/s, so energy runs from the handle into the light links and
    /// the tip moves much faster than the handle. Nothing dissipates, so any change of the total energy is the integrator's.
    /// </summary>
    private static Articulation MakeWhip(PhysicsWorld3D world, int substeps)
    {
        var def = new ArticulationDef { FixedBase = true, Substeps = substeps, GravityScale = 0f };
        def.AddLink(new ArticulationLinkDef { Mass = 1f, InertiaTensor = new Matrix3x3F(1, 0, 0, 0, 1, 0, 0, 0, 1) });
        float len = 0.25f;
        for (int i = 0; i < 12; i++)
        {
            float m = 1f * MathF.Pow(0.65f, i);
            float r = 0.02f;
            def.AddLink(new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0f, 0f, 1f), Origin = new Vector3F(i == 0 ? 0f : len, 0f, 0f), Mass = m,
                CenterOfMass = new Vector3F(0.5f * len, 0f, 0f),
                InertiaTensor = new Matrix3x3F(0.5f * m * r * r, 0, 0, 0, m * len * len / 12f, 0, 0, 0, m * len * len / 12f),
            });
        }

        return world.AddArticulation(def, new Vector3D(0, 30, 0));
    }

    /// <summary>Kinetic energy (zero gravity) and the largest relative energy change over the run.</summary>
    internal static (double Initial, double WorstRelative, double MaxTipSpeed) RunWhip(int substeps, int steps, float dt = 1f / 60f)
    {
        var world = new PhysicsWorld3D();
        var whip = MakeWhip(world, substeps);
        var q = new double[whip.PositionCount]; for (int k = 1; k < q.Length; k++) { q[k] = 0.3; }
        whip.SetPositions(q);
        var qd = new double[whip.VelocityCount]; qd[0] = 12.0; whip.SetVelocities(qd);
        double e0 = Kinetic(whip), worst = 0, tip = 0;
        for (int i = 0; i < steps; i++)
        {
            world.Step(dt);
            worst = Math.Max(worst, Math.Abs(Kinetic(whip) - e0) / e0);
            tip = Math.Max(tip, whip.GetLink(12).LinearVelocity.Length());
        }

        return (e0, worst, tip);
    }

    private static double Kinetic(Articulation chain)
    {
        int n = chain.VelocityCount;
        var m = new double[n * n]; var qd = new double[n];
        chain.ComputeMassMatrix(m); chain.GetVelocities(qd);
        double ke = 0;
        for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) { ke += 0.5 * qd[i] * m[i * n + j] * qd[j]; } }
        return ke;
    }

    // The earlier second-order step needed 64 substeps to hold this whip's energy to 1e-3 (it lost 18% at 4, 1.4% at 16); the fourth-order step with the spin-based automatic count does better with fewer.
    [Fact]
    public void AFastWhipKeepsItsEnergyWithTheAutomaticSubsteps()
    {
        var r = RunWhip(0, 120);
        Assert.True(r.WorstRelative < 1e-3, "relative energy change " + r.WorstRelative);
        Assert.True(r.MaxTipSpeed > 40.0, "the tip should crack: " + r.MaxTipSpeed);
    }

    [Fact]
    public void FewerFixedSubstepsLoseMoreEnergy()
    {
        double e16 = RunWhip(16, 120).WorstRelative, e32 = RunWhip(32, 120).WorstRelative, e64 = RunWhip(64, 120).WorstRelative;
        Assert.True(e32 < e16 && e64 < e32, e16 + " " + e32 + " " + e64);
    }
}
