using System;
using RecursiveAI.OnePhysics.Articulations;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationModelTests
{
    private static readonly D3 Gravity = new(0, -9.81, 0);

    private static LinkSpec Link(int parent, JointKind kind, D3 axis, D3 at, double mass, D3 com, M3 inertia) =>
        new() { Parent = parent, Kind = kind, Axis = axis, PlacementRot = Q4.Identity, PlacementPos = at, Mass = mass, Com = com, Inertia = inertia };

    private static M3 Diag(double a, double b, double c) => new(a, 0, 0, 0, b, 0, 0, 0, c);

    /// <summary>A fixed root and two revolute links about z with point masses at the ends: the planar two-link arm of every robotics text.</summary>
    private static ArticulationModel TwoLinkArm(double m1, double l1, double m2, double l2)
    {
        var z = new D3(0, 0, 1);
        var tiny = Diag(1e-9, 1e-9, 1e-9);
        return new ArticulationModel(new[]
        {
            Link(-1, JointKind.Fixed, z, D3.Zero, 1, D3.Zero, Diag(1, 1, 1)),
            Link(0, JointKind.Revolute, z, D3.Zero, m1, new D3(l1, 0, 0), tiny),
            Link(1, JointKind.Revolute, z, new D3(l1, 0, 0), m2, new D3(l2, 0, 0), tiny),
        });
    }

    [Fact]
    public void TwoLinkArmMassMatrixAndGravityMatchTheClosedForm()
    {
        const double m1 = 2.0, l1 = 0.7, m2 = 1.3, l2 = 0.5;
        var arm = TwoLinkArm(m1, l1, m2, l2);
        double q1 = 0.4, q2 = -0.9;
        arm.Q[0] = q1; arm.Q[1] = q2;
        arm.ForwardKinematics();
        var m = new double[4];
        arm.MassMatrix(m);
        double c2 = Math.Cos(q2);
        double m11 = (m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2 * m2 * l1 * l2 * c2, m12 = m2 * l2 * l2 + m2 * l1 * l2 * c2, m22 = m2 * l2 * l2;
        Assert.Equal(m11, m[0], 1e-6); Assert.Equal(m12, m[1], 1e-6); Assert.Equal(m12, m[2], 1e-6); Assert.Equal(m22, m[3], 1e-6);
        var tau = new double[2];
        arm.InverseDynamics(new double[2], Gravity, ReadOnlySpan<double>.Empty, tau);
        double g = 9.81;
        Assert.Equal((m1 + m2) * g * l1 * Math.Cos(q1) + m2 * g * l2 * Math.Cos(q1 + q2), tau[0], 1e-6);
        Assert.Equal(m2 * g * l2 * Math.Cos(q1 + q2), tau[1], 1e-6);
    }

    /// <summary>A tree with every joint kind, a floating base and a branch.</summary>
    private static ArticulationModel MixedTree()
    {
        var rng = new Random(7);
        double R() => rng.NextDouble() * 2 - 1;
        var specs = new LinkSpec[7];
        specs[0] = Link(-1, JointKind.Free, D3.Zero, D3.Zero, 3, new D3(0.1, 0, 0.05), Diag(0.4, 0.5, 0.3));
        specs[1] = Link(0, JointKind.Revolute, new D3(0, 0, 1), new D3(0.5, 0, 0), 1.2, new D3(0.2, 0.05, 0), Diag(0.02, 0.03, 0.01));
        specs[2] = Link(1, JointKind.Prismatic, new D3(1, 0, 0), new D3(0.4, 0, 0), 0.8, new D3(0.1, 0, 0.1), Diag(0.01, 0.02, 0.01));
        specs[3] = Link(2, JointKind.Spherical, D3.Zero, new D3(0.3, 0.1, 0), 0.6, new D3(0.1, 0, 0), Diag(0.01, 0.01, 0.02));
        specs[4] = Link(0, JointKind.Revolute, new D3(0, 1, 0), new D3(-0.5, 0, 0.2), 1.5, new D3(-0.2, 0, 0), Diag(0.05, 0.02, 0.04));
        specs[5] = Link(4, JointKind.Fixed, D3.Zero, new D3(-0.3, 0, 0), 0.4, new D3(-0.05, 0.02, 0), Diag(0.005, 0.005, 0.005));
        specs[6] = Link(5, JointKind.Revolute, new D3(0, 0, 1), new D3(-0.2, 0, 0), 0.5, new D3(-0.1, 0, 0), Diag(0.004, 0.006, 0.005));
        specs[3].PlacementRot = new Q4(0.1, 0.2, 0.3, 0.9).Normalized();
        var model = new ArticulationModel(specs);
        for (int i = 0; i < model.N; i++)
        {
            int po = model.PosOff[i];
            switch (model.Kind[i])
            {
                case JointKind.Free:
                {
                    var q = new Q4(R(), R(), R(), R() + 2).Normalized();
                    model.Q[po] = R(); model.Q[po + 1] = R(); model.Q[po + 2] = R(); model.Q[po + 3] = q.X; model.Q[po + 4] = q.Y; model.Q[po + 5] = q.Z; model.Q[po + 6] = q.W;
                    break;
                }

                case JointKind.Spherical:
                {
                    var q = new Q4(R(), R(), R(), R() + 2).Normalized();
                    model.Q[po] = q.X; model.Q[po + 1] = q.Y; model.Q[po + 2] = q.Z; model.Q[po + 3] = q.W;
                    break;
                }

                case JointKind.Revolute: case JointKind.Prismatic: model.Q[po] = R(); break;
            }
        }

        for (int d = 0; d < model.TotalDof; d++) { model.Qd[d] = R(); }
        model.ForwardKinematics();
        return model;
    }

    [Fact]
    public void InverseDynamicsEqualsMassMatrixTimesAccelerationPlusBias()
    {
        var model = MixedTree();
        int n = model.TotalDof;
        var rng = new Random(3);
        var qdd = new double[n];
        for (int i = 0; i < n; i++) { qdd[i] = rng.NextDouble() * 2 - 1; }
        var tau = new double[n]; var bias = new double[n]; var m = new double[n * n];
        model.InverseDynamics(qdd, Gravity, ReadOnlySpan<double>.Empty, tau);
        model.InverseDynamics(new double[n], Gravity, ReadOnlySpan<double>.Empty, bias);
        model.MassMatrix(m);
        for (int i = 0; i < n; i++)
        {
            double s = bias[i];
            for (int j = 0; j < n; j++) { s += m[i * n + j] * qdd[j]; Assert.Equal(m[i * n + j], m[j * n + i], 1e-9); }
            Assert.True(Math.Abs(s - tau[i]) <= 1e-9 * (1 + Math.Abs(tau[i])), "row " + i + ": " + s + " vs " + tau[i]);
        }
    }

    [Fact]
    public void ForwardDynamicsInvertsInverseDynamics()
    {
        var model = MixedTree();
        int n = model.TotalDof;
        var rng = new Random(11);
        var tau = new double[n];
        for (int i = 0; i < n; i++) { tau[i] = rng.NextDouble() * 4 - 2; }
        var qdd = new double[n];
        Assert.True(model.ForwardDynamics(tau, new double[n], Gravity, ReadOnlySpan<double>.Empty, qdd));
        var back = new double[n];
        model.InverseDynamics(qdd, Gravity, ReadOnlySpan<double>.Empty, back);
        for (int i = 0; i < n; i++) { Assert.True(Math.Abs(back[i] - tau[i]) <= 1e-9 * (1 + Math.Abs(tau[i])), "dof " + i + ": " + back[i] + " vs " + tau[i]); }
    }

    [Fact]
    public void PointJacobianMatchesFiniteDifferenceOfForwardKinematics()
    {
        var model = MixedTree();
        int n = model.TotalDof;
        var local = new D3(0.1, -0.2, 0.15);
        var jac = new double[3 * n];
        model.PointJacobian(6, local, jac, Span<double>.Empty);
        var p0 = model.WorldPoint(6, local);
        const double eps = 1e-6;
        for (int d = 0; d < n; d++)
        {
            var saveQd = (double[])model.Qd.Clone();
            Array.Clear(model.Qd);
            model.Qd[d] = 1.0;
            var q0 = (double[])model.Q.Clone();
            model.IntegratePositions(eps);
            model.ForwardKinematics();
            var p1 = model.WorldPoint(6, local);
            Assert.Equal((p1.X - p0.X) / eps, jac[d], 1e-4); Assert.Equal((p1.Y - p0.Y) / eps, jac[n + d], 1e-4); Assert.Equal((p1.Z - p0.Z) / eps, jac[2 * n + d], 1e-4);
            Array.Copy(q0, model.Q, q0.Length); Array.Copy(saveQd, model.Qd, n);
            model.ForwardKinematics();
        }
    }

    [Fact]
    public void UndampedPendulumKeepsItsEnergyAndPeriod()
    {
        var z = new D3(0, 0, 1);
        const double L = 1.0, m = 2.0;
        var pend = new ArticulationModel(new[]
        {
            Link(-1, JointKind.Fixed, z, D3.Zero, 1, D3.Zero, Diag(1, 1, 1)),
            Link(0, JointKind.Revolute, z, D3.Zero, m, new D3(0, -L, 0), Diag(1e-9, 1e-9, 1e-9)),
        });
        pend.Q[0] = 0.05;
        pend.ForwardKinematics();
        double h = 1.0 / 240.0;
        var tau = new double[1]; var dadd = new double[1]; var qdd = new double[1];
        double e0 = pend.KineticEnergy() + pend.PotentialEnergy(Gravity), maxDrift = 0;
        int crossings = 0; double firstCross = 0, lastCross = 0, prev = pend.Q[0];
        for (int step = 1; step <= 2400; step++)
        {
            // Drift, kick, drift: positions and velocities both at whole steps, second order.
            pend.IntegratePositions(0.5 * h);
            pend.ForwardKinematics();
            Assert.True(pend.ForwardDynamics(tau, dadd, Gravity, ReadOnlySpan<double>.Empty, qdd));
            pend.Qd[0] += h * qdd[0];
            pend.IntegratePositions(0.5 * h);
            pend.ForwardKinematics();
            maxDrift = Math.Max(maxDrift, Math.Abs(pend.KineticEnergy() + pend.PotentialEnergy(Gravity) - e0));
            if (prev > 0 && pend.Q[0] <= 0) { crossings++; if (crossings == 1) { firstCross = step * h; } lastCross = step * h; }
            prev = pend.Q[0];
        }

        double amplitudeEnergy = m * 9.81 * L * (1 - Math.Cos(0.05));
        Assert.True(maxDrift < 0.001 * amplitudeEnergy, "energy drift " + maxDrift + " of " + amplitudeEnergy);
        double period = (lastCross - firstCross) / (crossings - 1);
        Assert.Equal(2 * Math.PI * Math.Sqrt(L / 9.81), period, 0.01);
    }
}
