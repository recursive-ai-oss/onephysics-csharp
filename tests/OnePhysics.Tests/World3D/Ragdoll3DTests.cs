using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Ragdoll3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 93)
    {
        var o = new WorldOptions3D { ThreadCount = threads, WorldId = id };
        Ragdoll.ApplyRecommendedOptions(o);
        return new PhysicsWorld3D(o);
    }

    private static Body Floor(PhysicsWorld3D w) => w.AddBox(new Vector3F(200f, 1f, 200f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f));

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static QuaternionF[] Identity(int n)
    {
        var p = new QuaternionF[n];
        Array.Fill(p, QuaternionF.Identity);
        return p;
    }

    private static QuaternionF About(Vector3F axis, float angle)
    {
        double h = 0.5 * angle;
        float s = (float)Math64.Sin(h), c = (float)Math64.Cos(h);
        return new QuaternionF(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    // ---- limits --------------------------------------------------------------------------------

    /// <summary>A pelvis box on the floor and a capsule leg hinged at its side, lying out horizontally: gravity swings it down onto the swing limit.</summary>
    private static RagdollDef Pendulum(float swing)
    {
        const float R = 0.70710678f;
        var up = new QuaternionF(0f, 0f, R, R);   // joint x axis = world +y
        return new RagdollDef
        {
            SelfCollision = RagdollSelfCollision.None,
            Parts =
            [
                new RagdollPartDef { Name = "base", Shape = RagdollShapeKind.Box, ShapeHalfSize = new Vector3F(0.2f, 0.2f, 0.2f), Position = new Vector3F(0, 0.2f, 0), Mass = 200f },
                new RagdollPartDef
                {
                    Name = "arm", Parent = 0, Shape = RagdollShapeKind.Capsule, ShapeHalfSize = new Vector3F(0.3f, 0.05f, 0), Position = new Vector3F(0.5f, 1.4f, 0), Mass = 5f,
                    Rotation = new QuaternionF(0, 0, -R, R),   // capsule's y axis along world +x
                    JointPosition = new Vector3F(0.15f, 1.4f, 0), JointRotation = new QuaternionF(0, 0, -R, R),   // twist axis along +x: the arm points out sideways
                    LowerTwist = -0.2f, UpperTwist = 0.2f, SwingYHalfAngle = swing, SwingZHalfAngle = swing,
                },
            ],
        };
    }

    [Fact]
    public void SwingLimitHoldsAHangingArmUnderGravity()
    {
        var w = NewWorld();
        Floor(w);
        const float Swing = 0.6f;
        var rag = w.AddRagdoll(Pendulum(Swing), new Vector3D(0, 0, 0));
        Run(w, 300);
        var arm = rag.GetBody(1);
        // The arm's twist axis is its y axis in body space; it started along +x. Measure the angle from +x.
        var axis = arm.Rotation.Rotate(new Vector3F(0, 1, 0));
        float ang = (float)Math64.Atan2(Math.Sqrt(axis.Y * axis.Y + axis.Z * axis.Z), Math.Abs(axis.X));
        Assert.InRange(ang, Swing - 0.05f, Swing + 0.08f);   // resting on the limit, within the joint's softness
    }

    // ---- standing ------------------------------------------------------------------------------

    private static (PhysicsWorld3D W, Ragdoll R) Standing(int threads = 1, int id = 93)
    {
        var w = NewWorld(threads, id);
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 0.001, 0));
        r.DriveToPose(Identity(r.PartCount), 1f);
        return (w, r);
    }

    [Fact]
    public void PoweredHumanoidHoldsItsPoseOver600Steps()
    {
        var (w, r) = Standing();
        var pose = Identity(r.PartCount);
        float worst = 0f;
        for (int i = 0; i < 600; i++)
        {
            w.Step(Dt);
            if (i >= 60) { worst = Math.Max(worst, r.GetPoseError(pose)); }
        }

        Assert.True(r.IsPowered);
        Assert.InRange(worst, 0f, 0.06f);   // radians, the worst joint at any step after the first second
        Assert.InRange(r.Root.Position.Y, 0.93, 1.0);   // the pelvis stays at standing height (bind 0.98)
        Assert.InRange(r.GetBody(2).Position.Y, 1.5, 1.6);   // head
    }

    [Fact]
    public void LimpHumanoidCollapses()
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 0.001, 0));
        Assert.False(r.IsPowered);
        Run(w, 300);
        Assert.True(r.Root.Position.Y < 0.5);
        Assert.True(r.GetBody(2).Position.Y < 0.5);   // the head is on the floor level too

        // Strength 0 is limp as well.
        var (w2, r2) = Standing(1, 94);
        Run(w2, 30);
        r2.DriveToPose(Identity(r2.PartCount), 0f);
        Assert.False(r2.IsPowered);
        Assert.False(r2.GetJoint(7).IsPoseDriven);
        Run(w2, 300);
        Assert.True(r2.Root.Position.Y < 0.5);
    }

    [Fact]
    public void ArmTracksAPoseChangeAndSettles()
    {
        var (w, r) = Standing();
        Run(w, 60);
        var pose = Identity(r.PartCount);
        pose[3] = About(new Vector3F(0, 0, 1), 1.0f);   // left upper arm swings out sideways by a radian
        pose[4] = About(new Vector3F(1, 0, 0), -0.8f);  // the forearm bends
        r.DriveToPose(pose, 1f);
        int settled = -1;
        for (int i = 1; i <= 120; i++)
        {
            w.Step(Dt);
            if (settled < 0 && r.GetPoseError(pose) < 0.05f) { settled = i; }
        }

        Assert.InRange(settled, 1, 60);   // within a second of the change
        Assert.InRange(r.GetPoseError(pose), 0f, 0.04f);
        Assert.InRange(r.Root.Position.Y, 0.93, 1.0);
    }

    [Fact]
    public void KnockedFigureReturnsToItsPose()
    {
        var (w, r) = Standing();
        var pose = Identity(r.PartCount);
        Run(w, 60);
        var box = w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(0, 1.3, -2.0), density: 100f);   // 2.7 kg at 8 m/s from behind: a shove the toes can take (the heel is only 10 cm behind the ankle)
        box.LinearVelocity = new Vector3F(0, 0, 8f);
        float peak = 0f;
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            peak = Math.Max(peak, r.GetBody(1).LinearVelocity.Length());
        }

        Assert.True(peak > 0.1f);   // the hit moved the torso
        Run(w, 240);
        Assert.InRange(r.GetPoseError(pose), 0f, 0.06f);
        Assert.InRange(r.Root.Position.Y, 0.9, 1.02);
    }

    [Fact]
    public void FallenFigureStillRecoversItsJointPose()
    {
        var w = NewWorld();
        Floor(w);
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 0.001, 0));
        Run(w, 240);   // limp, lying on the floor with limbs in whatever shape
        var pose = Identity(r.PartCount);
        float before = r.GetPoseError(pose);
        Assert.True(before > 0.8f);
        r.DriveToPose(pose, 1f);
        Run(w, 240);
        // Limbs pinned under the body or against the floor cannot all come back, but no joint stays far off and the worst error falls by more than half.
        Assert.InRange(r.GetPoseError(pose), 0f, Math.Min(0.4f, 0.5f * before));
    }

    // ---- hooks ---------------------------------------------------------------------------------

    private static void Scenario(PhysicsWorld3D w, Ragdoll a, Ragdoll b, int step)
    {
        var pose = Identity(a.PartCount);
        if (step % 50 == 0)
        {
            pose[3] = About(new Vector3F(0, 0, 1), 0.2f * (step / 50 % 5));
            pose[9] = About(new Vector3F(1, 0, 0), -0.1f * (step / 50 % 4));
            a.DriveToPose(pose, 1f);
            b.DriveToPose(pose, 0.5f);
        }

        w.Step(Dt);
    }

    [Fact]
    public void SnapshotMidMotionContinuesBitIdentically()
    {
        var w = NewWorld();
        Floor(w);
        var a = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 0.001, 0));
        var b = w.AddRagdoll(RagdollDef.CreateHumanoid(1.2f, 30f), new Vector3D(2, 0.001, 0), About(new Vector3F(0, 1, 0), 0.5f));
        for (int i = 0; i < 70; i++) { Scenario(w, a, b, i); }
        var snap = w.Snapshot();
        var hashes1 = new ulong[80];
        for (int i = 0; i < 80; i++) { Scenario(w, a, b, 70 + i); hashes1[i] = w.ComputeHash(); }
        var end1 = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(snap, w.Snapshot());
        Assert.True(a.IsValid && b.IsPowered);
        for (int i = 0; i < 80; i++) { Scenario(w, a, b, 70 + i); Assert.Equal(hashes1[i], w.ComputeHash()); }
        Assert.Equal(end1, w.Snapshot());

        // A fresh world with the same options restores to the same bytes and then continues the same way.
        var f = NewWorld();
        f.Restore(snap);
        Assert.Equal(snap, f.Snapshot());
        var a2 = f.GetRagdolls()[0];
        var b2 = f.GetRagdolls()[1];
        for (int i = 0; i < 80; i++) { Scenario(f, a2, b2, 70 + i); Assert.Equal(hashes1[i], f.ComputeHash()); }
    }

    [Fact]
    public void ThreadCountDoesNotChangeTheHash()
    {
        ulong h1, h8;
        {
            var (w, r) = Standing(1, 95);
            w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(0, 1.3, 2.0), density: 300f).LinearVelocity = new Vector3F(0, 0, -9f);
            Run(w, 150);
            h1 = w.ComputeHash();
        }

        {
            var (w, r) = Standing(8, 95);
            w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(0, 1.3, 2.0), density: 300f).LinearVelocity = new Vector3F(0, 0, -9f);
            Run(w, 150);
            h8 = w.ComputeHash();
        }

        Assert.Equal(h1, h8);
    }

    [Fact]
    public void SharedSceneIsTheSameOnOneAndThreeThreadsAndAfterARestore()
    {
        ulong a = Ragdolls3DScene.Run(1, false), b = Ragdolls3DScene.Run(3, false), c = Ragdolls3DScene.Run(1, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
    }

    [Fact]
    public void CorruptedRagdollSnapshotsAreRejectedOrReserializeIdentically()
    {
        var (w, r) = Standing();
        Run(w, 10);
        var snap = w.Snapshot();
        var rng = new Random(5);
        var other = NewWorld();
        for (int k = 0; k < 400; k++)
        {
            var bad = (byte[])snap.Clone();
            bad[rng.Next(snap.Length)] ^= (byte)(1 << rng.Next(8));
            try
            {
                other.Restore(bad);
                Assert.Equal(bad, other.Snapshot());
            }
            catch (SnapshotFormatException) { }
        }
    }

    // ---- API -----------------------------------------------------------------------------------

    [Fact]
    public void RemoveDestroysTheFigureAndStalesTheHandle()
    {
        var (w, r) = Standing();
        var root = r.Root;
        var joint = r.GetJoint(3);
        Assert.True(joint.IsValid);
        Assert.False(r.GetJoint(0).IsValid);
        Assert.Equal(13, r.PartCount);
        r.Remove();
        Assert.False(r.IsValid);
        Assert.False(root.IsValid);
        Assert.False(joint.IsValid);
        Assert.Throws<StaleHandleException>(() => r.PartCount);
        Assert.Throws<StaleHandleException>(() => r.DriveToPose(Identity(13)));
        Assert.Throws<InvalidHandleException>(() => default(Ragdoll).StopDriving());
        Run(w, 5);
        var r2 = w.AddRagdoll(RagdollDef.Humanoid);   // the slot is reused with a new generation
        Assert.True(r2.IsValid);
        Assert.False(r.IsValid);
    }

    [Fact]
    public void DestroyingAPartBodyEndsTheRagdollButNotTheOtherBodies()
    {
        var w = NewWorld();
        var r = w.AddRagdoll(RagdollDef.Humanoid);
        var head = r.GetBody(2);
        var torso = r.GetBody(1);
        head.Destroy();
        Assert.False(r.IsValid);
        Assert.True(torso.IsValid);
        Run(w, 3);
        Assert.True(w.Snapshot().Length > 0);
        Assert.False(w.TryGetRagdoll(torso, out _));
    }

    [Fact]
    public void SetPoseTeleportsEveryPartAndSatisfiesTheJoints()
    {
        var w = NewWorld();
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(10, 0, 0));
        var pose = Identity(13);
        pose[0] = About(new Vector3F(0, 1, 0), 0.7f);
        pose[3] = About(new Vector3F(0, 0, 1), 1.0f);
        pose[7] = About(new Vector3F(1, 0, 0), 0.5f);
        r.SetPose(pose, new Vector3D(-4, 1, 3), About(new Vector3F(0, 1, 0), 1.2f));
        Assert.InRange(r.GetPoseError(pose), 0f, 1e-3f);
        // Joint anchors coincide: every joint's reaction after one step is small and the figure has not been thrown.
        w.Step(Dt);
        Assert.InRange(r.GetPoseError(pose), 0f, 0.05f);
        Assert.InRange(r.GetBody(5).Position.X, -5.0, -3.0);
    }

    [Fact]
    public void MoveToPoseCarriesTheFigureToTheKeyframe()
    {
        var w = NewWorld();
        w.Options.Gravity = default;
        var r = w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 1, 0));
        var pose = Identity(13);
        pose[3] = About(new Vector3F(0, 0, 1), 0.6f);
        r.MoveToPose(pose, new Vector3D(0, 1, 0), default, Dt);
        w.Step(Dt);
        Assert.InRange(r.GetPoseError(pose), 0f, 0.08f);
    }

    [Fact]
    public void AddRagdollWithAStartingPoseSetsIt()
    {
        var w = NewWorld();
        var pose = Identity(13);
        pose[4] = About(new Vector3F(1, 0, 0), -1f);
        var r = w.AddRagdoll(RagdollDef.Humanoid, default, default, pose);
        Assert.InRange(r.GetPoseError(pose), 0f, 1e-3f);
        Assert.False(r.IsPowered);
    }

    [Fact]
    public void SelfCollisionModesSetGroupsAndJointCollision()
    {
        foreach (var mode in new[] { RagdollSelfCollision.None, RagdollSelfCollision.NonAdjacent, RagdollSelfCollision.All })
        {
            var w = NewWorld();
            var other = w.AddRagdoll(RagdollDef.Humanoid);   // slot 0
            var def = RagdollDef.Humanoid;
            def.SelfCollision = mode;
            var r = w.AddRagdoll(def, new Vector3D(5, 0, 0));   // slot 1
            var g0 = r.GetBody(0).GetShapes()[0].GroupIndex;
            var g9 = r.GetBody(9).GetShapes()[0].GroupIndex;
            Assert.Equal(g0, g9);
            if (mode == RagdollSelfCollision.None) { Assert.Equal(-2, g0); }   // never collide: a negative group shared by the parts, distinct per ragdoll
            else { Assert.Equal(0, g0); }
            Assert.Equal(mode == RagdollSelfCollision.All, r.GetJoint(3).CollideConnected);
            Assert.Equal(0, other.GetBody(1).GetShapes()[0].GroupIndex);
        }
    }

    [Fact]
    public void HostileDefinitionsAreRefusedWithTypedErrorsAndNothingIsAdded()
    {
        var w = NewWorld();
        RagdollDef Good() => RagdollDef.Humanoid;

        var cycle = Good(); cycle.Parts[3].Parent = 4;
        Assert.Contains("cycle", Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(cycle)).Message);
        var self = Good(); self.Parts[2].Parent = 2;
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(self));
        var bad = Good(); bad.Parts[5].Parent = 99;
        Assert.Contains("99", Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(bad)).Message);
        var neg = Good(); neg.Parts[5].Parent = -7;
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(neg));
        var twoRoots = Good(); twoRoots.Parts[1].Parent = -1;
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(twoRoots));
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(new RagdollDef()));
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(new RagdollDef { Parts = new RagdollPartDef[65] }));
        var nul = Good(); nul.Parts[4] = null!;
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(nul));
        var nan = Good(); nan.Parts[1].Position = new Vector3F(float.NaN, 0, 0);
        Assert.Contains("Position", Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(nan)).Message);
        var mass = Good(); mass.Parts[1].Mass = 0f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(mass));
        var size = Good(); size.Parts[1].ShapeHalfSize = new Vector3F(1, -1, 1);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(size));
        var zero = Good(); zero.Parts[1].Rotation = default;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(zero));
        var limits = Good(); limits.Parts[1].LowerTwist = 1f; limits.Parts[1].UpperTwist = -1f;
        Assert.Throws<InvalidRagdollException>(() => w.AddRagdoll(limits));
        var swing = Good(); swing.Parts[1].SwingYHalfAngle = 0f;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(swing));
        var gain = Good(); gain.Parts[1].MaxTorque = float.PositiveInfinity;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(gain));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(Good(), new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(Good(), new Vector3D(1e9, 0, 0)));
        Assert.Throws<ArgumentNullException>(() => w.AddRagdoll(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => RagdollDef.CreateHumanoid(float.NaN));
        Assert.Empty(w.GetRagdolls());
        Assert.Equal(0, w.Core.Bodies.Handles.Count);

        var r = w.AddRagdoll(Good());
        Assert.Throws<ArgumentException>(() => r.DriveToPose(Identity(5)));
        var np = Identity(13); np[4] = new QuaternionF(float.NaN, 0, 0, 1);
        Assert.Throws<ArgumentOutOfRangeException>(() => r.DriveToPose(np));
        np[4] = default;
        Assert.Throws<ArgumentOutOfRangeException>(() => r.DriveToPose(np));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.DriveToPose(Identity(13), float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.DriveToPose(Identity(13), 1.5f));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.GetBody(13));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.SetPose(Identity(13), new Vector3D(double.PositiveInfinity, 0, 0)));
        Assert.False(r.IsPowered);
    }

    [Fact]
    public void PoseDriveOnJointsIsGuarded()
    {
        var w = NewWorld();
        var a = w.AddBox(new Vector3F(1, 1, 1));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(2, 0, 0));
        Joint hinge = w.AddHinge(a, b, new Vector3D(1, 0, 0), new Vector3F(0, 0, 1));
        Assert.Throws<InvalidJointException>(() => hinge.SetPoseDrive(QuaternionF.Identity, 1f, 1f, 1f));
        var st = w.CreateJoint(new SwingTwistJointDef { BodyA = a, BodyB = b });
        Assert.Throws<ArgumentOutOfRangeException>(() => st.SetPoseDrive(QuaternionF.Identity, -1f, 1f, 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => st.SetPoseDrive(default, 1f, 1f, 1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => st.SetPoseDrive(QuaternionF.Identity, 1f, float.NaN, 1f));
        Assert.False(st.IsPoseDriven);
        st.SetPoseDrive(QuaternionF.Identity, 100f, 10f, 50f);
        Assert.True(st.IsPoseDriven);
        st.ClearPoseDrive();
        Assert.False(st.IsPoseDriven);
    }
}
