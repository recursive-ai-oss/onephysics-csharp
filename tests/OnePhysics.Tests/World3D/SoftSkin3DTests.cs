using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Tests.Core;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

// Skinned soft bodies: a sleeve (an open tube along y) on a two-joint arm. The reference skinning below is written from the documented formula, not from the engine's code.
public class SoftSkin3DTests
{
    private const float Dt = 1f / 60f;
    private const float Length = 1f;
    private const int Rings = 11;      // along y
    private const int Around = 12;
    private static readonly int[] OneParent = { -1 };
    private static readonly int[] ArmParents = { -1, 0 };

    private sealed class Sleeve
    {
        public SoftBodyMesh Mesh = null!;
        public Vector3F[] Rest = Array.Empty<Vector3F>();
        public Vector3F[] Normals = Array.Empty<Vector3F>();
        public int[] Joints = Array.Empty<int>();
        public float[] Weights = Array.Empty<float>();
    }

    private static Sleeve MakeSleeve()
    {
        var s = new Sleeve();
        int n = Rings * Around;
        s.Rest = new Vector3F[n]; s.Normals = new Vector3F[n]; s.Joints = new int[4 * n]; s.Weights = new float[4 * n];
        var tris = new List<int>();
        for (int r = 0; r < Rings; r++)
        {
            for (int a = 0; a < Around; a++)
            {
                int i = r * Around + a;
                float ang = 2f * MathF.PI * a / Around, y = Length * r / (Rings - 1);
                float c = (float)Math.Cos(ang), sn = (float)Math.Sin(ang);
                s.Rest[i] = new Vector3F(0.1f * c, y, 0.1f * sn);
                s.Normals[i] = new Vector3F(c, 0f, sn);
                // joint 0 below the middle ring, joint 1 above; the two blend over the middle three rings
                float t = Math.Clamp((r - (Rings - 1) / 2f + 1f) / 2f, 0f, 1f);
                s.Joints[4 * i] = 0; s.Joints[4 * i + 1] = 1;
                s.Weights[4 * i] = 1f - t; s.Weights[4 * i + 1] = t;
                if (r + 1 < Rings)
                {
                    int b = (a + 1) % Around;
                    int i00 = r * Around + a, i01 = r * Around + b, i10 = (r + 1) * Around + a, i11 = (r + 1) * Around + b;
                    tris.AddRange(new[] { i00, i01, i10, i01, i11, i10 });
                }
            }
        }

        s.Mesh = SoftBodyMesh.FromTriangles(s.Rest, tris.ToArray());
        return s;
    }

    private static readonly Vector3F[] ArmBind = { new Vector3F(0f, 0f, 0f), new Vector3F(0f, 0.5f, 0f) };

    private static SoftSkin MakeSkin(Sleeve s, float cuffMax, float freeMax, float backDist = 0f, float backRadius = 0f)
    {
        int n = s.Rest.Length;
        var max = new float[n]; var bd = new float[n]; var br = new float[n];
        for (int i = 0; i < n; i++)
        {
            bool cuff = i < Around;
            max[i] = cuff ? cuffMax : freeMax;
            if (!cuff) { bd[i] = backDist; br[i] = backRadius; }
        }

        return SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, s.Weights, max, bd, br, s.Normals);
    }

    private static SoftBodyDef MakeDef(Sleeve s, SoftSkin skin, Vector3D at) => new()
    {
        Mesh = s.Mesh, Skin = skin, Position = at, Mass = 0.5f, EdgeCompliance = 0f, BendCompliance = 1e-3f, SelfCollision = false, ParticleRadius = 0.01f, AllowSleep = false,
    };

    private static PhysicsWorld3D NewWorld(int threads = 1, Action<WorldOptions3D>? tweak = null) => World3DTests.NewWorld(threads, 41, tweak);

    // the animation: the upper joint swings about z and x, the lower one swings about z; world poses, joint 1 carried by joint 0
    private static void Pose(int step, Vector3D at, SkinJointPose[] joints)
    {
        float a0 = 0.5f * (float)Math.Sin(step * 0.07), a1 = 0.9f * (float)Math.Sin(step * 0.11 + 0.5);
        var qLow = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), a0);
        var qHigh = qLow * QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), a1);
        var o1 = qLow.Rotate(ArmBind[1] - ArmBind[0]);
        joints[0] = new SkinJointPose(at + new Vector3D(ArmBind[0].X, ArmBind[0].Y, ArmBind[0].Z), qLow);
        joints[1] = new SkinJointPose(new Vector3D(at.X + o1.X, at.Y + o1.Y, at.Z + o1.Z), qHigh);
    }

    // scalar linear blend skinning, from the formula in docs/spec/softbody.md
    private static (double X, double Y, double Z, Vector3F N) Skinned(Sleeve s, int i, SkinJointPose[] joints, float scale = 1f)
    {
        double ax = 0, ay = 0, az = 0;
        float sx = 0, sy = 0, sz = 0;
        for (int k = 0; k < 4; k++)
        {
            float w = s.Weights[4 * i + k];
            if (w == 0f) { continue; }
            int j = s.Joints[4 * i + k];
            var q = joints[j].Rotation;
            float ql = float.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            q = new QuaternionF(q.X / ql, q.Y / ql, q.Z / ql, q.W / ql);
            var rest = s.Rest[i];
            var bind = ArmBind[j];
            var r = q.Rotate(new Vector3F(rest.X * scale - bind.X * scale, rest.Y * scale - bind.Y * scale, rest.Z * scale - bind.Z * scale));
            ax += (double)w * (joints[j].Position.X + (double)r.X);
            ay += (double)w * (joints[j].Position.Y + (double)r.Y);
            az += (double)w * (joints[j].Position.Z + (double)r.Z);
            var m = q.Rotate(s.Normals[i]);
            sx += w * m.X; sy += w * m.Y; sz += w * m.Z;
        }

        float l = float.Sqrt(sx * sx + sy * sy + sz * sz);
        return (ax, ay, az, new Vector3F(sx / l, sy / l, sz / l));
    }

    [Fact]
    public void KinematicParticlesEqualTheScalarSkinningBitForBit()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var at = new Vector3D(1, 2, 3);
        // every particle with a maximum distance of 0 is kinematic: the whole sleeve is skinned
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0f), at));
        var joints = new SkinJointPose[2];
        for (int step = 0; step < 40; step++)
        {
            Pose(step, at, joints);
            b.SetSkeletonTransforms(joints);
            w.Step(Dt);
            for (int i = 0; i < s.Rest.Length; i++)
            {
                var want = Skinned(s, i, joints);
                var got = b.GetPosition(i);
                Assert.Equal(BitConverter.DoubleToInt64Bits(want.X), BitConverter.DoubleToInt64Bits(got.X));
                Assert.Equal(BitConverter.DoubleToInt64Bits(want.Y), BitConverter.DoubleToInt64Bits(got.Y));
                Assert.Equal(BitConverter.DoubleToInt64Bits(want.Z), BitConverter.DoubleToInt64Bits(got.Z));
                var sk = b.GetSkinnedPosition(i);
                Assert.Equal(BitConverter.DoubleToInt64Bits(want.X), BitConverter.DoubleToInt64Bits(sk.X));
            }
        }
    }

    [Fact]
    public void PoseFromRotationsFollowsTheSkeleton()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var at = new Vector3D(0, 1, 0);
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0f), at));
        var d1 = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), 0.7f);
        var d0 = QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), -0.3f);
        b.SetSkeletonPose(new[] { d0, d1 }, at);
        // rotations are relative to the parent in bind axes: joint 1 is turned by d0 * d1, and sits where joint 0 carries its bind offset
        var qHigh = d0 * d1;
        var p1 = d0.Rotate(ArmBind[1]);
        var joints = new[] { new SkinJointPose(at, d0), new SkinJointPose(new Vector3D(at.X + p1.X, at.Y + p1.Y, at.Z + p1.Z), qHigh) };
        for (int i = 0; i < s.Rest.Length; i++)
        {
            var want = Skinned(s, i, joints);
            var got = b.GetSkinnedPosition(i);
            Assert.True(Math.Abs(want.X - got.X) < 1e-5 && Math.Abs(want.Y - got.Y) < 1e-5 && Math.Abs(want.Z - got.Z) < 1e-5, "particle " + i);
        }
    }

    [Fact]
    public void MaximumDistanceAndBackstopHoldAfterEveryStep()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        // a ball the sleeve falls onto and an arm that swings hard: contacts and constraints push against the limits
        w.AddSphere(0.25f, new Vector3D(0.3, 0.6, 0), type: BodyType.Static);
        var at = new Vector3D(0, 0.3, 0);
        const float D = 0.05f, Bd = 0.04f, R = 0.04f;
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, D, Bd, R), at));
        var joints = new SkinJointPose[2];
        int moved = 0;
        for (int step = 0; step < 240; step++)
        {
            Pose(step, at, joints);
            b.SetSkeletonTransforms(joints);
            w.Step(Dt);
            for (int i = Around; i < s.Rest.Length; i++)
            {
                var t = Skinned(s, i, joints);
                var p = b.GetPosition(i);
                double dx = p.X - t.X, dy = p.Y - t.Y, dz = p.Z - t.Z;
                double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                Assert.True(dist <= D * (1 + 1e-4) + 1e-5, "max distance, step " + step + ", particle " + i + ": " + dist);
                double ex = dx + t.N.X * Bd, ey = dy + t.N.Y * Bd, ez = dz + t.N.Z * Bd;
                double back = Math.Sqrt(ex * ex + ey * ey + ez * ez);
                Assert.True(back >= R * (1 - 1e-4) - 1e-5, "back-stop, step " + step + ", particle " + i + ": " + back);
                if (dist > 0.5 * D) { moved++; }
            }
        }

        Assert.True(moved > 100, "the limits were never tested: " + moved);
    }

    [Fact]
    public void BackstopBindsWhereMaximumDistanceAloneWouldNot()
    {
        // one free particle ring pulled inwards by gravity-free pressure from a pinned ball is hard to arrange; instead place a particle inside the back-stop and step
        var s = MakeSleeve();
        var w = NewWorld(1, o => o.Gravity = default);
        var at = new Vector3D(0, 0, 0);
        const float D = 0.1f, Bd = 0.05f, R = 0.05f;
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, D, Bd, R), at));
        int p = 5 * Around + 3;
        var n = Skinned(s, p, Bind(at), 1f).N;
        var t = b.GetSkinnedPosition(p);
        // 0.03 behind the surface: inside the back-stop sphere (centre 0.05 behind, radius 0.05)
        b.SetPosition(p, new Vector3D(t.X - n.X * 0.03, t.Y - n.Y * 0.03, t.Z - n.Z * 0.03));
        w.Step(Dt);
        var q = b.GetPosition(p);
        double dx = q.X - t.X + n.X * Bd, dy = q.Y - t.Y + n.Y * Bd, dz = q.Z - t.Z + n.Z * Bd;
        Assert.True(Math.Sqrt(dx * dx + dy * dy + dz * dz) >= R * (1 - 1e-4) - 1e-5);
        double mx = q.X - t.X, my = q.Y - t.Y, mz = q.Z - t.Z;
        Assert.True(Math.Sqrt(mx * mx + my * my + mz * mz) <= D * (1 + 1e-4) + 1e-5);
    }

    private static SkinJointPose[] Bind(Vector3D at) => new[]
    {
        new SkinJointPose(at, QuaternionF.Identity),
        new SkinJointPose(new Vector3D(at.X, at.Y + 0.5, at.Z), QuaternionF.Identity),
    };

    [Fact]
    public void BindPoseLeavesTheBodyAtRest()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var at = new Vector3D(0, 5, 0);
        var def = MakeDef(s, MakeSkin(s, 0f, 0.05f, 0.03f, 0.03f), at);
        def.GravityScale = 0f;
        var b = w.AddSoftBody(def);
        var start = new Vector3D[s.Rest.Length];
        b.CopyPositions(start);
        for (int step = 0; step < 120; step++) { w.Step(Dt); }
        double worst = 0, fastest = 0; int wi = -1;
        for (int i = 0; i < s.Rest.Length; i++)
        {
            var p = b.GetPosition(i);
            double dd = Math.Abs(p.X - start[i].X) + Math.Abs(p.Y - start[i].Y) + Math.Abs(p.Z - start[i].Z);
            if (dd > worst) { worst = dd; wi = i; }
            var v = b.GetVelocity(i);
            fastest = Math.Max(fastest, Math.Abs(v.X) + Math.Abs(v.Y) + Math.Abs(v.Z));
        }

        Assert.True(worst < 2e-5, "drift " + worst + " at " + wi + " moved " + (b.GetPosition(wi) - start[wi]) + " masses " + b.Mass);
        Assert.True(fastest < 1e-2, "speed " + fastest);
    }

    [Fact]
    public void ASleeveHangsWithinItsLimitsUnderGravityAndSleepsWhenThePoseIsStill()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var at = new Vector3D(0, 5, 0);
        var def = MakeDef(s, MakeSkin(s, 0f, 0.08f), at);
        def.AllowSleep = true;
        var b = w.AddSoftBody(def);
        for (int step = 0; step < 600 && !b.IsAsleep; step++) { w.Step(Dt); }
        Assert.True(b.IsAsleep);
        // a changed pose wakes it
        b.SetSkeletonPose(new[] { QuaternionF.Identity, QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), 0.3f) }, at);
        Assert.False(b.IsAsleep);
    }

    [Fact]
    public void TeleportPlacesTheBodyOnItsSkinnedPositionsWithZeroVelocity()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0.05f), new Vector3D(0, 3, 0)));
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        var joints = new SkinJointPose[2];
        var at = new Vector3D(10, 1, -4);
        Pose(17, at, joints);
        b.SetSkeletonTransforms(joints, teleport: true);
        for (int i = 0; i < s.Rest.Length; i++)
        {
            var want = Skinned(s, i, joints);
            var got = b.GetPosition(i);
            Assert.Equal(want.X, got.X); Assert.Equal(want.Y, got.Y); Assert.Equal(want.Z, got.Z);
            Assert.Equal(default, b.GetVelocity(i));
        }
    }

    [Fact]
    public void SnapshotRestoreAndReplayAreBitIdentical()
    {
        var s = MakeSleeve();
        var at = new Vector3D(0, 2, 0);
        PhysicsWorld3D Make(out SoftBody b)
        {
            var w = NewWorld();
            w.AddSphere(0.3f, new Vector3D(0.35, 1.5, 0), type: BodyType.Static);
            b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0.06f, 0.05f, 0.05f), at));
            return w;
        }

        var joints = new SkinJointPose[2];
        var w1 = Make(out var b1);
        var hashes = new List<ulong>();
        byte[] snap = Array.Empty<byte>();
        for (int step = 0; step < 90; step++)
        {
            if (step == 40) { snap = w1.Snapshot(); }
            Pose(step, at, joints);
            b1.SetSkeletonTransforms(joints);
            w1.Step(Dt);
            if (step >= 40) { hashes.Add(w1.ComputeHash()); }
        }

        var w2 = NewWorld();
        w2.Restore(snap);
        var b2 = w2.GetSoftBody(0);
        for (int step = 40; step < 90; step++)
        {
            Pose(step, at, joints);
            b2.SetSkeletonTransforms(joints);
            w2.Step(Dt);
            Assert.Equal(hashes[step - 40], w2.ComputeHash());
        }

        Assert.Equal(w1.Snapshot(), w2.Snapshot());
        // a restore into a world that already holds the body gives the same bytes back
        w2.Restore(snap);
        Assert.Equal(snap, w2.Snapshot());
    }

    [Fact]
    public void ThreadCountDoesNotChangeASkinnedWorld()
    {
        ulong Run(int threads)
        {
            var s = MakeSleeve();
            var w = NewWorld(threads);
            w.AddBox(new Vector3F(10f, 1f, 10f), new Vector3D(0, 0.2, 0), type: BodyType.Static);
            var at = new Vector3D(0, 1, 0);
            var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0.06f, 0.05f, 0.05f), at));
            var joints = new SkinJointPose[2];
            for (int step = 0; step < 60; step++) { Pose(step, at, joints); b.SetSkeletonTransforms(joints); w.Step(Dt); }
            return w.ComputeHash();
        }

        Assert.Equal(Run(1), Run(4));
    }

    [Fact]
    public void SteppingAndPosingAllocateNothing()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        w.AddBox(new Vector3F(10f, 1f, 10f), new Vector3D(0, 0.2, 0), type: BodyType.Static);
        var at = new Vector3D(0, 1, 0);
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0.06f, 0.05f, 0.05f), at));
        var joints = new SkinJointPose[2];
        var rot = new[] { QuaternionF.Identity, QuaternionF.Identity };
        for (int i = 0; i < 30; i++) { Pose(i, at, joints); b.SetSkeletonTransforms(joints); w.Step(Dt); }
        AllocMeter.Settle();
        long bytes = 0;
        for (int i = 30; i < 60; i++)
        {
            Pose(i, at, joints);
            bytes += AllocMeter.Of(() => { b.SetSkeletonTransforms(joints); b.SetSkeletonPose(rot, at); b.SetSkeletonTransforms(joints); w.Step(Dt); });
        }

        Assert.Equal(0, bytes);
    }

    [Fact]
    public void HostileSkinsAreRefusedWithTypedErrors()
    {
        var s = MakeSleeve();
        int n = s.Rest.Length;
        var max = new float[n];
        void Throws(SoftBodyError error, string field, Func<SoftSkin> make)
        {
            var ex = Assert.Throws<SoftBodyException>(make);
            Assert.Equal(error, ex.Error);
            Assert.Contains(field, ex.Message);
        }

        Throws(SoftBodyError.BadCount, "parents", () => SoftSkin.Create(s.Mesh, ReadOnlySpan<int>.Empty, ArmBind, s.Joints, s.Weights, max));
        Throws(SoftBodyError.BadCount, "bindPositions", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind.AsSpan(0, 1), s.Joints, s.Weights, max));
        Throws(SoftBodyError.BadCount, "jointIndices", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints.AsSpan(1), s.Weights, max));
        Throws(SoftBodyError.BadCount, "maxDistance", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, s.Weights, max.AsSpan(1)));
        Throws(SoftBodyError.BadIndex, "parents", () => SoftSkin.Create(s.Mesh, new[] { -1, 1 }, ArmBind, s.Joints, s.Weights, max));
        Throws(SoftBodyError.BadIndex, "parents", () => SoftSkin.Create(s.Mesh, new[] { 0, 0 }, ArmBind, s.Joints, s.Weights, max));
        var badJoint = (int[])s.Joints.Clone(); badJoint[7] = 2;
        Throws(SoftBodyError.BadIndex, "jointIndices", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, badJoint, s.Weights, max));
        var nanW = (float[])s.Weights.Clone(); nanW[3] = float.NaN;
        Throws(SoftBodyError.NonFinite, "weights", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, nanW, max));
        var negW = (float[])s.Weights.Clone(); negW[3] = -1f;
        Throws(SoftBodyError.OutOfRange, "weights", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, negW, max));
        var zeroW = (float[])s.Weights.Clone(); Array.Clear(zeroW, 8, 4);
        Throws(SoftBodyError.OutOfRange, "weights", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, zeroW, max));
        var badMax = (float[])max.Clone(); badMax[2] = float.PositiveInfinity;
        Throws(SoftBodyError.NonFinite, "maxDistance", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, s.Weights, badMax));
        badMax[2] = -0.1f;
        Throws(SoftBodyError.OutOfRange, "maxDistance", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, s.Weights, badMax));
        var nanBind = new[] { new Vector3F(0, 0, 0), new Vector3F(float.NaN, 0, 0) };
        Throws(SoftBodyError.NonFinite, "bindPositions", () => SoftSkin.Create(s.Mesh, ArmParents, nanBind, s.Joints, s.Weights, max));
        // a back-stop larger than distance + maximum distance leaves nothing allowed
        var radius = new float[n]; Array.Fill(radius, 0.5f);
        var dist = new float[n]; Array.Fill(dist, 0.4f);
        Throws(SoftBodyError.OutOfRange, "backstopRadius", () => SoftSkin.Create(s.Mesh, ArmParents, ArmBind, s.Joints, s.Weights, max, dist, radius, s.Normals));

        // definition and handle checks
        var skin = MakeSkin(s, 0f, 0.05f);
        var other = SoftBodyMesh.Cloth(1f, 1f, 3, 3);
        var w = NewWorld();
        var ex2 = Assert.Throws<SoftBodyException>(() => w.AddSoftBody(new SoftBodyDef { Mesh = other, Skin = skin }));
        Assert.Equal(SoftBodyError.BadCount, ex2.Error);
        var plain = w.AddSoftBody(new SoftBodyDef { Mesh = other });
        Assert.False(plain.HasSkin);
        Assert.Throws<InvalidOperationException>(() => plain.SetSkeletonPose(new[] { QuaternionF.Identity }, default));
        var b = w.AddSoftBody(MakeDef(s, skin, default));
        Assert.True(b.HasSkin);
        Assert.Equal(2, b.SkeletonJointCount);
        Assert.Throws<ArgumentException>(() => b.SetSkeletonPose(new[] { QuaternionF.Identity }, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetSkeletonPose(new[] { QuaternionF.Identity, default }, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetSkeletonPose(new[] { QuaternionF.Identity, QuaternionF.Identity }, new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetSkeletonPose(new[] { QuaternionF.Identity, QuaternionF.Identity }, new Vector3D(3e6, 0, 0)));
        Assert.Throws<ArgumentException>(() => b.SetSkeletonTransforms(new SkinJointPose[3]));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.SetSkeletonTransforms(new SkinJointPose[2]));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.GetSkinnedPosition(n));
        b.Destroy();
        Assert.Throws<StaleHandleException>(() => b.SetSkeletonTransforms(new SkinJointPose[2]));
    }

    [Fact]
    public void DebugGeometryCarriesTheSkinnedTargets()
    {
        var s = MakeSleeve();
        var w = NewWorld();
        var b = w.AddSoftBody(MakeDef(s, MakeSkin(s, 0f, 0.05f), new Vector3D(0, 2, 0)));
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.SoftBodies, g);
        var entry = g.SoftBodies[0];
        Assert.True(entry.FirstTargetPoint >= 0);
        var t = b.GetSkinnedPosition(5);
        var p = g.Points[entry.FirstTargetPoint + 5];
        Assert.Equal(t.X, p.X); Assert.Equal(t.Y, p.Y); Assert.Equal(t.Z, p.Z);
    }

    [Fact]
    public void TheSharedSceneIsTheSameOnAnyThreadCountAndAfterARestore()
    {
        ulong a = SoftSkinned3DScene.Run(1, false), b = SoftSkinned3DScene.Run(3, false), c = SoftSkinned3DScene.Run(1, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
    }
}
