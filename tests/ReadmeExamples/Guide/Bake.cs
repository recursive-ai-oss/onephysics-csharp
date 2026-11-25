using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType
using RecursiveAI.OnePhysics.Bake;               // BakeOptions, BakeClip
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/bake.md.</summary>
public static class Bake
{
    public static (int Bytes, int Frames, double TopY, double MidY, ulong Hash) Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 4, 0));
        world.AddCloth(1.2f, 1.2f, 8, new Vector3D(3, 2, 0));
        world.ApplyOfflineQuality();                                  // more substeps and iterations, no sleep, surface collision on

        using BakeRecorder bake = world.BeginBake(new BakeOptions { SceneName = "crate-drop", FrameStride = 2 });
        for (int i = 0; i < 120; i++) world.Step(1f / 60);
        byte[] file = bake.Finish();                                 // the same bytes on every platform

        BakeClip clip = BakeClip.Parse(file);                        // no world needed from here on
        BakeEntity box = clip.FindRigid(crate.RawHandle.Index, crate.RawHandle.Generation)!;
        box.TrySampleRigid(0.5, out BakePose top);                   // half a second in; positions are linear, rotations nlerp
        var cloth = new Vector3D[clip.Soft[0].PointCount];
        clip.Soft[0].TrySamplePoints(0.5 + clip.FrameDuration / 2, cloth);   // between two frames
        return (file.Length, clip.FrameCount, top.Position.Y, cloth[0].Y, clip.GetFrameHash(clip.FrameCount - 1));
    }
}
