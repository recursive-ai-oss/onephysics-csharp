using System;
using System.IO;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Presentation;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The follower world on the same bytes everywhere: the authoritative scene (a moving platform, a walking character, a block fractured at step 60) stepped beside a follower with hair, cloth or a flag, a rope and debris, the follower's hash
/// written after every step beside the authority's, and the same run recorded and replayed without the authority (the recording's digest and the replayed hash of every step).
/// </summary>
internal static class PresentationScenarios
{
    private const float Dt = PresentationScenes.Dt;

    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("presentation/follow-3d", ctx => Follow3D(ctx));
        Scenarios.Register("presentation/follow-2d", ctx => Follow2D(ctx));
        Scenarios.Register("presentation/replay-3d", ctx => Replay3D(ctx));
        Scenarios.Register("presentation/replay-2d", ctx => Replay2D(ctx));
    }

    private static void Follow3D(ScenarioContext ctx)
    {
        var s = PresentationScenes.Build3(ctx.Threads, true);
        for (int i = 0; i < 200; i++)
        {
            s.Advance();
            ctx.Write(unchecked((long)s.Authority.ComputeHash()));
            ctx.Write(unchecked((long)s.Follower!.ComputeHash()));
            ctx.Write(s.Follower.CosmeticBodyCount); ctx.Write(s.Follower.MirrorCount);
            ctx.EndStep();
        }
    }

    private static void Follow2D(ScenarioContext ctx)
    {
        var s = PresentationScenes.Build2(ctx.Threads, true);
        for (int i = 0; i < 200; i++)
        {
            s.Advance();
            ctx.Write(unchecked((long)s.Authority.ComputeHash()));
            ctx.Write(unchecked((long)s.Follower!.ComputeHash()));
            ctx.Write(s.Follower.CosmeticBodyCount); ctx.Write(s.Follower.MirrorCount);
            ctx.EndStep();
        }
    }

    private static void WriteDigest(ScenarioContext ctx, byte[] recording)
    {
        var sha = new Sha256();
        sha.Append(recording);
        ctx.Write(recording.Length);
        ctx.Write(sha.Finish());
    }

    private static void Replay3D(ScenarioContext ctx)
    {
        var s = PresentationScenes.Build3(ctx.Threads, true);
        using var ms = new MemoryStream();
        var live = new ulong[150];
        using (var rec = s.Follower!.StartRecording(ms))
        {
            for (int i = 0; i < 150; i++)
            {
                if (i == 40) s.Follower.World.AddBox(new Vector3F(0.2f, 0.2f, 0.2f), new Vector3D(2, 3, 3));            // a host change between steps
                if (i == 100) { var q = s.Follower.Quality; q.MaxBodies = 5; s.Follower.Quality = q; }                    // a budget change
                s.Advance();
                live[i] = s.Follower.ComputeHash();
            }
        }

        var bytes = ms.ToArray();
        WriteDigest(ctx, bytes); ctx.EndStep();
        var player = new PresentationPlayer(new MemoryStream(bytes));
        var follower = player.CreateFollower3D(ctx.Threads);
        int n = 0;
        while (player.ReadNextStep())
        {
            ctx.Write(unchecked((long)follower.ComputeHash())); ctx.Write(live[n] == follower.ComputeHash() ? 1 : 0); ctx.EndStep(); n++;
        }
    }

    private static void Replay2D(ScenarioContext ctx)
    {
        var s = PresentationScenes.Build2(ctx.Threads, true);
        using var ms = new MemoryStream();
        var live = new ulong[150];
        using (var rec = s.Follower!.StartRecording(ms))
        {
            for (int i = 0; i < 150; i++)
            {
                if (i == 40) s.Follower.World.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(2, 3));
                if (i == 100) { var q = s.Follower.Quality; q.MaxBodies = 5; s.Follower.Quality = q; }
                s.Advance();
                live[i] = s.Follower.ComputeHash();
            }
        }

        var bytes = ms.ToArray();
        WriteDigest(ctx, bytes); ctx.EndStep();
        var player = new PresentationPlayer(new MemoryStream(bytes));
        var follower = player.CreateFollower2D(ctx.Threads);
        int n = 0;
        while (player.ReadNextStep())
        {
            ctx.Write(unchecked((long)follower.ComputeHash())); ctx.Write(live[n] == follower.ComputeHash() ? 1 : 0); ctx.EndStep(); n++;
        }
    }
}
