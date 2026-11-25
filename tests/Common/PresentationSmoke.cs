using System;
using System.IO;
using RecursiveAI.OnePhysics.Presentation;

// The follower world in both dimensions: the authority's bytes are the same with and without a follower, a recording replays to the same follower hash without the authority, and a budget holds.
// The digest combines the two followers' hashes, so the smoke apps print the same value on every platform.
internal static class PresentationSmoke
{
    internal static bool Run(out ulong digest)
    {
        digest = 0;
        var plain3 = PresentationScenes.Build3(1, false); var with3 = PresentationScenes.Build3(1, true);
        var plain2 = PresentationScenes.Build2(1, false); var with2 = PresentationScenes.Build2(1, true);
        using var rec3 = new MemoryStream(); using var rec2 = new MemoryStream();
        using (with3.Follower!.StartRecording(rec3))
        using (with2.Follower!.StartRecording(rec2))
        {
            for (int i = 0; i < 100; i++) { plain3.Advance(); with3.Advance(); plain2.Advance(); with2.Advance(); }
        }

        bool untouched = plain3.Authority.ComputeHash() == with3.Authority.ComputeHash() && plain2.Authority.ComputeHash() == with2.Authority.ComputeHash();
        rec3.Position = 0; rec2.Position = 0;
        var p3 = new PresentationPlayer(rec3); var f3 = p3.CreateFollower3D(); while (p3.ReadNextStep()) { }
        var p2 = new PresentationPlayer(rec2); var f2 = p2.CreateFollower2D(); while (p2.ReadNextStep()) { }
        bool replayed = f3.ComputeHash() == with3.Follower.ComputeHash() && f2.ComputeHash() == with2.Follower.ComputeHash() && p3.StepsPlayed == 100 && p2.StepsPlayed == 100;
        var q = with3.Follower.Quality; q.MaxBodies = 4; with3.Follower.Quality = q; with3.Advance();
        bool budget = with3.Follower.CosmeticBodyCount == 4;
        digest = with3.Follower.ComputeHash() * 31UL + with2.Follower.ComputeHash();
        return untouched && replayed && budget;
    }
}
