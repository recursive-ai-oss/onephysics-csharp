using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Tests.Common;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class SurfaceScenarios
{
    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("contacts/surfaces-2d", ctx => Run(ctx, false));
        Scenarios.Register("contacts/surfaces-3d", ctx => Run(ctx, true));
    }
    private static void Run(ScenarioContext ctx, bool spatial)
    {
        var expected = new List<ulong>();
        if (spatial) ContactSurfaceScene.Spatial(ctx.Threads, 1, true, expected.Add); else ContactSurfaceScene.Planar(ctx.Threads, 1, true, expected.Add);
        int i = 0;
        void Step(ulong hash)
        {
            if (expected[i++] != hash) throw new InvalidOperationException("Surface scalar and wide kernels differ at step " + (i - 1));
            ctx.Write(unchecked((long)hash)); ctx.EndStep();
        }
        if (spatial) ContactSurfaceScene.Spatial(ctx.Threads, 8, true, Step); else ContactSurfaceScene.Planar(ctx.Threads, 8, true, Step);
    }
}
