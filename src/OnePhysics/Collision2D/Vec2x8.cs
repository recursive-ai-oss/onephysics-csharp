using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Storage for up to eight 2D points inside a struct without allocating.</summary>
[InlineArray(Constants2D.MaxPolygonVertices)]
[Experimental("OPHYS0001")]
public struct Vec2x8
{
    private Vec2 _e0;
}
