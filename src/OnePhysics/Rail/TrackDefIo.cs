using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Snapshot encoding of a <see cref="TrackDef"/>, shared by both engines' rail sections.</summary>
internal static class TrackDefIo
{
    internal static void Write(StateWriter w, TrackDef d)
    {
        w.NamedDouble(d.Start.X); w.NamedDouble(d.Start.Y); w.NamedDouble(d.Start.Z); w.NamedSingle(d.StartHeading);
        w.NamedInt32(d.Elements.Length);
        foreach (var e in d.Elements) { w.NamedByte((byte)e.Kind); w.NamedSingle(e.Length); w.NamedSingle(e.StartCurvature); w.NamedSingle(e.EndCurvature); }
        int ne = d.Elevation?.Length ?? 0;
        w.NamedInt32(ne);
        for (int i = 0; i < ne; i++) { w.NamedSingle(d.Elevation![i].X); w.NamedSingle(d.Elevation![i].Y); }
        int nc = d.Cant?.Length ?? 0;
        w.NamedInt32(nc);
        for (int i = 0; i < nc; i++) { w.NamedSingle(d.Cant![i].X); w.NamedSingle(d.Cant![i].Y); }
        w.NamedSingle(d.Gauge); w.NamedSingle(d.Friction); w.NamedBool(d.Collide);
    }

    internal static TrackDef Read(StateReader r)
    {
        var d = new TrackDef { Start = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble()), StartHeading = r.ReadSingle() };
        int n = r.ReadCount(13, "track elements");
        if (n > TrackDef.MaxElements) { throw r.Error("a track with more than " + TrackDef.MaxElements.ToString(CultureInfo.InvariantCulture) + " elements"); }
        var els = new TrackElement[n];
        for (int i = 0; i < n; i++)
        {
            byte kind = r.ReadByte(); float len = r.ReadSingle(), k0 = r.ReadSingle(), k1 = r.ReadSingle();
            if (kind > (byte)TrackElementKind.Transition) { throw r.Error("unknown track element kind"); }
            els[i] = kind == 0 ? TrackElement.Straight(len) : kind == 1 ? TrackElement.Arc(len, 1f / k0) : TrackElement.Transition(len, k0, k1);
            if (kind == 1 && !(k0 != 0f)) { throw r.Error("an arc without curvature"); }
            // The element is rebuilt from its kind, so the saved curvatures must be exactly what that kind writes (a straight's are zero, an arc's are equal and survive 1/(1/k)); otherwise the next save would differ.
            if (BitConverter.SingleToInt32Bits(els[i].StartCurvature) != BitConverter.SingleToInt32Bits(k0) || BitConverter.SingleToInt32Bits(els[i].EndCurvature) != BitConverter.SingleToInt32Bits(k1)) { throw r.Error("a track element whose curvatures do not match its kind"); }
        }

        d.Elements = els;
        int ne = r.ReadCount(8, "track elevation points");
        if (ne > TrackDef.MaxProfilePoints) { throw r.Error("too many elevation points"); }
        if (ne > 0) { var p = new Vector2F[ne]; for (int i = 0; i < ne; i++) { p[i] = new Vector2F(r.ReadSingle(), r.ReadSingle()); } d.Elevation = p; }
        int nc = r.ReadCount(8, "track cant points");
        if (nc > TrackDef.MaxProfilePoints) { throw r.Error("too many cant points"); }
        if (nc > 0) { var p = new Vector2F[nc]; for (int i = 0; i < nc; i++) { p[i] = new Vector2F(r.ReadSingle(), r.ReadSingle()); } d.Cant = p; }
        d.Gauge = r.ReadSingle(); d.Friction = r.ReadSingle(); d.Collide = r.ReadBool();
        try { d.Validate(); }
        catch (ArgumentException e) { throw r.Error("invalid track: " + e.Message); }
        return d;
    }
}
