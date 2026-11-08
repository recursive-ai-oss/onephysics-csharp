using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

internal readonly record struct SnapshotLocation(string Section, string Entity, int Index, int Generation, string Expression, int Element, int Offset, int Size, string Type, int RelatedBodyA = -1, int RelatedBodyB = -1, bool LiteralName = false)
{
    internal string Field
    {
        get
        {
            if (LiteralName) { return Expression; }
            string f = Expression;
            int span = f.IndexOf(".AsSpan", StringComparison.Ordinal);
            if (span >= 0) { f = f.Substring(0, span); }
            if (f.StartsWith("s.", StringComparison.Ordinal) || f.StartsWith("d.", StringComparison.Ordinal)) { f = f.Substring(2); }
            f = f.TrimStart('_');
            f = f switch
            {
                "LinearVelocity" => "linear velocity", "AngularVelocity" => "angular velocity", "Position" => "position",
                "Rotation" => "rotation", "InverseMass" => "inverse mass", "InverseInertia" => "inverse inertia",
                "SleepTime" => "sleep time", "PayloadAll" => "solver payload", "_generation" or "generation" => "generation",
                _ => f,
            };
            if (Element >= 0)
            {
                if (f is "position" or "linear velocity" or "angular velocity") { f += " " + (Element < 3 ? "xyz"[Element].ToString() : Element.ToString(CultureInfo.InvariantCulture)); }
                else { f += "[" + Element.ToString(CultureInfo.InvariantCulture) + "]"; }
            }
            return f;
        }
    }

    internal string Key => Section + "/" + Entity + "/" + Index.ToString("D10", CultureInfo.InvariantCulture) + "/" + Generation.ToString("D10", CultureInfo.InvariantCulture) + "/" + Field;
}

internal sealed class SnapshotDescription
{
    internal required byte[] Bytes;
    internal required List<SnapshotLocation> Fields;
}
