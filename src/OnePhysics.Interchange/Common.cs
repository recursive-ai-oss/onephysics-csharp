using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Xml;
using System.Xml.Linq;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Thrown when an input file cannot be imported. <see cref="Format"/> names the file format and <see cref="Location"/> the element, line or key that was wrong; the message says what was expected there.</summary>
public sealed class InterchangeException : Exception
{
    /// <summary>Creates the exception.</summary>
    /// <param name="format">The format being read ("URDF", "MJCF", "glTF", "Tiled", "LDtk").</param>
    /// <param name="location">Where in the input the problem is (an element path, a line, a JSON path).</param>
    /// <param name="message">What was expected.</param>
    public InterchangeException(string format, string location, string message) : base(format + ": " + location + ": " + message)
    {
        Format = format; Location = location;
    }

    /// <summary>The file format being read.</summary>
    public string Format { get; }

    /// <summary>Where in the input the problem is.</summary>
    public string Location { get; }
}

/// <summary>Something in the input that is valid but that the importer cannot express; the import goes on without it.</summary>
/// <param name="Code">A short stable identifier, for example "mimic-joint".</param>
/// <param name="Location">Where in the input it was found.</param>
/// <param name="Message">What was left out or approximated, and why.</param>
public readonly record struct InterchangeWarning(string Code, string Location, string Message);

/// <summary>A triangle mesh handed back by a <see cref="MeshResolver"/>.</summary>
public sealed class InterchangeMesh
{
    /// <summary>Creates a mesh from vertices and triangle indices (three per triangle).</summary>
    /// <exception cref="ArgumentException">There are no vertices or no complete triangle, or an index is out of range.</exception>
    public InterchangeMesh(Vector3F[] vertices, int[] indices)
    {
        ArgumentNullException.ThrowIfNull(vertices);
        ArgumentNullException.ThrowIfNull(indices);
        if (vertices.Length == 0 || indices.Length < 3 || indices.Length % 3 != 0) { throw new ArgumentException("A mesh needs vertices and a multiple of three indices."); }
        foreach (int i in indices) { if ((uint)i >= (uint)vertices.Length) { throw new ArgumentException("Index " + i + " is outside the " + vertices.Length + " vertices."); } }
        Vertices = vertices; Indices = indices;
    }

    /// <summary>The vertices.</summary>
    public Vector3F[] Vertices { get; }

    /// <summary>Triangle indices, three per triangle.</summary>
    public int[] Indices { get; }
}

/// <summary>Finds the mesh a file refers to by name (a URDF mesh filename, an MJCF asset file). The importer never touches the file system: the host decides what a name means. Return null for an unknown name; the importer then reports a warning and leaves the shape out.</summary>
/// <param name="name">The name exactly as written in the file.</param>
public delegate InterchangeMesh? MeshResolver(string name);

internal static class Io
{
    public const int MaxChars = 64 * 1024 * 1024;
    public const int MaxDepth = 256;

    public static XDocument LoadXml(string text, string format)
    {
        ArgumentNullException.ThrowIfNull(text);
        if (text.Length > MaxChars) { throw new InterchangeException(format, "document", "the text is longer than " + MaxChars + " characters."); }
        try
        {
            // Depth first: XDocument builds iteratively, but a hostile file should be refused before it holds a hundred thousand nested elements.
            using (var probe = XmlReader.Create(new StringReader(text), Settings()))
            {
                while (probe.Read())
                {
                    if (probe.Depth > MaxDepth) { throw new InterchangeException(format, "line " + ((IXmlLineInfo)probe).LineNumber, "elements are nested more than " + MaxDepth + " deep."); }
                }
            }

            using var reader = XmlReader.Create(new StringReader(text), Settings());
            return XDocument.Load(reader, LoadOptions.SetLineInfo);
        }
        catch (XmlException e)
        {
            throw new InterchangeException(format, "line " + e.LineNumber, "not well-formed XML (" + e.Message + ").");
        }
    }

    private static XmlReaderSettings Settings() => new()
    {
        DtdProcessing = DtdProcessing.Prohibit, XmlResolver = null, IgnoreComments = true, IgnoreProcessingInstructions = true, MaxCharactersInDocument = MaxChars, MaxCharactersFromEntities = 0,
    };

    public static string ReadAll(Stream stream, string format, long limit)
    {
        ArgumentNullException.ThrowIfNull(stream);
        using var reader = new StreamReader(stream, new System.Text.UTF8Encoding(false), true, 4096, true);
        var buffer = new char[8192];
        var sb = new System.Text.StringBuilder();
        int n;
        while ((n = reader.Read(buffer, 0, buffer.Length)) > 0)
        {
            sb.Append(buffer, 0, n);
            if (sb.Length > limit) { throw new InterchangeException(format, "document", "the input is longer than " + limit + " characters."); }
        }

        return sb.ToString();
    }

    public static string Where(XElement e)
    {
        var parts = new List<string>();
        for (XElement? x = e; x != null && parts.Count < 32; x = x.Parent)
        {
            string n = x.Name.LocalName;
            string? id = (string?)x.Attribute("name");
            parts.Add(id != null ? n + "[" + id + "]" : n);
        }

        parts.Reverse();
        int line = ((IXmlLineInfo)e).HasLineInfo() ? ((IXmlLineInfo)e).LineNumber : 0;
        return string.Join("/", parts) + (line > 0 ? " (line " + line + ")" : "");
    }

    public const double Bound = 1e9;

    public static double Number(string text, string format, string where, string what)
    {
        if (!double.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) || !double.IsFinite(v) || Math.Abs(v) > Bound)
        {
            throw new InterchangeException(format, where, what + " must be a finite number within ±1e9, found '" + Clip(text) + "'.");
        }

        return v;
    }

    public static double[] Numbers(string text, int count, string format, string where, string what)
    {
        var parts = text.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != count) { throw new InterchangeException(format, where, what + " needs " + count + " numbers, found " + parts.Length + "."); }
        var r = new double[count];
        for (int i = 0; i < count; i++) { r[i] = Number(parts[i], format, where, what); }
        return r;
    }

    public static string Clip(string s) => s.Length <= 40 ? s : string.Concat(s.AsSpan(0, 40), "...");

    /// <summary>A rotation about a unit axis by an angle in radians, from the correctly rounded sine and cosine.</summary>
    public static QuaternionD AxisAngle(double x, double y, double z, double angle)
    {
        double len = Math.Sqrt(x * x + y * y + z * z);
        var (s, c) = Math64.SinCos(0.5 * angle);
        double k = s / len;
        return new QuaternionD(x * k, y * k, z * k, c);
    }

    public static QuaternionD Normalize(QuaternionD q)
    {
        double l = q.Length();
        return new QuaternionD(q.X / l, q.Y / l, q.Z / l, q.W / l);
    }

    public static Vector3F ToF(Vector3D v) => new((float)v.X, (float)v.Y, (float)v.Z);

    public static QuaternionF ToF(QuaternionD q) { q = Normalize(q); return new QuaternionF((float)q.X, (float)q.Y, (float)q.Z, (float)q.W); }

    public static Matrix3x3F ToF(Matrix3x3D m) => new((float)m.M11, (float)m.M12, (float)m.M13, (float)m.M21, (float)m.M22, (float)m.M23, (float)m.M31, (float)m.M32, (float)m.M33);
}
