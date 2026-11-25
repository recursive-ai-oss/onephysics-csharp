using System;
using System.Buffers.Binary;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>What one awake dynamic body looked like at the end of a settle run. Present in <see cref="SettleReport.Bodies"/> only when the options asked for detail.</summary>
public readonly struct SettleBodyResult
{
    /// <summary>Creates a per-body result.</summary>
    public SettleBodyResult(BodyHandle body, float speed, float angularSpeed, float force, bool supported, bool inBounds)
    {
        Body = body; Speed = speed; AngularSpeed = angularSpeed; Force = force; IsSupported = supported; IsInBounds = inBounds;
    }

    /// <summary>The body.</summary>
    public BodyHandle Body { get; }
    /// <summary>Linear speed of the centre of mass after the last step, m/s.</summary>
    public float Speed { get; }
    /// <summary>Angular speed after the last step, rad/s.</summary>
    public float AngularSpeed { get; }
    /// <summary>Unbalanced force on the body in newtons: mass times the velocity change over the last step divided by dt. Zero for a body in equilibrium; gravity for a body in free fall.</summary>
    public float Force { get; }
    /// <summary>True when the body is connected to a static or kinematic body through touching contacts and joints.</summary>
    public bool IsSupported { get; }
    /// <summary>False when the body's centre of mass is outside the bounds given in the options.</summary>
    public bool IsInBounds { get; }
}

/// <summary>
/// The outcome of <c>world.Settle</c>: whether the world reached equilibrium, how many steps it took, how far from equilibrium it still is, and which bodies never found support or left the stated bounds.
/// Residuals describe the last step run. Everything but the arrays is a plain value and is what a replay compares; the arrays are filled only when the options set <c>ReportBodies</c>.
/// </summary>
public sealed class SettleReport
{
    internal const int EncodedSize = 72;

    internal SettleReport() { }

    /// <summary>True when every awake dynamic body inside the bounds, and every soft-body and rod particle, stayed at or below the speed, angular speed and acceleration tolerances for the required number of consecutive steps.</summary>
    public bool Converged { get; internal set; }
    /// <summary>Steps taken. The world's step count advanced by this much.</summary>
    public int StepsUsed { get; internal set; }
    /// <summary>Awake dynamic bodies inside the bounds that were judged.</summary>
    public int BodiesJudged { get; internal set; }
    /// <summary>Worst linear speed over the judged bodies after the last step, m/s.</summary>
    public float WorstSpeed { get; internal set; }
    /// <summary>The body with the worst linear speed; the null handle when no body was judged.</summary>
    public BodyHandle WorstSpeedBody { get; internal set; }
    /// <summary>Worst angular speed over the judged bodies after the last step, rad/s.</summary>
    public float WorstAngularSpeed { get; internal set; }
    /// <summary>Worst unbalanced force over the judged bodies in the last step, newtons.</summary>
    public float WorstForce { get; internal set; }
    /// <summary>The body with the worst unbalanced force; the null handle when no body was judged.</summary>
    public BodyHandle WorstForceBody { get; internal set; }
    /// <summary>Worst particle speed over awake soft bodies and rods after the last step, m/s.</summary>
    public float WorstDeformableSpeed { get; internal set; }
    /// <summary>Number of dynamic bodies not connected to any static or kinematic body through touching contacts and joints (still falling, or resting only on other loose bodies).</summary>
    public int UnsupportedCount { get; internal set; }
    /// <summary>Number of dynamic bodies whose centre of mass is outside the bounds given in the options; zero when no bounds were given.</summary>
    public int OutOfBoundsCount { get; internal set; }
    /// <summary>True when the world converged, no body is unsupported and none left the bounds.</summary>
    public bool IsClean => Converged && UnsupportedCount == 0 && OutOfBoundsCount == 0;
    /// <summary>The unsupported bodies in slot order. Null unless <c>ReportBodies</c> was set.</summary>
    public BodyHandle[]? Unsupported { get; internal set; }
    /// <summary>The bodies outside the bounds in slot order. Null unless <c>ReportBodies</c> was set.</summary>
    public BodyHandle[]? OutOfBounds { get; internal set; }
    /// <summary>Every awake dynamic body's residual in slot order. Null unless <c>ReportBodies</c> was set.</summary>
    public SettleBodyResult[]? Bodies { get; internal set; }

    /// <inheritdoc/>
    public override string ToString() => string.Create(CultureInfo.InvariantCulture,
        $"Settle(converged {Converged}, {StepsUsed} steps, speed {WorstSpeed:g4} m/s, force {WorstForce:g4} N, {UnsupportedCount} unsupported, {OutOfBoundsCount} out of bounds)");

    // The replay record carries these 72 bytes; a replayed settle must reproduce them exactly.
    internal void Write(Span<byte> d)
    {
        d.Slice(0, EncodedSize).Clear();
        d[0] = Converged ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(4), StepsUsed);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(8), BodiesJudged);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(12), WorstSpeed);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(16), WorstAngularSpeed);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(20), WorstForce);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(24), WorstDeformableSpeed);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(28), UnsupportedCount);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(32), OutOfBoundsCount);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(36), WorstSpeedBody.Index);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(40), WorstSpeedBody.Generation);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(44), WorstForceBody.Index);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(48), WorstForceBody.Generation);
    }
}
