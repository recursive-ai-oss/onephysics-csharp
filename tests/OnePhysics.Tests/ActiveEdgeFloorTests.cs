using System;
using System.IO;
using RecursiveAI.OneMath;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public class ActiveEdgeFloorTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    [InlineData(null)]
    public void SlidingAcrossFlatTriangulatedFloorHasNoVerticalKick(bool? heightField)
    {
        const int cells = 32, samples = cells + 1;
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var material = new P3.Material3D(0, 0, 0);
        if (heightField == null) w.AddBox(new(100, 1, 100), at: new(0, -0.5, 0), type: BodyType.Static, material: material);
        else if (heightField == true) w.AddHeightField(new float[samples * samples], samples, samples, new(1, 1, 1), at: new(-16, 0, -16), material: material);
        else
        {
            var vertices = new Vector3F[samples * samples]; var indices = new int[cells * cells * 6];
            for (int z = 0; z < samples; z++) for (int x = 0; x < samples; x++) vertices[z * samples + x] = new(x - 16, 0, z - 16);
            int n = 0;
            for (int z = 0; z < cells; z++) for (int x = 0; x < cells; x++)
            {
                int a = z * samples + x, b = a + 1, c = a + samples, d = c + 1;
                indices[n++] = a; indices[n++] = c; indices[n++] = b; indices[n++] = b; indices[n++] = c; indices[n++] = d;
            }
            w.AddMesh(vertices, indices, material: material);
        }
        var box = w.AddBox(new(1, 1, 1), at: new(-12, 0.5, 0.125), material: material);
        for (int i = 0; i < 60; i++) w.Step(1f / 60);
        box.LinearVelocity = new(4, 0, 0);
        float maximum = 0; string details = "";
        for (int i = 0; i < 330; i++)
        {
            w.Step(1f / 60);
            if (Math.Abs(box.LinearVelocity.Y) > maximum)
            {
                maximum = Math.Abs(box.LinearVelocity.Y); details = " step=" + i + " omega=" + box.AngularVelocity;
                foreach (var c in box.Contacts) { details += " [" + c.Normal + ", n=" + c.PointCount; for (int p = 0; p < c.PointCount; p++) details += " p=" + c.Point(p) + " s=" + c.Separation(p); details += "]"; }
            }
        }
        string? path = Environment.GetEnvironmentVariable("ONEPHYSICS_ACTIVE_EDGE_METRICS");
        if (path != null) File.AppendAllText(path, (heightField == null ? "box" : heightField == true ? "heightfield" : "mesh") + " maxAbsVy=" + maximum.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + details + "\n");
        Assert.True(maximum <= 1e-4f, "Maximum vertical speed across flat internal edges was " + maximum);
    }
}
