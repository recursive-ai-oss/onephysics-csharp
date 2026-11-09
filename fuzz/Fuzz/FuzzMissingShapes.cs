using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

internal static class FuzzMissingShapes
{
    private static float Single(byte[] bytes, int offset) => offset <= bytes.Length - 4 ? BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(offset, 4))) : 0f;
    private static Vector3F Vertex(byte[] bytes, int offset) => new(Single(bytes, offset), Single(bytes, offset+4), Single(bytes, offset+8));

    public static void Plane(byte[] bytes)
    {
        try
        {
            var p=ConvexShape3D.Plane(Vertex(bytes,0),Single(bytes,12));
            _=RecursiveAI.OnePhysics.LowLevel.Gjk3D.Distance(p,ConvexShape3D.Sphere(1),RigidTransformF.Identity);
        }
        catch(ArgumentOutOfRangeException) { }
    }
    public static void MassOffset2D(byte[] bytes)
    {
        try { _=Shape2D.CreateCircle(default,1).WithCenterOfMassOffset(new(Single(bytes,0),Single(bytes,4))).ComputeMass(); }
        catch(InvalidShapeException) { }
    }
    public static void MassOffset3D(byte[] bytes)
    {
        try { _=ConvexShape3D.Sphere(1).WithCenterOfMassOffset(Vertex(bytes,0)).GetMassProperties(); }
        catch(ArgumentOutOfRangeException) { }
    }
    public static void Frustum(byte[] bytes)
    {
        try { _ = ConvexShape3D.TaperedCylinder(Single(bytes,0),Single(bytes,4),Single(bytes,8)).GetMassProperties(); }
        catch (ArgumentOutOfRangeException) { }
    }
    public static void Triangle(byte[] bytes)
    {
        try { _ = ConvexShape3D.Triangle(Vertex(bytes,0),Vertex(bytes,12),Vertex(bytes,24),Single(bytes,36)).GetMassProperties(); }
        catch (ArgumentOutOfRangeException) { }
    }
    public static void HeightField(byte[] bytes)
    {
        int n=bytes.Length>0 ? bytes[0] : 0;
        var heights=new float[n];
        for(int i=0;i<n;i++) heights[i]=Single(bytes,5+i*4);
        try
        {
            var field=new HeightField2D(heights, Single(bytes,1));
            for(int i=0;i<field.IntervalCount;i++) _=field.GetSegment(i);
        }
        catch(ArgumentOutOfRangeException) { }
        catch(InvalidShapeException) { }
    }
}
