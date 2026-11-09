using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Submerged area and centroid in the shape's coordinate frame, metres and m². Zero area has a zero centroid.</summary>
public readonly struct SubmergedArea2D
{
    /// <summary>Constructs a result from an area and centroid.</summary>
    public SubmergedArea2D(float area, Vector2F centroid) { Area=area;Centroid=centroid; }
    /// <summary>Submerged area in m².</summary>
    public float Area { get; }
    /// <summary>Centre of buoyancy, metres.</summary>
    public Vector2F Centroid { get; }
}

/// <summary>Plane clipping and area moments for planar buoyancy. Polygon cores are exact; rounded outlines use a deterministic 128-direction boundary approximation.</summary>
public static class Buoyancy2D
{
    /// <summary>Clips a local shape below dot(normal,p)=offset. The normal must be unit length; finite inputs required. Circles use exact cap formulae; polygon rounding/capsules use 128 boundary directions. Allocates scratch; worlds retain it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The plane is nonfinite or its normal is not unit.</exception>
    /// <exception cref="ArgumentException">The shape is empty.</exception>
    public static SubmergedArea2D ComputeSubmergedArea(in Shape2D shape, Vector2F normal, float offset)
        => new AreaCutter2D().Cut(in shape,normal,offset);
}

internal sealed class AreaCutter2D
{
    private readonly Vector2F[] _boundary=new Vector2F[256];
    private readonly Vector2F[] _clipped=new Vector2F[258];
    internal SubmergedArea2D Cut(in Shape2D shape,Vector2F normal,float offset)
    {
        if(shape.Count==0)throw new ArgumentException("The shape is empty.",nameof(shape));
        float nl=normal.X*normal.X+normal.Y*normal.Y;
        if(!float.IsFinite(offset)||!float.IsFinite(nl)||Math.Abs(nl-1)>1e-5f)throw new ArgumentOutOfRangeException(nameof(normal),"Plane must be finite with a unit normal.");
        if(shape.Kind==ShapeKind2D.Circle)
        {
            var center=shape.Centroid;float radius=shape.Radius;
            double h=offset-normal.X*center.X-normal.Y*center.Y;
            if(h<=-radius)return default;
            if(h>=radius)return new((float)(Math.PI*radius*radius),new(center.X,center.Y));
            double a=Math.Sqrt(radius*radius-h*h);
            double area=radius*radius*(Math64.Asin(h/radius)+Math.PI/2)+h*a;
            double shift=-(2.0/3.0)*a*a*a/area;
            return new((float)area,new((float)(center.X+normal.X*shift),(float)(center.Y+normal.Y*shift)));
        }
        int count;
        if(shape.Radius==0)
        {
            count=shape.Count;for(int i=0;i<count;i++)_boundary[i]=new(shape.Vertices[i].X,shape.Vertices[i].Y);
        }
        else
        {
            count=128;
            for(int i=0;i<count;i++)
            {
                var (sin,cos)=Math32.SinCos(i*(6.283185307179586f/128));
                float best=float.NegativeInfinity;int winner=0;
                for(int j=0;j<shape.Count;j++) {var v=shape.Vertices[j];float dot=v.X*cos+v.Y*sin;if(dot>best){best=dot;winner=j;}}
                var p=shape.Vertices[winner];_boundary[i]=new(p.X+shape.Radius*cos,p.Y+shape.Radius*sin);
            }
        }
        if(count<3)return default;
        int n=0;var previous=_boundary[count-1];float pd=normal.X*previous.X+normal.Y*previous.Y-offset;
        for(int i=0;i<count;i++)
        {
            var current=_boundary[i];float cd=normal.X*current.X+normal.Y*current.Y-offset;
            if((pd<=0)!=(cd<=0)) {float t=pd/(pd-cd);_clipped[n++]=previous+(current-previous)*t;}
            if(cd<=0)_clipped[n++]=current;
            previous=current;pd=cd;
        }
        if(n<3)return default;
        double twice=0,x=0,y=0;previous=_clipped[n-1];
        for(int i=0;i<n;i++) {var current=_clipped[i];double cross=(double)previous.X*current.Y-(double)previous.Y*current.X;twice+=cross;x+=(previous.X+current.X)*cross;y+=(previous.Y+current.Y)*cross;previous=current;}
        if(twice<=1e-20)return default;
        return new((float)(twice/2),new((float)(x/(3*twice)),(float)(y/(3*twice))));
    }
}
