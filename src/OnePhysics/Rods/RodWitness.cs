using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Rods;

internal struct RodWitness
{
    internal float Separation, Fraction;
    internal Vector3F Normal;
    internal Vector3D Surface;
    internal int Subshape;
}

internal static class RodGeometry
{
    internal static Vector3F Float(Vector3D a)=>new((float)a.X,(float)a.Y,(float)a.Z);
    internal static Vector3D Double(Vector3F a)=>new(a.X,a.Y,a.Z);
    internal static double Dot(Vector3D a,Vector3D b)=>a.X*b.X+a.Y*b.Y+a.Z*b.Z;
    internal static float Parameter(Vector3D a,Vector3D b,Vector3D point)
    { var d=b-a;double length=Dot(d,d);return length>1e-24?(float)System.Math.Clamp(Dot(point-a,d)/length,0,1):0; }
    internal static double Length(Vector3D d)=>System.Math.Sqrt(Dot(d,d));
    // Ericson, Real-Time Collision Detection: closest points of two finite segments.
    internal static void Closest(Vector3D p,Vector3D q,Vector3D x,Vector3D y,out float u,out float v,out Vector3D a,out Vector3D b)
    {
        var d=q-p;var e=y-x;var r=p-x;double aa=Dot(d,d),bb=Dot(e,e),c=Dot(d,r),f=Dot(e,r),s,t;
        if(aa<=1e-24&&bb<=1e-24){s=t=0;}
        else if(aa<=1e-24){s=0;t=System.Math.Clamp(f/bb,0,1);}
        else if(bb<=1e-24){t=0;s=System.Math.Clamp(-c/aa,0,1);}
        else
        {
            double de=Dot(d,e),det=aa*bb-de*de;s=det>1e-24?System.Math.Clamp((de*f-c*bb)/det,0,1):0;
            t=(de*s+f)/bb;
            if(t<0){t=0;s=System.Math.Clamp(-c/aa,0,1);}else if(t>1){t=1;s=System.Math.Clamp((de-c)/aa,0,1);}
        }
        u=(float)s;v=(float)t;a=p+d*s;b=x+e*t;
    }
}
