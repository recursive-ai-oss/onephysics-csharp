using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Fields;

internal sealed partial class FieldStore
{
    private Vector3F[] _moments=[];
    private int[] _momentGenerations=[];
    private float[] _buoyancyDelta=[];
    private CellEntry[] _cells=[];
    private int[] _neighbors=[];
    private readonly struct CellEntry(long x,long y,long z,int body) : IComparable<CellEntry>
    {
        internal readonly long X=x,Y=y,Z=z;
        internal readonly int Body=body;
        public int CompareTo(CellEntry other)
        {
            int c=X.CompareTo(other.X);if(c!=0)return c;
            c=Y.CompareTo(other.Y);if(c!=0)return c;
            c=Z.CompareTo(other.Z);return c!=0?c:Body.CompareTo(other.Body);
        }
        internal bool SameCell(CellEntry other)=>X==other.X&&Y==other.Y&&Z==other.Z;
    }
    private CellEntry Cell(int body,float width)
    {
        var p=Position(body);return new((long)Math.Floor(p.X/width),(long)Math.Floor(p.Y/width),(long)Math.Floor(p.Z/width),body);
    }
    internal void SetMoment(int b, Vector3F moment)
    {
        FieldDefinition.CheckVector(moment,nameof(moment));
        PrepareBodyTag(b);_moments[b]=moment;
    }
    private void PrepareBodyTag(int b)
    {
        if(b>=_moments.Length) {Array.Resize(ref _moments,b+1);Array.Resize(ref _momentGenerations,b+1);Array.Resize(ref _buoyancyDelta,b+1);}
        int generation=Host.FieldCore.Bodies.Handles.GenerationOf(b);
        if(_momentGenerations[b]!=generation){_moments[b]=default;_buoyancyDelta[b]=0;_momentGenerations[b]=generation;}
    }
    internal float BuoyancyFactor(int b)=>b<_moments.Length&&Host.FieldCore.Bodies.Handles.GenerationOf(b)==_momentGenerations[b]?BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(_buoyancyDelta[b])^0x3F800000):1;
    internal void SetBuoyancyFactor(int b,float factor) {PrepareBodyTag(b);_buoyancyDelta[b]=BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(factor)^0x3F800000);}
    internal Vector3F Moment(int b) => b<_moments.Length && Host.FieldCore.Bodies.Handles.GenerationOf(b)==_momentGenerations[b]?_moments[b]:default;
    private void StepMagnets(FieldDefinition d,float dt)
    {
        var bodies=Host.FieldCore.Bodies;
        if(_cells.Length<bodies.HighWater) {Array.Resize(ref _cells,bodies.HighWater);Array.Resize(ref _neighbors,bodies.HighWater);}
        int count=0;
        for(int b=0;b<bodies.HighWater;b++) if(bodies.IsLive(b)&&Matches(b,d)&&Dot(Moment(b),Moment(b))>0) _cells[count++]=Cell(b,d.Radius);
        Array.Sort(_cells,0,count);
        int occupancy=0;
        for(int i=0;i<count;i++)
        {
            occupancy=i>0&&_cells[i].SameCell(_cells[i-1])?occupancy+1:1;
            if(occupancy>64)throw new InvalidOperationException("Magnetism cutoff cell contains more than 64 dipoles; use a smaller cutoff or fewer tags.");
        }
        // A cutoff-sized grid bounds each body's 27-cell neighbourhood to 1728 entries.
        // Cells sort lexicographically; force sums still visit neighbours by body slot.
        for(int a=0;a<bodies.HighWater;a++)
        {
            if(!bodies.IsLive(a)||!bodies.IsDynamic(a)||!Matches(a,d))continue;
            var ma=Moment(a);if(Dot(ma,ma)==0)continue;
            var p=Position(a);var origin=Cell(a,d.Radius);int n=0;
            for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++)
            {
                var key=new CellEntry(origin.X+x,origin.Y+y,origin.Z+z,-1);int lo=0,hi=count;
                while(lo<hi) {int mid=lo+(hi-lo)/2;if(_cells[mid].CompareTo(key)<0)lo=mid+1;else hi=mid;}
                for(int j=lo;j<count&&_cells[j].SameCell(key);j++)if(_cells[j].Body!=a)_neighbors[n++]=_cells[j].Body;
            }
            Array.Sort(_neighbors,0,n);
            for(int j=0;j<n;j++)
            {
                int b=_neighbors[j];var r=PositionDifference(p,Position(b));float l2=Dot(r,r);if(l2==0||l2>d.Radius*d.Radius)continue;
                float len=float.Sqrt(l2),soft=Math.Max(len,d.Softening);var normal=r/len;var mb=Moment(b);
                float an=Dot(ma,normal),bn=Dot(mb,normal);
                // Gradient of the dipole energy, with normalized physical constant in Strength.
                var force=(mb*an+ma*bn+normal*(Dot(ma,mb)-5*an*bn))*(3*d.Strength/(soft*soft*soft*soft));
                Apply(a,force,default,dt,false);
            }
        }
    }
}
