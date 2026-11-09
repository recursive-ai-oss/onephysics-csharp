using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

// Rod contact search. Candidates (shapes near each segment, segment pairs near each other) are gathered once per substep with a
// margin, from the broadphase and a sweep-and-prune, instead of being rediscovered every pass. Each pass then re-runs the exact tests
// on the cached candidates only, so the cache never changes which contacts exist: a pass checks that no segment or moving body has
// left the margin since the gather and regathers if one has. Sweeps run in parallel (one task per rod, or per block of pairs) against
// the poses at the start of the pass and write to per-rod and per-pair slots; the solves then apply the results serially in rod and
// pair order, so the output does not depend on the thread count.
internal sealed partial class RodStore
{
    // Distance in metres by which a segment or a body's near box may move after a gather before the candidate lists are rebuilt.
    private const double GatherMargin=0.004;
    private const int PairBlock=32;
    // A proof that a rod segment (or end point) does not reach a shape. Clear is a lower bound on the gap above the skin over the whole substep
    // sweep, EndGap one at its end, both for the endpoints A and B (the end positions of that sweep). The proof carries to later passes while the
    // endpoints drift less than Clear, and to the next substep while the endpoints and the (unpushed) rigid body hold still enough (EndGap).
    private struct Cert { internal float Clear,EndGap;internal int Version,Substep,Epoch;internal Vector3D A,B; }
    private struct WorldHit { internal int Segment,KeySegment,Shape;internal RodWitness Witness;internal float Fraction; }
    private struct PairHit { internal bool Valid;internal float U,V;internal Vector3F Normal; }
    private sealed class RodGather
    {
        internal double[] Box=[];
        internal int[] CandStart=[],Cand=[],CapStart=[];
        // No-contact certificates, one per candidate entry (segment sweeps) and per cap entry. The previous gather's lists are kept for one rebuild so certificates carry over.
        internal Cert[] SegCert=[],CapCert=[],OldSegCert=[],OldCapCert=[];
        internal int[] OldCand=[],OldCandStart=[],OldCapStart=[];
        internal int CandCount,OldCandCount;
        internal bool Built;
        internal readonly List<int> Shapes=new();
        internal Vector3D[] NearMin=[],NearMax=[];
        internal WorldHit[] Hits=new WorldHit[8];
        internal int HitCount;
        internal bool Active;
    }
    // Test hook: rediscover every candidate on every pass and trust no certificate. Results must not change.
    internal bool FreshSearch;
    private RodGather?[] _gathers=[];
    private bool _gatherValid;
    private int _gatherEpoch,_substep;
    private int[] _bodyVersion=[];
    private bool[] _pushedMark=[];
    private readonly List<int> _pushed=new();
    private Vector3D[] _gatherShift=[];
    private float[] _gatherMotion=[];
    private double _gatherReach;
    private PairHit[] _pairHits=new PairHit[16];
    private int _pairCount;
    private int[] _taskStops=new int[16];

    // Called at the start of each substep; step is the substep's index, and the first one of a step also starts a new certificate epoch.
    private void PrepareGather(int step)
    {
        _substep=step;
        if(step==0)
        {
            _gatherEpoch++;
            Array.Clear(_bodyVersion);Array.Clear(_pushedMark);_pushed.Clear();
        }
        int bodies=_bodyStart.Length;
        if(_bodyVersion.Length<bodies){Array.Resize(ref _bodyVersion,bodies);Array.Resize(ref _pushedMark,bodies);Array.Resize(ref _gatherShift,bodies);Array.Resize(ref _gatherMotion,bodies);}
        if(_gathers.Length!=Data.Length)Array.Resize(ref _gathers,Data.Length);
        _gatherValid=false;
    }
    private void NoteBodyPush(int b)
    {
        _bodyVersion[b]++;
        if(!_pushedMark[b]){_pushedMark[b]=true;_pushed.Add(b);}
    }
    private void NearFrame(int body,out Vector3D shift,out float motion)
    {
        BodyPose(body,_time1,out var position,out _);shift=position-_bodyEnd[body];motion=BodyMotionBound(body,_time1-_time0);
    }
    // The rigid body's motion over the substep, split into the translation of its collision pose and a bound on how far rotation can carry a point of the shape (see BodyMotionBound).
    private void BodyMotion(int b,out Vector3D translation,out double turn)
    {
        BodyPose(b,_time1,out var end,out var z);translation=end-_bodySubStart[b];var a=_rotationSubStart[b];
        float sign=a.X*z.X+a.Y*z.Y+a.Z*z.Z+a.W*z.W<0?-1:1;
        double x=sign*z.X-a.X,y=sign*z.Y-a.Y,w=sign*z.W-a.W,zz=sign*z.Z-a.Z;
        turn=(float)(4*Math.Sqrt(x*x+y*y+zz*zz+w*w)*Host.BodyExtent(b));
    }
    private static void SegmentBounds(RodParticles d,int s,out Vector3D min,out Vector3D max)
    {
        var a0=d.Previous[s];var b0=d.Previous[s+1];var a1=d.PositionData[s];var b1=d.PositionData[s+1];
        min=new(Math.Min(Math.Min(a0.X,b0.X),Math.Min(a1.X,b1.X)),Math.Min(Math.Min(a0.Y,b0.Y),Math.Min(a1.Y,b1.Y)),Math.Min(Math.Min(a0.Z,b0.Z),Math.Min(a1.Z,b1.Z)));
        max=new(Math.Max(Math.Max(a0.X,b0.X),Math.Max(a1.X,b1.X)),Math.Max(Math.Max(a0.Y,b0.Y),Math.Max(a1.Y,b1.Y)),Math.Max(Math.Max(a0.Z,b0.Z),Math.Max(a1.Z,b1.Z)));
    }
    private bool GatherStale()
    {
        if(!_gatherValid||FreshSearch)return true;
        foreach(int b in _pushed)
        {
            NearFrame(b,out var shift,out float motion);var was=_gatherShift[b];
            double move=Math.Max(Math.Abs(shift.X-was.X),Math.Max(Math.Abs(shift.Y-was.Y),Math.Abs(shift.Z-was.Z)))+Math.Abs(motion-_gatherMotion[b]);
            if(move>GatherMargin*.5)return true;
        }
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];var g=_gathers[r];if(d==null||g==null||!g.Active)continue;
            for(int s=0;s<d.Length.Length;s++)
            {
                SegmentBounds(d,s,out var min,out var max);int k=6*s;const double slack=GatherMargin*.5;
                if(min.X<g.Box[k]-slack||min.Y<g.Box[k+1]-slack||min.Z<g.Box[k+2]-slack||max.X>g.Box[k+3]+slack||max.Y>g.Box[k+4]+slack||max.Z>g.Box[k+5]+slack)return true;
            }
        }
        return false;
    }
    private void EnsureGather()
    {
        if(GatherStale())Gather();
    }
    private void Gather()
    {
        foreach(int b in _pushed)_pushedMark[b]=false;
        _pushed.Clear();
        var bodies=Host.RodCore.Bodies;double reach=0;
        // Shapes of moving bodies are tested at the pose they have at the end of the substep, which can be far from where the broadphase holds them.
        for(int b=0;b<bodies.HighWater;b++)
        {
            if(!bodies.IsLive(b)||bodies.TypeOf(b)==BodyType.Static)continue;
            NearFrame(b,out var shift,out float motion);_gatherShift[b]=shift;_gatherMotion[b]=motion;
            reach=Math.Max(reach,Math.Max(Math.Abs(shift.X),Math.Max(Math.Abs(shift.Y),Math.Abs(shift.Z)))+motion);
        }
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];
            if(d==null||!d.CollisionEnabled){if(_gathers[r]!=null)_gathers[r]!.Active=false;continue;}
            (_gathers[r]??=new RodGather()).Active=true;
        }
        _gatherReach=reach;
        Host.RodCore.Executor.For(Data.Length,new GatherRodTask(this));
        GatherPairs();
        _gatherValid=true;
    }
    private readonly struct GatherRodTask(RodStore store):IParallelBody
    {
        public void Run(int task)
        {
            var d=store.Data[task];var g=store._gathers[task];
            if(d!=null&&g!=null&&g.Active)store.GatherRod(d,g,store._gatherReach);
        }
    }
    private void GatherRod(RodParticles d,RodGather g,double reach)
    {
        int segments=d.Length.Length,particles=segments+1;
        (g.Cand,g.OldCand)=(g.OldCand,g.Cand);(g.CandStart,g.OldCandStart)=(g.OldCandStart,g.CandStart);(g.CapStart,g.OldCapStart)=(g.OldCapStart,g.CapStart);
        (g.SegCert,g.OldSegCert)=(g.OldSegCert,g.SegCert);(g.CapCert,g.OldCapCert)=(g.OldCapCert,g.CapCert);
        g.OldCandCount=g.CandCount;bool carry=g.Built;
        if(g.Box.Length<6*segments){g.Box=new double[6*segments];}
        if(g.CandStart.Length<segments+1){g.CandStart=new int[segments+1];g.CapStart=new int[particles+1];}
        Vector3D lo=new(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity),hi=new(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity);
        for(int s=0;s<segments;s++)
        {
            SegmentBounds(d,s,out var min,out var max);int k=6*s;
            g.Box[k]=min.X;g.Box[k+1]=min.Y;g.Box[k+2]=min.Z;g.Box[k+3]=max.X;g.Box[k+4]=max.Y;g.Box[k+5]=max.Z;
            lo=new(Math.Min(lo.X,min.X),Math.Min(lo.Y,min.Y),Math.Min(lo.Z,min.Z));hi=new(Math.Max(hi.X,max.X),Math.Max(hi.Y,max.Y),Math.Max(hi.Z,max.Z));
        }
        double radius=d.Material.Radius,pad=radius+.001+GatherMargin;
        // The broadphase boxes of a moving body are shifted and padded by the same bounds as the exact test, so the moving trees are asked with that reach added.
        var list=g.Shapes;list.Clear();
        Host.CollectShapes(lo-new Vector3D(pad,pad,pad),hi+new Vector3D(pad,pad,pad),false,list);
        double wide=pad+reach;
        Host.CollectShapes(lo-new Vector3D(wide,wide,wide),hi+new Vector3D(wide,wide,wide),true,list);
        list.Sort();
        int shapes=0;
        if(g.NearMin.Length<list.Count){int n=Math.Max(16,list.Count*2);g.NearMin=new Vector3D[n];g.NearMax=new Vector3D[n];}
        for(int i=0;i<list.Count;i++)
        {
            int s=list[i];if(!Host.Matches(d,s))continue;
            NearBox(s,radius,GatherMargin,out var nmin,out var nmax);
            list[shapes]=s;g.NearMin[shapes]=nmin;g.NearMax[shapes]=nmax;shapes++;
        }
        int count=0;
        for(int s=0;s<segments;s++)
        {
            g.CandStart[s]=count;int k=6*s;
            for(int i=0;i<shapes;i++)
            {
                var nmin=g.NearMin[i];var nmax=g.NearMax[i];
                if(g.Box[k]>nmax.X||g.Box[k+3]<nmin.X||g.Box[k+1]>nmax.Y||g.Box[k+4]<nmin.Y)continue;
                if(!Host.Planar&&(g.Box[k+2]>nmax.Z||g.Box[k+5]<nmin.Z))continue;
                if(count==g.Cand.Length)Array.Resize(ref g.Cand,Math.Max(16,count*2));
                g.Cand[count++]=list[i];
            }
        }
        g.CandStart[segments]=count;g.CandCount=count;
        int caps=0;
        for(int p=0;p<particles;p++)
        {
            g.CapStart[p]=caps;int s=Math.Min(p,segments-1);caps+=g.CandStart[s+1]-g.CandStart[s];
        }
        g.CapStart[particles]=caps;
        if(g.SegCert.Length<count)g.SegCert=new Cert[Math.Max(16,count*2)];
        if(g.CapCert.Length<caps)g.CapCert=new Cert[Math.Max(16,caps*2)];
        Array.Clear(g.SegCert,0,count);Array.Clear(g.CapCert,0,caps);
        if(carry&&g.OldCandStart.Length>=segments+1)
        {
            // Both lists of a segment are in ascending shape order, so one merge walk matches the shapes that are still candidates.
            for(int s=0;s<segments;s++)
            {
                int j=g.OldCandStart[s],jEnd=g.OldCandStart[s+1];
                for(int k=g.CandStart[s];k<g.CandStart[s+1];k++)
                {
                    int shape=g.Cand[k];while(j<jEnd&&g.OldCand[j]<shape)j++;
                    if(j==jEnd)break;
                    if(g.OldCand[j]!=shape)continue;
                    g.SegCert[k]=g.OldSegCert[j];
                }
            }
            for(int p=0;p<particles;p++)
            {
                int s=Math.Min(p,segments-1);int j=g.OldCandStart[s],jEnd=g.OldCandStart[s+1];
                for(int k=g.CandStart[s];k<g.CandStart[s+1];k++)
                {
                    int shape=g.Cand[k];while(j<jEnd&&g.OldCand[j]<shape)j++;
                    if(j==jEnd)break;
                    if(g.OldCand[j]!=shape)continue;
                    g.CapCert[g.CapStart[p]+k-g.CandStart[s]]=g.OldCapCert[g.OldCapStart[p]+j-g.OldCandStart[s]];
                }
            }
        }
        g.Built=true;
    }
    // The box the exact test (Near) compares a segment's swept bounds with, grown by a margin.
    private void NearBox(int shape,double radius,double margin,out Vector3D min,out Vector3D max)
    {
        Host.Bounds(shape,out min,out max);int body=Host.RodCore.Shapes.Owner[shape];
        NearFrame(body,out var shift,out float motion);
        double pad=motion+radius+.001+margin;
        min+=shift-new Vector3D(pad,pad,pad);max+=shift+new Vector3D(pad,pad,pad);
    }

    private double[] _pMinX=[],_pMinY=[],_pMinZ=[],_pMaxX=[],_pMaxY=[],_pMaxZ=[];
    private int[] _pRod=[],_pSeg=[],_cellLow=[],_cellHigh=[],_cellStart=[],_cellItems=[],_bigBoxes=[];
    private ulong[] _pairKeys=new ulong[16];
    private const int MaxBoxCells=64;
    // Pairs of segments (of different rods, or of one rod that collides with itself) whose padded boxes come within the margin, found with a
    // uniform grid: each box is listed in every cell it overlaps (boxes spanning more than MaxBoxCells cells are tested against everything), and a pair
    // is reported by the first cell the two boxes share. The grid's order does not matter, because the pairs are then sorted by their unique
    // (rod, segment, rod, segment) key, which is the order the solves use.
    private void GatherPairs()
    {
        int total=0;
        for(int r=0;r<Data.Length;r++)if(Data[r] is {} rod&&rod.CollisionEnabled)total+=rod.Length.Length;
        if(_pMinX.Length<total)
        {
            int n=Math.Max(total,_pMinX.Length*2);
            _pMinX=new double[n];_pMinY=new double[n];_pMinZ=new double[n];_pMaxX=new double[n];_pMaxY=new double[n];_pMaxZ=new double[n];
            _pRod=new int[n];_pSeg=new int[n];_cellLow=new int[3*n];_cellHigh=new int[3*n];
        }
        int count=0,pairs=0;
        double lowX=double.PositiveInfinity,lowY=double.PositiveInfinity,lowZ=double.PositiveInfinity,highX=double.NegativeInfinity,highY=double.NegativeInfinity,highZ=double.NegativeInfinity,width=0;
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];if(d==null||!d.CollisionEnabled)continue;var g=_gathers[r]!;double radius=d.Material.Radius+Skin(d)+GatherMargin;
            for(int s=0;s<d.Length.Length;s++)
            {
                int k=6*s;double x0=g.Box[k]-radius,y0=g.Box[k+1]-radius,z0=g.Box[k+2]-radius,x1=g.Box[k+3]+radius,y1=g.Box[k+4]+radius,z1=g.Box[k+5]+radius;
                _pMinX[count]=x0;_pMinY[count]=y0;_pMinZ[count]=z0;_pMaxX[count]=x1;_pMaxY[count]=y1;_pMaxZ[count]=z1;_pRod[count]=r;_pSeg[count]=s;count++;
                lowX=Math.Min(lowX,x0);lowY=Math.Min(lowY,y0);lowZ=Math.Min(lowZ,z0);highX=Math.Max(highX,x1);highY=Math.Max(highY,y1);highZ=Math.Max(highZ,z1);
                width+=Math.Max(x1-x0,Math.Max(y1-y0,z1-z0));
            }
        }
        if(count>1)
        {
            double cell=Math.Max(2*width/count,1e-9);int nx,ny,nz;
            while(true)
            {
                nx=(int)Math.Min(1<<20,(highX-lowX)/cell+1);ny=(int)Math.Min(1<<20,(highY-lowY)/cell+1);nz=(int)Math.Min(1<<20,(highZ-lowZ)/cell+1);
                if((long)nx*ny*nz<=4L*count+64)break;
                cell*=1.26;
            }
            int cells=nx*ny*nz;
            if(_cellStart.Length<cells+1)_cellStart=new int[Math.Max(cells+1,_cellStart.Length*2)];
            Array.Clear(_cellStart,0,cells+1);
            int bigCount=0,entries=0;
            if(_bigBoxes.Length<count)_bigBoxes=new int[count];
            for(int o=0;o<count;o++)
            {
                int i0=(int)((_pMinX[o]-lowX)/cell),j0=(int)((_pMinY[o]-lowY)/cell),k0=(int)((_pMinZ[o]-lowZ)/cell);
                int i1=Math.Min(nx-1,(int)((_pMaxX[o]-lowX)/cell)),j1=Math.Min(ny-1,(int)((_pMaxY[o]-lowY)/cell)),k1=Math.Min(nz-1,(int)((_pMaxZ[o]-lowZ)/cell));
                _cellLow[3*o]=i0;_cellLow[3*o+1]=j0;_cellLow[3*o+2]=k0;_cellHigh[3*o]=i1;_cellHigh[3*o+1]=j1;_cellHigh[3*o+2]=k1;
                if((long)(i1-i0+1)*(j1-j0+1)*(k1-k0+1)>MaxBoxCells){_bigBoxes[bigCount++]=o;continue;}
                for(int k=k0;k<=k1;k++)for(int j=j0;j<=j1;j++)for(int i=i0;i<=i1;i++){_cellStart[i+nx*(j+ny*k)+1]++;entries++;}
            }
            for(int c=0;c<cells;c++)_cellStart[c+1]+=_cellStart[c];
            if(_cellItems.Length<entries)_cellItems=new int[Math.Max(entries,_cellItems.Length*2)];
            // Fill in box order: a cell's list is ascending, and _cellStart[c] doubles as the write cursor (restored afterwards by shifting).
            for(int o=0;o<count;o++)
            {
                int i0=_cellLow[3*o],j0=_cellLow[3*o+1],k0=_cellLow[3*o+2],i1=_cellHigh[3*o],j1=_cellHigh[3*o+1],k1=_cellHigh[3*o+2];
                if((long)(i1-i0+1)*(j1-j0+1)*(k1-k0+1)>MaxBoxCells)continue;
                for(int k=k0;k<=k1;k++)for(int j=j0;j<=j1;j++)for(int i=i0;i<=i1;i++)_cellItems[_cellStart[i+nx*(j+ny*k)]++]=o;
            }
            for(int c=cells;c>0;c--)_cellStart[c]=_cellStart[c-1];
            _cellStart[0]=0;
            for(int k=0;k<nz;k++)for(int j=0;j<ny;j++)for(int i=0;i<nx;i++)
            {
                int c=i+nx*(j+ny*k);int from=_cellStart[c],to=_cellStart[c+1];
                for(int p=from;p<to;p++)
                {
                    int oa=_cellItems[p];
                    for(int q=p+1;q<to;q++)
                    {
                        int ob=_cellItems[q];
                        // Reported by the lowest cell both boxes occupy, so each pair appears once.
                        if(Math.Max(_cellLow[3*oa],_cellLow[3*ob])!=i||Math.Max(_cellLow[3*oa+1],_cellLow[3*ob+1])!=j||Math.Max(_cellLow[3*oa+2],_cellLow[3*ob+2])!=k)continue;
                        AddPair(oa,ob,ref pairs);
                    }
                }
            }
            for(int b=0;b<bigCount;b++)
            {
                int oa=_bigBoxes[b];
                for(int ob=0;ob<count;ob++)
                {
                    if(ob==oa)continue;
                    if(_pMaxX[ob]<_pMinX[oa]||_pMinX[ob]>_pMaxX[oa]||_pMaxY[ob]<_pMinY[oa]||_pMinY[ob]>_pMaxY[oa]||_pMaxZ[ob]<_pMinZ[oa]||_pMinZ[ob]>_pMaxZ[oa])continue;
                    bool otherBig=(long)(_cellHigh[3*ob]-_cellLow[3*ob]+1)*(_cellHigh[3*ob+1]-_cellLow[3*ob+1]+1)*(_cellHigh[3*ob+2]-_cellLow[3*ob+2]+1)>MaxBoxCells;
                    if(otherBig&&ob<oa)continue;
                    AddPair(oa,ob,ref pairs);
                }
            }
        }
        Array.Sort(_pairKeys,0,pairs);
        if(_segmentPairs.Length<pairs)_segmentPairs=new SegmentPair[Math.Max(pairs,_segmentPairs.Length*2)];
        for(int i=0;i<pairs;i++)
        {
            ulong k=_pairKeys[i];_segmentPairs[i]=new SegmentPair{A=(int)(k>>42),SA=(int)((k>>29)&8191),B=(int)((k>>13)&65535),SB=(int)(k&8191)};
        }
        _pairCount=pairs;
        if(_pairHits.Length<pairs)_pairHits=new PairHit[Math.Max(pairs,_pairHits.Length*2)];
    }
    // Adds the pair of boxes oa and ob (known to overlap in the grid cell) when the rods' filters allow them to touch.
    private void AddPair(int oa,int ob,ref int pairs)
    {
        if(_pMinX[oa]>_pMaxX[ob]||_pMinX[ob]>_pMaxX[oa]||_pMinY[oa]>_pMaxY[ob]||_pMinY[ob]>_pMaxY[oa]||_pMinZ[oa]>_pMaxZ[ob]||_pMinZ[ob]>_pMaxZ[oa])return;
        int ra=_pRod[oa],sa=_pSeg[oa],rb=_pRod[ob],sb=_pSeg[ob];var da=Data[ra]!;var db=Data[rb]!;
        if(ra==rb){if(!da.SelfCollision||MaterialNeighbours(da,sa,sb))return;}
        else
        {
            if(!Host.Layers(da.Layer,db.Layer))return;
            if(da.Group==db.Group&&da.Group!=0){if(da.Group<0)return;}
            else if((da.Category&db.Mask)==0||(db.Category&da.Mask)==0)return;
        }
        if(pairs>=1048576)throw new InvalidOperationException("Rod candidate pair limit 1048576 exceeded.");
        if(pairs==_pairKeys.Length)Array.Resize(ref _pairKeys,pairs*2);
        bool swap=ra>rb||(ra==rb&&sa>sb);
        // 16 bits of rod slot and 13 of segment (a rod has at most 4096 segments) per end.
        _pairKeys[pairs++]=swap?((ulong)rb<<42)|((ulong)sb<<29)|((ulong)ra<<13)|(uint)sa:((ulong)ra<<42)|((ulong)sa<<29)|((ulong)rb<<13)|(uint)sb;
    }

    private void SumStops(int count)
    {
        for(int i=0;i<count;i++)_conservativeStops+=_taskStops[i];
    }
    private void EnsureTaskStops(int count)
    {
        if(_taskStops.Length<count)_taskStops=new int[Math.Max(count,_taskStops.Length*2)];
        Array.Clear(_taskStops,0,count);
    }

    // ---- world contacts

    private readonly struct WorldSweepTask(RodStore store):IParallelBody
    {
        public void Run(int task)=>store.SweepWorldRod(task);
    }
    private void WorldContacts(float h)
    {
        EnsureGather();
        EnsureTaskStops(Data.Length);
        Host.RodCore.Executor.For(Data.Length,new WorldSweepTask(this));
        SumStops(Data.Length);
        for(int r=0;r<Data.Length;r++)
        {
            var d=Data[r];var g=_gathers[r];if(d==null||g==null||!g.Active)continue;
            for(int i=0;i<g.HitCount;i++)
            {
                ref var hit=ref g.Hits[i];
                WorldWitness(r,d,hit.Segment,hit.KeySegment,hit.Shape,hit.Witness,hit.Fraction,h);
            }
        }
    }
    private static void AddHit(RodGather g,int segment,int keySegment,int shape,in RodWitness witness,float fraction)
    {
        if(g.HitCount==g.Hits.Length)Array.Resize(ref g.Hits,g.Hits.Length*2);
        g.Hits[g.HitCount++]=new WorldHit{Segment=segment,KeySegment=keySegment,Shape=shape,Witness=witness,Fraction=fraction};
    }
    private static double Distance3(Vector3D a,Vector3D b)=>RodGeometry.Length(a-b);
    // True when the certificate proves the sweep cannot reach the shape, either because the endpoints barely moved since the pass that made it,
    // or (for a rigid body no solve has pushed this step) because the gap at the end of the previous substep still exceeds this substep's motion.
    private bool Certified(ref Cert c,int body,Vector3D a0,Vector3D b0,Vector3D a1,Vector3D b1)
    {
        if(FreshSearch||c.Epoch!=_gatherEpoch)return false;
        int version=_bodyVersion[body];
        if(c.Substep==_substep)
            return c.Version==version&&c.Clear>0&&Math.Max(Distance3(a1,c.A),Distance3(b1,c.B))<=c.Clear*.5;
        if(c.Substep!=_substep-1||c.Version!=0||version!=0||!(c.EndGap>0))return false;
        double gap=c.EndGap-Math.Max(Distance3(a0,c.A),Distance3(b0,c.B));
        if(!(gap>0))return false;
        BodyMotion(body,out var shift,out double turn);
        double speed=Math.Max(RodGeometry.Length(a1-a0-shift),RodGeometry.Length(b1-b0-shift))+turn;
        if(!(gap*.8>speed))return false;
        float end=(float)(gap-speed);
        c.Clear=end;c.EndGap=end;c.A=a1;c.B=b1;c.Substep=_substep;c.Version=0;
        return true;
    }
    private void Certify(ref Cert c,int body,Vector3D a1,Vector3D b1,float clear,float endGap)
    {
        if(!(clear>1e-4f)){c=default;return;}
        c=new Cert{Clear=clear,EndGap=endGap,Version=_bodyVersion[body],Substep=_substep,Epoch=_gatherEpoch,A=a1,B=b1};
    }
    // Reads rod, rigid pose and cache state only; writes this rod's gather slot.
    private void SweepWorldRod(int r)
    {
        var d=Data[r];var g=_gathers[r];if(d==null||g==null||!g.Active)return;
        g.HitCount=0;int stops=0;var shapes=Host.RodCore.Shapes;float radius=d.Material.Radius,skin=Skin(d);int segments=d.Length.Length;
        for(int segment=0;segment<segments;segment++)
        {
            if(d.InverseMass[segment]+d.InverseMass[segment+1]==0)continue;
            var a0=d.Previous[segment];var b0=d.Previous[segment+1];var a1=d.PositionData[segment];var b1=d.PositionData[segment+1];
            for(int k=g.CandStart[segment];k<g.CandStart[segment+1];k++)
            {
                int s=g.Cand[k];
                if(!shapes.Handles.IsLive(s))continue;
                int body=shapes.Owner[s];if(!d.CollideWithAttachedBodies&&AttachedTo(d,body))continue;
                if(Certified(ref g.SegCert[k],body,a0,b0,a1,b1))continue;
                if(!Near(s,a0,b0,a1,b1,radius))continue;
                if(!SweepWorld(s,a0,b0,a1,b1,radius,skin,ref stops,out float clear,out float endGap,out var witness,out float fraction))
                {
                    Certify(ref g.SegCert[k],body,a1,b1,clear,endGap);
                    continue;
                }
                AddHit(g,segment,segment,s,witness,fraction);
            }
        }
        // End-cap sweeps complement the interior segment witness. A face can constrain
        // both ends even when GJK deterministically returns only one closest feature.
        for(int p=0;p<d.PositionData.Length;p++)
        {
            if(d.InverseMass[p]==0)continue;
            int segment=Math.Min(p,segments-1);float u=p==segments?1:0;
            var start=d.Previous[p];var end=d.PositionData[p];
            for(int k=g.CandStart[segment];k<g.CandStart[segment+1];k++)
            {
                int s=g.Cand[k];int slot=g.CapStart[p]+k-g.CandStart[segment];
                if(!shapes.Handles.IsLive(s))continue;
                int body=shapes.Owner[s];if(!d.CollideWithAttachedBodies&&AttachedTo(d,body))continue;
                if(Certified(ref g.CapCert[slot],body,start,start,end,end))continue;
                if(!Near(s,start,start,end,end,radius))continue;
                if(!SweepWorld(s,start,start,end,end,radius,skin,ref stops,out float clear,out float endGap,out var witness,out float fraction))
                {
                    Certify(ref g.CapCert[slot],body,end,end,clear,endGap);
                    continue;
                }
                witness.Fraction=u;AddHit(g,segment,-p-1,s,witness,fraction);
            }
        }
        _taskStops[r]=stops;
    }

    // ---- rod pairs

    private readonly struct PairSweepTask(RodStore store):IParallelBody
    {
        public void Run(int task)=>store.SweepPairBlock(task);
    }
    private static void PairBoxes(RodParticles d,int s,out double minX,out double minY,out double minZ,out double maxX,out double maxY,out double maxZ)
    {
        SegmentBounds(d,s,out var min,out var max);double radius=d.Material.Radius+Skin(d);
        minX=min.X-radius;minY=min.Y-radius;minZ=min.Z-radius;maxX=max.X+radius;maxY=max.Y+radius;maxZ=max.Z+radius;
    }
    private void SweepPairBlock(int block)
    {
        int end=Math.Min(_pairCount,(block+1)*PairBlock);int stops=0;
        for(int i=block*PairBlock;i<end;i++)
        {
            var pair=_segmentPairs[i];var a=Data[pair.A]!;var b=Data[pair.B]!;ref var hit=ref _pairHits[i];hit.Valid=false;
            PairBoxes(a,pair.SA,out double ax0,out double ay0,out double az0,out double ax1,out double ay1,out double az1);
            PairBoxes(b,pair.SB,out double bx0,out double by0,out double bz0,out double bx1,out double by1,out double bz1);
            if(ax0>bx1||bx0>ax1||ay0>by1||by0>ay1||az0>bz1||bz0>az1)continue;
            if(!SweepPair(a,pair.SA,b,pair.SB,ref stops,out float u,out float v,out var normal))continue;
            hit.Valid=true;hit.U=u;hit.V=v;hit.Normal=normal;
        }
        _taskStops[block]=stops;
    }
    private void PairContacts(float h)
    {
        EnsureGather();
        int blocks=(_pairCount+PairBlock-1)/PairBlock;
        EnsureTaskStops(blocks);
        Host.RodCore.Executor.For(blocks,new PairSweepTask(this));
        SumStops(blocks);
        for(int i=0;i<_pairCount;i++)
        {
            if(!_pairHits[i].Valid)continue;
            var pair=_segmentPairs[i];var a=Data[pair.A]!;var b=Data[pair.B]!;float u=_pairHits[i].U,v=_pairHits[i].V;var normal=_pairHits[i].Normal;
            var key=(pair.A,pair.SA,-pair.B-1,pair.SB);
            if(!_contactKeys.TryGetValue(key,out int index))
            {
                index=AddContact(key);_contacts[index]=new Contact{Rod=pair.A,Segment=pair.SA,Other=pair.B,OtherSegment=pair.SB,Shape=-1,Body=-1,Enabled=true,NormalLimit=float.MaxValue,Friction=(float)Math.Sqrt(a.Friction*b.Friction),Friction2=(float)Math.Sqrt(a.Friction*b.Friction),Restitution=Math.Max(a.Restitution,b.Restitution)};
            }
            ref var contact=ref _contacts[index];
            if(contact.Pass==0 && contact.Normal==default)ConfigurePair(ref contact,a,b,u,v,normal);
            contact.Pass=_contactPass;contact.U=u;contact.V=v;contact.Normal=normal;
            if(!contact.Enabled)continue;
            var point=a.PositionData[pair.SA]*(1-u)+a.PositionData[pair.SA+1]*u;
            var other=b.PositionData[pair.SB]*(1-v)+b.PositionData[pair.SB+1]*v;
            double separation=RodGeometry.Dot(point-other,RodGeometry.Double(normal))-a.Material.Radius-b.Material.Radius+contact.SeparationOffset;
            double wa=(1-u)*(1-u)*a.InverseMass[pair.SA]+u*u*a.InverseMass[pair.SA+1];
            double wb=(1-v)*(1-v)*b.InverseMass[pair.SB]+v*v*b.InverseMass[pair.SB+1];
            if(separation>=0||wa+wb<=0)continue;
            float lambda=Math.Min((float)(-separation/(wa+wb)),Math.Max(0,contact.NormalLimit*h-contact.NormalLambda));contact.NormalLambda+=lambda;var impulse=normal*lambda;
            Move(a,pair.SA,impulse*((1-u)*a.InverseMass[pair.SA]));Move(a,pair.SA+1,impulse*(u*a.InverseMass[pair.SA+1]));
            Move(b,pair.SB,-impulse*((1-v)*b.InverseMass[pair.SB]));Move(b,pair.SB+1,-impulse*(v*b.InverseMass[pair.SB+1]));
        }
    }
}
