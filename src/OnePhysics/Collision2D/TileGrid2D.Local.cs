using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics;

public sealed partial class TileGrid2D
{
    // Local boundary rebuild. A changed cell can only change lattice edges inside the box V of lattice points its
    // polygon (or, for a contour grid, its four dual cells) can touch. Every vertex outside V keeps the same edges and
    // the same turn choices, so any old segment whose window (ghost, both ends, ghost) never visits V is still a
    // segment of the new boundary. Fragments of the new boundary are traced only around V, jumping over unchanged
    // straight runs by using the old segment list, and the segments that visit V are regenerated from them.
    private static readonly int[] StepX={1,1,0,-1,-1,-1,0,1}, StepY={0,1,1,1,0,-1,-1,-1};

    private sealed class LocalTraceException : Exception { }

    private readonly record struct Box(int X0,int Y0,int X1,int Y1)
    {
        public bool Contains(Point p) => p.X>=X0 && p.X<=X1 && p.Y>=Y0 && p.Y<=Y1;
    }

    /// <summary>Smallest and largest t in [0,n] with p + t*(sx,sy) inside the box; false when none.</summary>
    private static bool Clip(Point p,int sx,int sy,int n,in Box v,out int t0,out int t1)
    {
        t0=0; t1=n;
        if(!ClipAxis(p.X,sx,v.X0,v.X1,ref t0,ref t1) || !ClipAxis(p.Y,sy,v.Y0,v.Y1,ref t0,ref t1)) return false;
        return t0<=t1;
    }
    private static bool ClipAxis(int p,int s,int lo,int hi,ref int t0,ref int t1)
    {
        if(s==0) return p>=lo && p<=hi;
        int a,b; if(s>0) { a=lo-p; b=hi-p; } else { a=p-hi; b=p-lo; }
        if(a>t0) t0=a; if(b<t1) t1=b;
        return true;
    }
    private static bool Touches(Point g1,Point a,Point b,Point g2,in Box v) => PieceTouches(g1,a,v) || PieceTouches(a,b,v) || PieceTouches(b,g2,v);
    private static bool PieceTouches(Point p,Point q,in Box v)
    {
        int n=Math.Max(Math.Abs(q.X-p.X),Math.Abs(q.Y-p.Y));
        if(n==0) return v.Contains(p);
        return Clip(p,Math.Sign(q.X-p.X),Math.Sign(q.Y-p.Y),n,v,out _,out _);
    }

    private Seg[] BuildLocal(Seg[] old,int x0,int y0,int x1,int y1)
    {
        var box=IsContour ? new Box(2*x0-1,2*y0-1,2*x1+3,2*y1+3) : new Box(2*x0,2*y0,2*x1+2,2*y1+2);
        try { return new Tracer(this,old,box).Run(); }
        catch(LocalTraceException) { return BuildGlobal(); }
    }

    // Net directed unit edges at v: net[d] > 0 means an exposed edge leaves v in direction d, net[d] < 0 an exposed edge arrives from v + step(d).
    private void Net(Point v,Edge[] buf,int[] net)
    {
        Array.Clear(net);
        int xlo,xhi,ylo,yhi;
        if(IsContour) { xlo=v.X>>1; xhi=(v.X+1)>>1; ylo=v.Y>>1; yhi=(v.Y+1)>>1; }
        else { xlo=(v.X-1)>>1; xhi=v.X>>1; ylo=(v.Y-1)>>1; yhi=v.Y>>1; }
        for(int cy=ylo;cy<=yhi;cy++) for(int cx=xlo;cx<=xhi;cx++)
        {
            if(cx<0 || cy<0 || cx>=CellCountX || cy>=CellCountY) continue;
            int n=CellEdges(cx,cy,buf);
            for(int i=0;i<n;i++)
            {
                var e=buf[i];
                if(e.A==v) net[Direction(e)]++; else if(e.B==v) net[Direction(new Edge(e.B,e.A))]--;
            }
        }
    }

    // Which outgoing edge follows an arriving one at a vertex. A vertex has one outgoing edge per arriving edge; where
    // several solids touch at a corner the arriving edge takes the least counter-clockwise turn, which keeps
    // corner-touching solids separate. If two arriving edges would pick the same outgoing edge, every arriving edge
    // instead follows its own solid wedge (the first outgoing edge clockwise from the arriving edge's ray), so the
    // answer depends only on the edges at this vertex and never on the order loops are traced in.
    private static int ChooseOut(int[] net,int dirIn)
    {
        int mine=LeastTurn(net,dirIn); if(mine<0) throw new LocalTraceException();
        int picked=0;
        for(int d=0;d<8;d++) if(net[d]<0) { int other=LeastTurn(net,(d+4)%8); if(other<0) throw new LocalTraceException(); if(other!=-1) picked|=1<<other; }
        int ins=0; for(int d=0;d<8;d++) if(net[d]<0) ins++;
        if(System.Numerics.BitOperations.PopCount((uint)picked)==ins) return mine;
        int ray=(dirIn+4)%8,best=9,pick=-1;
        for(int d=0;d<8;d++) if(net[d]>0) { int w=(ray-d+8)%8; if(w==0) w=8; if(w<best) { best=w; pick=d; } }
        if(pick<0) throw new LocalTraceException();
        return pick;
    }
    private static int LeastTurn(int[] net,int dirIn)
    {
        int best=9,pick=-1;
        for(int d=0;d<8;d++) if(net[d]>0) { int turn=(d-dirIn+8)%8; if(turn<best) { best=turn; pick=d; } }
        return pick;
    }

    private sealed class Tracer
    {
        private readonly TileGrid2D _g; private readonly Seg[] _old; private readonly Box _v;
        private readonly Edge[] _buf=new Edge[16]; private readonly int[] _net=new int[8];
        private readonly HashSet<Edge> _covered=new();
        private readonly List<Seg> _emitted=new();
        internal Tracer(TileGrid2D g,Seg[] old,Box v) { _g=g; _old=old; _v=v; }

        internal Seg[] Run()
        {
            var edges=new List<Edge>();
            int xMin=Math.Max(_v.X0,0), xMax=Math.Min(_v.X1,2*_g.Width), yMin=Math.Max(_v.Y0,0), yMax=Math.Min(_v.Y1,2*_g.Height);
            for(int y=yMin;y<=yMax;y++) for(int x=xMin;x<=xMax;x++)
            {
                var p=new Point(x,y); Net(p);
                for(int d=0;d<8;d++)
                {
                    var q=new Point(x+StepX[d],y+StepY[d]);
                    if(_net[d]>0) edges.Add(new Edge(p,q)); else if(_net[d]<0) edges.Add(new Edge(q,p));
                }
            }
            edges.Sort();
            for(int i=0;i<edges.Count;i++)
            {
                if(i>0 && edges[i].Equals(edges[i-1])) continue;
                if(_covered.Contains(edges[i])) continue;
                Fragment(edges[i]);
            }
            var retained=new List<Seg>(_old.Length);
            foreach(var s in _old) if(!Touches(s.G1,s.A,s.B,s.G2,_v)) retained.Add(s);
            _emitted.Sort(static(a,b)=>SegmentComparer.Instance.Compare(a.Shape,b.Shape));
            var result=new List<Seg>(retained.Count+_emitted.Count);
            int r=0,e=0;
            while(r<retained.Count || e<_emitted.Count)
            {
                int c=r==retained.Count?1:e==_emitted.Count?-1:SegmentComparer.Instance.Compare(retained[r].Shape,_emitted[e].Shape);
                if(c<0) result.Add(retained[r++]);
                else if(c>0) { if(result.Count==0 || SegmentComparer.Instance.Compare(result[^1].Shape,_emitted[e].Shape)!=0) result.Add(_emitted[e]); e++; }
                else { result.Add(retained[r++]); e++; }
            }
            return result.ToArray();
        }

        private void Net(Point v) => _g.Net(v,_buf,_net);
        private Edge Next(Edge cur)
        {
            Net(cur.B); int pick=ChooseOut(_net,Direction(cur));
            return new Edge(cur.B,new Point(cur.B.X+StepX[pick],cur.B.Y+StepY[pick]));
        }
        private Edge Prev(Edge cur)
        {
            Net(cur.A); Edge found=default; int count=0;
            var ins=new List<Edge>(2);
            for(int d=0;d<8;d++) if(_net[d]<0) ins.Add(new Edge(new Point(cur.A.X+StepX[d],cur.A.Y+StepY[d]),cur.A));
            foreach(var candidate in ins) if(Next(candidate).Equals(cur)) { found=candidate; count++; }
            if(count!=1) throw new LocalTraceException();
            return found;
        }
        private bool IsCorner(Edge into,Edge outOf) => Direction(into)!=Direction(outOf) || _g.Material(into)!=_g.Material(outOf);

        // Walks from start along (or against) the boundary until four corners outside V have passed since the last
        // vertex in V. Returns true when the walk comes back to start (a closed loop); points are appended in walk order.
        private bool Walk(Edge start,bool fwd,List<Point> pts,bool unlimited)
        {
            Edge cur=start; int count=0;
            while(true)
            {
                Point j=fwd?cur.B:cur.A;
                Edge nxt=fwd?Next(cur):Prev(cur);
                if(nxt.Equals(start)) return true;
                if(_v.Contains(j)) count=0; else if(IsCorner(fwd?cur:nxt,fwd?nxt:cur)) count++;
                if(_v.Contains(nxt.A) || _v.Contains(nxt.B)) _covered.Add(nxt);
                Point far=fwd?nxt.B:nxt.A;
                if(pts.Count>8*(_g.Width+1)*(_g.Height+1)+64) throw new LocalTraceException();
                pts.Add(far); cur=nxt;
                if(count>=4 && !unlimited) return false;
                if(_v.Contains(far)) continue;
                // Jump over an unchanged straight run outside V: the old segment list knows where it ends.
                int d=Direction(cur); int sx=StepX[d], sy=StepY[d];
                int ux=fwd?sx:-sx, uy=fwd?sy:-sy;
                var next=new Point(far.X+ux,far.Y+uy);
                if(_v.Contains(next)) continue;
                // At a vertex where solids touch the boundary may turn even though a straight edge exists, so ask the pairing rule.
                bool straight=Direction(fwd?Next(cur):Prev(cur))==d;
                if(!straight) continue;
                int room=PieceRoom(far,sx,sy,fwd);
                if(room<2) continue;
                int m=room;
                if(Clip(far,ux,uy,room,_v,out int t0,out _)) m=Math.Min(room,t0-1);
                if(m<2) continue;
                var land=new Point(far.X+ux*m,far.Y+uy*m);
                pts.Add(land);
                cur=fwd?new Edge(new Point(land.X-sx,land.Y-sy),land):new Edge(land,new Point(land.X+sx,land.Y+sy));
            }
        }
        // Lattice steps from p, forward or backward along direction (sx,sy), that stay on one old boundary piece.
        private int PieceRoom(Point p,int sx,int sy,bool fwd)
        {
            foreach(var s in _old)
            {
                int n=Math.Max(Math.Abs(s.B.X-s.A.X),Math.Abs(s.B.Y-s.A.Y)); if(n==0) continue;
                if(Math.Sign(s.B.X-s.A.X)!=sx || Math.Sign(s.B.Y-s.A.Y)!=sy) continue;
                int t=sx!=0?(p.X-s.A.X)*sx:(p.Y-s.A.Y)*sy;
                if(t<0 || t>n || p.X!=s.A.X+sx*t || p.Y!=s.A.Y+sy*t) continue;
                if(fwd) { if(t<n) return n-t; }
                else if(t>0) return t;
            }
            throw new LocalTraceException();
        }

        private void Fragment(Edge e)
        {
            _covered.Add(e);
            var fwd=new List<Point>(); var back=new List<Point>();
            bool closed=Walk(e,true,fwd,false);
            if(!closed && Walk(e,false,back,false))
            {
                // Both ends reach the start: the whole loop is within reach, so walk it completely.
                fwd.Clear(); back.Clear();
                if(!Walk(e,true,fwd,true)) throw new LocalTraceException();
                closed=true;
            }
            var path=new List<Point>(back.Count+fwd.Count+2);
            if(!closed) for(int i=back.Count-1;i>=0;i--) path.Add(back[i]);
            path.Add(e.A); path.Add(e.B); path.AddRange(fwd);
            if(closed) { if(path[^1]!=path[0]) throw new LocalTraceException(); path.RemoveAt(path.Count-1); }
            // Corners: direction or material changes at a vertex between two consecutive path points.
            int n=path.Count; var corners=new List<Point>();
            for(int t=closed?0:1;t<(closed?n:n-1);t++)
            {
                Point a=path[(t+n-1)%n], b=path[t], c=path[(t+1)%n];
                int dx1=Math.Sign(b.X-a.X),dy1=Math.Sign(b.Y-a.Y),dx2=Math.Sign(c.X-b.X),dy2=Math.Sign(c.Y-b.Y);
                var into=new Edge(new Point(b.X-dx1,b.Y-dy1),b); var outOf=new Edge(b,new Point(b.X+dx2,b.Y+dy2));
                if(dx1!=dx2 || dy1!=dy2 || _g.Material(into)!=_g.Material(outOf)) corners.Add(b);
            }
            int m=corners.Count;
            if(closed)
            {
                for(int i=0;i<m;i++) Emit(corners[(i+m-1)%m],corners[i],corners[(i+1)%m],corners[(i+2)%m]);
            }
            else for(int i=1;i+2<m;i++) Emit(corners[i-1],corners[i],corners[i+1],corners[i+2]);
        }
        private void Emit(Point g1,Point a,Point b,Point g2)
        {
            if(Touches(g1,a,b,g2,_v)) _emitted.Add(_g.MakeSegment(g1,a,b,g2));
        }
    }
}
