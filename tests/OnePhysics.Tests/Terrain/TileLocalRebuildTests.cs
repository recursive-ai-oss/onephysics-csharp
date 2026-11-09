using System;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class TileLocalRebuildTests
{
    private static void AssertSame(TileGrid2D expected,TileGrid2D actual,string label)
    {
        Assert.True(expected.SegmentCount==actual.SegmentCount,label+": segment count "+expected.SegmentCount+" vs "+actual.SegmentCount);
        for(int i=0;i<expected.SegmentCount;i++)
        {
            Assert.True(TileGrid2D.SegmentComparer.Instance.Compare(expected.GetSegment(i),actual.GetSegment(i))==0,label+": segment "+i);
            Assert.Equal(expected.GetSegmentMaterialIndex(i),actual.GetSegmentMaterialIndex(i));
        }
    }

    [Theory]
    [InlineData(false,false,1)]
    [InlineData(false,true,2)]
    [InlineData(true,false,3)]
    public void LocalEditsEqualFreshConstruction(bool contour,bool materials,int seed)
    {
        const int W=26,H=18; var random=new Random(seed); var tiles=new TileKind2D[W*H]; int[] idx=materials?new int[W*H]:[];
        for(int i=0;i<idx.Length;i++) idx[i]=random.Next(3);
        var grid=new TileGrid2D(W,H,tiles,0.5f,idx,contour);
        int before=TileGrid2D.LocalFallbacks;
        for(int n=0;n<400;n++)
        {
            int w=1+random.Next(n%7==0?10:3),h=1+random.Next(n%5==0?8:3),x=random.Next(W-w+1),y=random.Next(H-h+1);
            var kind=contour?(random.Next(3)==0?TileKind2D.Empty:TileKind2D.Solid):(TileKind2D)random.Next(5);
            if(random.Next(4)==0) kind=TileKind2D.Empty;
            var mask=new bool[w*h]; for(int i=0;i<mask.Length;i++) mask[i]=random.Next(3)!=0;
            grid=grid.WithCells(x,y,w,h,mask,kind);
            for(int j=0;j<h;j++) for(int i=0;i<w;i++) if(mask[j*w+i]) tiles[(y+j)*W+x+i]=kind;
            var fresh=new TileGrid2D(W,H,tiles,0.5f,idx,contour);
            AssertSame(fresh,grid,"edit "+n);
        }
        Assert.Equal(before,TileGrid2D.LocalFallbacks);
    }

    [Fact]
    public void LargeGridLocalEditsEqualFreshConstruction()
    {
        const int W=300,H=64; var random=new Random(9); var tiles=new TileKind2D[W*H];
        for(int x=0;x<W;x++) { int top=20+random.Next(5); for(int y=0;y<top;y++) tiles[y*W+x]=TileKind2D.Solid; }
        foreach(bool contour in new[]{false,true})
        {
            var grid=new TileGrid2D(W,H,tiles,1f,default,contour); var t=(TileKind2D[])tiles.Clone();
            for(int n=0;n<60;n++)
            {
                int cx=random.Next(W),cy=10+random.Next(20),r=1+random.Next(6);
                int x0=Math.Max(0,cx-r),y0=Math.Max(0,cy-r),x1=Math.Min(W-1,cx+r),y1=Math.Min(H-1,cy+r);
                int w=x1-x0+1,h=y1-y0+1; var mask=new bool[w*h]; var kind=random.Next(3)==0?TileKind2D.Solid:TileKind2D.Empty;
                for(int j=0;j<h;j++) for(int i=0;i<w;i++) { int dx=x0+i-cx,dy=y0+j-cy; mask[j*w+i]=dx*dx+dy*dy<=r*r; if(mask[j*w+i]) t[(y0+j)*W+x0+i]=kind; }
                grid=grid.WithCells(x0,y0,w,h,mask,kind);
            }
            AssertSame(new TileGrid2D(W,H,t,1f,default,contour),grid,contour?"contour":"tile");
        }
    }
}
