using System;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics;

internal static class GridMaterials
{
    internal static ContactMaterial[] Convert(ReadOnlySpan<P2.Material2D> materials)
    {
        var table=new ContactMaterial[materials.Length];
        for(int i=0;i<table.Length;i++)
        {
            if(!materials[i].Surface.IsDefault)throw new ArgumentException("Indexed materials use shape-wide Surface properties.",nameof(materials));
            table[i]=new(materials[i].Friction,materials[i].Restitution,materials[i].RollingResistance);
        }
        return table;
    }
    internal static ContactMaterial[] Convert(ReadOnlySpan<P3.Material3D> materials)
    {
        var table=new ContactMaterial[materials.Length];
        for(int i=0;i<table.Length;i++)
        {
            if(!materials[i].Surface.IsDefault)throw new ArgumentException("Indexed materials use shape-wide Surface properties.",nameof(materials));
            table[i]=new(materials[i].Friction,materials[i].Restitution,materials[i].RollingResistance);
        }
        return table;
    }
    internal static void Check(ReadOnlySpan<int> indices,int count,int materialCount)
    {
        if(indices.Length!=count)throw new ArgumentException("Material indices must contain one entry per grid cell or height-field interval.",nameof(indices));
        for(int i=0;i<indices.Length;i++)if((uint)indices[i]>=(uint)materialCount)throw new ArgumentOutOfRangeException(nameof(indices),indices[i],"Material index at element "+i+" is outside the table.");
    }
}
