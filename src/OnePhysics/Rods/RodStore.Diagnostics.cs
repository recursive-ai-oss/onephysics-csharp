using System;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    internal RodStepStatistics Statistics;
    private float _maximumSpeed,_maximumPenetration;
    private double _maximumStretch;
    private int _maximumContacts,_conservativeStops;
    private void Measure(int lastPass)
    {
        _maximumContacts=Math.Max(_maximumContacts,_contactCount);
        foreach(var d in Data)
        {
            if(d==null)continue;_maximumStretch=Math.Max(_maximumStretch,d.MaximumStretch);
            foreach(var v in d.VelocityData)_maximumSpeed=Math.Max(_maximumSpeed,(float)Math.Sqrt((double)v.X*v.X+(double)v.Y*v.Y+(double)v.Z*v.Z));
        }
        for(int i=0;i<_contactCount;i++)
        {
            ref var c=ref _contacts[i];if(c.Pass!=lastPass||!c.Enabled)continue;var d=Data[c.Rod]!;
            double penetration;
            if(c.Other>=0)
            {
                var other=Data[c.Other]!;RodGeometry.Closest(d.PositionData[c.Segment],d.PositionData[c.Segment+1],other.PositionData[c.OtherSegment],other.PositionData[c.OtherSegment+1],out _,out _,out var a,out var b);
                penetration=d.Material.Radius+other.Material.Radius-RodGeometry.Length(a-b);
            }
            else
            {
                var witness=SafeDistance(c.Shape,d.PositionData[c.Segment],d.PositionData[c.Segment+1],d.Material.Radius,_time1);penetration=-witness.Separation;
            }
            if(double.IsFinite(penetration))_maximumPenetration=Math.Max(_maximumPenetration,(float)penetration);
        }
    }
}
