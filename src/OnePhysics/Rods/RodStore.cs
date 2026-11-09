using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Rods;

internal interface IRodHost
{
    WorldCore RodCore { get; }
    int EngineSectionId { get; }
    RodStore Rods { get; }
    bool Planar { get; }
    Vector3F Gravity { get; }
    Vector3F LocalCenter(int body);
    float BodyExtent(int body);
    void MovedBody(int body);
    bool Matches(RodParticles rod,int shape);
    bool Layers(int a,int b);
    void Bounds(int shape,out Vector3D minimum,out Vector3D maximum);
    // Appends the user ids (shape slots) of proxies whose stored box overlaps the box: the static tree, or the kinematic and dynamic trees.
    void CollectShapes(Vector3D minimum,Vector3D maximum,bool moving,System.Collections.Generic.List<int> output);
    RodWitness Distance(int shape,Vector3D a,Vector3D b,float radius,float time);
    ContactMaterial Mix(int shape,int subshape,float friction,float restitution);

    ReplayMutationScope RodMutation(string operation);
}

internal sealed partial class RodStore(IRodHost host) : ISnapshotSection
{
    internal readonly IRodHost Host = host;
    internal RodParticles?[] Data = [];
    internal int[] Generations = [];
    internal int Count;
    public int SectionId => 0x524F4453;
    public bool IsOptional => true;
    public bool IsPresent => Data.Length != 0;
    internal bool Alive(int slot, int generation) => (uint)slot < (uint)Data.Length && Data[slot] != null && Generations[slot] == generation;
    internal (int Slot, int Generation) Add(RodParticles data)
    {
        if (Data.Length >= 65536 && Count == Data.Length) throw new InvalidOperationException("Rod slot limit 65536 reached.");
        int particles = data.PositionData.Length;
        foreach (var d in Data) if (d != null) particles = checked(particles+d.PositionData.Length);
        if (particles > 1048576) throw new InvalidOperationException("Rod particle limit 1048576 reached.");
        using var mutation = Host.RodMutation("World.AddRod");
        int i = 0; while (i < Data.Length && Data[i] != null) i++;
        if (i == Data.Length) { Array.Resize(ref Data,i+1); Array.Resize(ref Generations,i+1); Generations[i] = 1; }
        Data[i] = data; Count++; return (i,Generations[i]);
    }
    internal void Remove(int i)
    {
        if (Generations[i] == int.MaxValue) throw new InvalidOperationException("Rod generation exhausted.");
        using var mutation = Host.RodMutation("Rod.Destroy"); Data[i] = null; Generations[i]++; Count--;
    }
    internal void CheckBounds()
    {
        for (int rod = 0; rod < Data.Length; rod++)
        {
            var d = Data[rod]; if (d == null) continue;
            for (int i = 0; i < d.PositionData.Length; i++)
            {
                var p = d.PositionData[i]; var v = d.VelocityData[i];
                if (!InBounds(p) || !Finite(v) || !InBounds(d.PinTarget[i]) || (Host.Planar && (p.Z != 0 || v.Z != 0 || d.PinTarget[i].Z != 0)))
                    throw new RodStateException(rod,i,"particle",$"Rod {rod}, particle {i}: position and target must be within ±4194304 metres, velocity finite, and planar Z zero.");
            }
            for (int i = 0; i < d.FrameData.Length; i++)
            {
                var q = d.FrameData[i]; var v = d.AngularVelocity[i];
                double norm = (double)q.X*q.X+(double)q.Y*q.Y+(double)q.Z*q.Z+(double)q.W*q.W;
                if (!(norm >= 0.999 && norm <= 1.001) || !Finite(v) || (Host.Planar && (q.X != 0 || q.Y != 0 || v.X != 0 || v.Y != 0)))
                    throw new RodStateException(rod,i,"frame",$"Rod {rod}, segment {i}: frame must be unit length, angular velocity finite, and planar rotations about Z only.");
            }
        }
    }
    private static bool InBounds(Vector3D p) => Math.Abs(p.X) <= 4194304 && Math.Abs(p.Y) <= 4194304 && Math.Abs(p.Z) <= 4194304;
    internal void CheckShift(Vector3D offset)
    {
        for (int rod = 0; rod < Data.Length; rod++)
        {
            if (Data[rod] is not {} d) continue;
            for (int i = 0; i < d.PositionData.Length; i++)
                if (!InBounds(d.PositionData[i]-offset) || !InBounds(d.PinTarget[i]-offset))
                    throw new ArgumentOutOfRangeException(nameof(offset),offset,$"Shift would move rod {rod}, particle {i} or its target outside ±4194304 metres.");
        }
    }
    internal void ValidateStep(float dt)
    {
        foreach (var d in Data)
        {
            if (d == null) continue;
            float h = dt/d.Substeps;
            if (!(h > 0)) throw new ArgumentOutOfRangeException(nameof(dt),dt,"Rod substep duration underflows float32.");
            for (int i = 0; i < d.PositionData.Length; i++)
            {
                if (d.Pinned[i]) continue;
                var v = (d.VelocityData[i]+Host.Gravity*(d.GravityScale*h))/(1+d.Damping*h);
                var p = d.PositionData[i]+new Vector3D((double)v.X*h,(double)v.Y*h,(double)v.Z*h);
                if (!Finite(v) || !InBounds(p)) throw new ArgumentOutOfRangeException(nameof(dt),dt,$"Rod particle {i}'s predicted motion is non-finite or outside ±4194304 metres.");
            }
        }
    }
    internal void Step(float dt)
    {
        _stepDt=dt;_maximumSpeed=0;_maximumPenetration=0;_maximumStretch=0;_maximumContacts=0;_conservativeStops=0;
        int substeps=1,iterations=1;
        foreach(var d in Data)if(d!=null)
        {
            substeps=Math.Max(substeps,d.Substeps);iterations=Math.Max(iterations,d.Iterations);
            Array.Copy(d.PositionData,d.PinStart,d.PositionData.Length);Array.Clear(d.AttachmentForce);
        }
        for(int b=0;b<Host.RodCore.Bodies.HighWater;b++)
        {
            if(!Host.RodCore.Bodies.IsLive(b))continue;
            _bodyEnd[b]=BodyPosition(b);_rotationEnd[b]=BodyRotation(b);_bodyDelta[b]=default;_rotationDelta[b]=QuaternionF.Identity;_bodyChanged[b]=false;_bodyVelocityDelta[b]=default;_bodyAngularDelta[b]=default;
        }
        float h=dt/substeps;
        for(int step=0;step<substeps;step++)
        {
            float t=(float)(step+1)/substeps;
            _time0=(float)step/substeps;_time1=t;
            for(int b=0;b<Host.RodCore.Bodies.HighWater;b++)if(Host.RodCore.Bodies.IsLive(b))BodyPose(b,_time0,out _bodySubStart[b],out _rotationSubStart[b]);
            _contactKeys.Clear();_contactCount=0;PrepareGather(step);
            if(step>0)
                for(int b=0;b<Host.RodCore.Bodies.HighWater;b++)
                {
                    if(!Host.RodCore.Bodies.IsLive(b))continue;
                    var dv=_bodyVelocityDelta[b];_bodyDelta[b]+=new Vector3D((double)dv.X*h,(double)dv.Y*h,(double)dv.Z*h);
                    _rotationDelta[b]=RodParticles.Turn(_rotationDelta[b],_bodyAngularDelta[b]*h);
                }
            Host.RodCore.Executor.For(Data.Length,new RodTask(this,h,t,-1));
            for(int pass=0;pass<iterations;pass++)
            {
                Host.RodCore.Executor.For(Data.Length,new RodTask(this,h,t,pass));
                Attachments(h,t);
                _contactPass=pass;
                WorldContacts(h);
                PairContacts(h);
            }
            Host.RodCore.Executor.For(Data.Length,new RodTask(this,h,t,-2));
            ContactVelocities(h,iterations-1);Measure(iterations-1);
        }
        FinishBodies();Statistics=new(_maximumSpeed,_maximumStretch,_maximumPenetration,_maximumContacts,_conservativeStops);
    }
    private readonly struct RodTask(RodStore store,float h,float time,int phase):IParallelBody
    {
        public void Run(int task)
        {
            var d=store.Data[task];if(d==null)return;
            if(phase==-1)d.Predict(h,store.Host.Gravity,time);
            else if(phase==-2)d.Finish(h);
            else if(phase<d.Iterations)d.Project(h);
        }
    }
    internal void Shift(Vector3D offset)
    {
        foreach (var d in Data)
        {
            if (d == null) continue;
            for (int i = 0; i < d.PositionData.Length; i++) { d.PositionData[i] -= offset; d.Previous[i] -= offset; d.PinTarget[i] -= offset; d.PinStart[i] -= offset; }
        }
    }
    internal void SelectRegion(RegionSelection selection, double[] min, double[] max, RegionSnapshot? archive)
    {
        if (archive != null) return;
        for (int i = 0; i < Data.Length; i++)
        {
            if (Data[i] is not {} d) continue;
            foreach (var p in d.PositionData)
                if (p.X >= min[0] && p.X <= max[0] && p.Y >= min[1] && p.Y <= max[1] && (Host.Planar || (p.Z >= min[2] && p.Z <= max[2])))
                { selection.Add("rod",i); break; }
        }
    }
    internal bool CloseRegion(RegionSelection selection)
    {
        bool changed=false;
        for(int r=0;r<Data.Length;r++)
        {
            if(Data[r] is not {} d)continue;
            for(int p=0;p<d.AttachmentBody.Length;p++)
            {
                int b=d.AttachmentBody[p];if(b<0||!Host.RodCore.Bodies.Handles.IsAlive(b,d.AttachmentGeneration[p]))continue;
                if(selection.Contains("rod",r))changed|=selection.AddBody(b);
                if(selection.Bodies[b]&&Host.RodCore.Bodies.IsDynamic(b))changed|=selection.Add("rod",r);
            }
        }
        return changed;
    }
    public void Reset() { Data = []; Generations = []; Count = 0; Statistics=default; }
    public ISnapshotSection Fork(WorldCore destination)
    {
        var copy = ((IRodHost)destination.GetSnapshotSection(Host.EngineSectionId)).Rods;
        copy.Data = new RodParticles?[Data.Length]; copy.Generations = (int[])Generations.Clone(); copy.Count = Count;copy.Statistics=Statistics;
        for (int i = 0; i < Data.Length; i++) copy.Data[i] = Data[i]?.Copy();
        return copy;
    }
    private static bool Finite(Vector3F v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
}
