using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Fields;

internal interface IFieldHost
{
    WorldCore FieldCore { get; }
    int EngineSectionId { get; }
    Vector3F AngularTorque(int body,Vector3F acceleration);
    FieldStore Fields { get; }
    bool Matches(int body, FieldDefinition definition);
    void AddForce(int body, Vector3F force, Vector3F torque);
    Vector3F Gravity { get; }
    float GravityScaleOf(int body);
    void Buoyancy(int body, FieldDefinition definition, float dt);
    ReplayMutationScope FieldMutation(string operation);
}

/// <summary>A generation-checked handle to a field. Definitions are copied on read and write. A destroyed handle throws InvalidOperationException; default handles are invalid.</summary>
public readonly struct Field
{
    private readonly FieldStore? _store;
    private readonly int _generation;
    internal Field(FieldStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }
    /// <summary>Stable slot, reused lowest-first after destruction.</summary>
    public int Index { get; }
    /// <summary>True while this slot and generation belong to a live field.</summary>
    public bool IsValid => _store != null && _store.Alive(Index, _generation);
    private FieldStore Resolve() => IsValid ? _store! : throw new InvalidOperationException("The field has been destroyed or is uninitialized.");
    /// <summary>Copied definition. Setting validates before replacement and is recorded by the world.</summary>
    public FieldDefinition Definition
    {
        get => Resolve().Definitions[Index]!.Clone();
        set { var s = Resolve(); ArgumentNullException.ThrowIfNull(value); value.ValidateStored(); FieldStore.ValidateReplacementCoordinates(value,s.Definitions[Index]!); s.ValidateDimension(value); var copy=value.Clone(); s.CaptureSensor(copy); using var mutation = s.Host.FieldMutation("Field.Definition"); s.Definitions[Index] = copy; }
    }
    /// <summary>Destroys this field; later access throws InvalidOperationException.</summary>
    public void Destroy() { var s = Resolve(); if(s.Generations[Index]==int.MaxValue)throw new InvalidOperationException("Field generation is exhausted."); using var mutation = s.Host.FieldMutation("Field.Destroy"); s.Definitions[Index] = null; s.Generations[Index] = checked(s.Generations[Index]+1); s.Count--; }
}

internal sealed partial class FieldStore(IFieldHost host) : ISnapshotSection
{
    internal readonly IFieldHost Host = host;
    internal FieldDefinition?[] Definitions = [];
    internal int[] Generations = [];
    internal int Count;
    private readonly Dictionary<int, GravityField> _callbacks = new();
    private Vector3F[] _zone = [];
    private bool[] _zoneActive = [];
    public int SectionId => 0x464C4453;
    public bool IsOptional => true;
    public bool IsPresent => Definitions.Length != 0 || _moments.Length != 0;
    internal bool Alive(int i, int generation) => (uint)i < (uint)Definitions.Length && Definitions[i] != null && Generations[i] == generation;
    internal Field Add(FieldDefinition definition)
    {
        ArgumentNullException.ThrowIfNull(definition); definition.Validate(); ValidateDimension(definition);
        var copy=definition.Clone();CaptureSensor(copy);
        using var mutation = Host.FieldMutation("World.AddField");
        int i = 0; while (i < Definitions.Length && Definitions[i] != null) i++;
        if (i == Definitions.Length) { Array.Resize(ref Definitions, i+1); Array.Resize(ref Generations, i+1); Generations[i] = 1; }
        Definitions[i] = copy; Count++;
        return new Field(this, i, Generations[i]);
    }
    internal void CaptureSensor(FieldDefinition definition)
    {
        if(definition.Sensor>=0 && definition.SensorGeneration==0)
        {
            var shapes=Host.FieldCore.Shapes;
            if(!shapes.Handles.IsLive(definition.Sensor))throw new ArgumentOutOfRangeException(nameof(definition),"Sensor slot is not live.");
            definition.SensorGeneration=shapes.Handles.GenerationOf(definition.Sensor);
        }
    }
    internal static void ValidateReplacementCoordinates(FieldDefinition value,FieldDefinition previous)
    {
        // An origin shift can move existing bounds into simulation headroom. Keeping
        // those coordinates unchanged is valid; newly authored coordinates retain the creation bound.
        CheckReplacementPoint(value.Center,previous.Center,nameof(value.Center));
        CheckReplacementPoint(value.Minimum,previous.Minimum,nameof(value.Minimum));
        CheckReplacementPoint(value.Maximum,previous.Maximum,nameof(value.Maximum));
        if(value.SurfaceHeight!=previous.SurfaceHeight&&Math.Abs(value.SurfaceHeight)>2097152)
            throw new ArgumentOutOfRangeException(nameof(value),"New field SurfaceHeight must be within ±2097152 metres.");
    }
    private static void CheckReplacementPoint(Vector3D value,Vector3D previous,string name)
    {
        if((value.X!=previous.X&&Math.Abs(value.X)>2097152)||(value.Y!=previous.Y&&Math.Abs(value.Y)>2097152)||(value.Z!=previous.Z&&Math.Abs(value.Z)>2097152))
            throw new ArgumentOutOfRangeException(name,value,"New field coordinates must be within ±2097152 metres.");
    }
    internal void ValidateDimension(FieldDefinition definition)
    {
        if (Host.FieldCore.Bodies.Layout.PositionLanes==2 && definition.Kind==FieldKind.Buoyancy && definition.Axis.Z!=0)
            throw new ArgumentOutOfRangeException(nameof(definition),"Planar buoyancy normal must lie in XY.");
    }
    internal void CheckShift(Vector3D offset)
    {
        foreach(var definition in Definitions)
        {
            if(definition==null)continue;
            FieldDefinition.CheckPoint(definition.Center-offset,"shifted field center",4194304);
            FieldDefinition.CheckPoint(definition.Minimum-offset,"shifted field minimum",4194304);
            FieldDefinition.CheckPoint(definition.Maximum-offset,"shifted field maximum",4194304);
            double height=definition.SurfaceHeight-definition.Axis.X*offset.X-definition.Axis.Y*offset.Y-definition.Axis.Z*offset.Z;
            if(!double.IsFinite(height)||Math.Abs(height)>4194304)throw new ArgumentOutOfRangeException(nameof(offset),"Shifted field surface exceeds ±4194304.");
        }
    }
    internal void Bind(int id, GravityField? callback)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(id);
        if (callback == null) _callbacks.Remove(id); else _callbacks[id] = callback;
    }
    internal static Vector3F Cross(Vector3F a, Vector3F b) => new(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X);
    internal static float Dot(Vector3F a, Vector3F b) => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
    internal static Vector3F PositionDifference(Vector3D a, Vector3D b) => new((float)(a.X-b.X),(float)(a.Y-b.Y),(float)(a.Z-b.Z));
    internal Vector3D Position(int b)
    {
        var p = Host.FieldCore.Bodies.PositionOf(b);
        return new(p[0],p[1],p.Length == 3 ? p[2] : 0);
    }
    internal bool Matches(int b, FieldDefinition d)
    {
        var p = Position(b);
        return p.X >= d.Minimum.X && p.X <= d.Maximum.X && p.Y >= d.Minimum.Y && p.Y <= d.Maximum.Y && p.Z >= d.Minimum.Z && p.Z <= d.Maximum.Z && Host.Matches(b,d);
    }
    internal static float Falloff(FieldDefinition d, float distance)
    {
        float r = Math.Max(distance,d.Softening);
        return d.Falloff == FieldFalloff.Constant ? d.Strength : d.Falloff == FieldFalloff.InverseLinear ? d.Strength/r : d.Strength/(r*r);
    }
    internal void Step(float dt)
    {
        if (Count == 0) return;
        Skipped = 0;
        var core = Host.FieldCore; var bodies = core.Bodies;
        // Resolve gravity zones without changing the order of force accumulation.
        if (_zone.Length < bodies.HighWater) { Array.Resize(ref _zone, bodies.HighWater); Array.Resize(ref _zoneActive,bodies.HighWater); }
        Array.Clear(_zoneActive);
        for (int b=0;b<bodies.HighWater;b++)
        {
            if (!bodies.IsLive(b)||!bodies.IsDynamic(b)) continue;
            int priority = int.MinValue; double weight = 0, x = 0, y = 0, z = 0;
            for (int f=0;f<Definitions.Length;f++)
            {
                var d = Definitions[f]; if (d == null || d.Kind != FieldKind.RegionalGravity || !Matches(b,d) || d.Priority < priority) continue;
                if (d.Priority > priority) { priority=d.Priority; weight=0; x=0;y=0;z=0; }
                weight+=d.Weight; x+=(double)d.Weight*d.Direction.X; y+=(double)d.Weight*d.Direction.Y; z+=(double)d.Weight*d.Direction.Z;
            }
            if (weight>0) { _zone[b] = new((float)(x/weight),(float)(y/weight),(float)(z/weight)); _zoneActive[b]=true; }
        }
        bool zonesApplied = false;
        for (int f=0;f<Definitions.Length;f++)
        {
            var d=Definitions[f]; if (d==null) continue;
            if (d.Kind == FieldKind.Magnetism) { StepMagnets(d,dt); continue; }
            for (int b=0;b<bodies.HighWater;b++)
            {
                if (!bodies.IsLive(b)||!bodies.IsDynamic(b)) continue;
                if (d.Kind == FieldKind.RegionalGravity)
                {
                    if (!zonesApplied && _zoneActive[b]) Apply(b,(_zone[b]-Host.Gravity)*Host.GravityScaleOf(b),default,dt,true);
                    continue;
                }
                if (!Matches(b,d)) continue;
                if (d.Kind == FieldKind.Buoyancy) { Host.Buoyancy(b,d,dt); continue; }
                if (d.Kind is FieldKind.Platform or FieldKind.Surface) continue;
                Vector3F a = default;
                if (d.Kind == FieldKind.Area) a=d.Direction;
                else if (d.Kind == FieldKind.CustomGravity)
                {
                    if (!_callbacks.TryGetValue(d.CallbackId,out var callback)) throw new InvalidOperationException("A CustomGravity field has no host callback bound to its CallbackId.");
                    a=callback(Position(b),core.StepCount);
                    // A callback result that is not finite or is beyond the field limit is dropped, as in Apply.
                    if(!Usable(a)){Skipped++;continue;}
                    a=(a-Host.Gravity)*Host.GravityScaleOf(b);
                }
                else
                {
                    var r = PositionDifference(Position(b),d.Center);
                    if (d.Kind == FieldKind.Vortex || (d.Line && d.Kind == FieldKind.RadialGravity)) r-=d.Axis*Dot(r,d.Axis);
                    float r2=Dot(r,r); if (r2 > d.Radius*d.Radius) continue;
                    float length=float.Sqrt(r2); var n=length>0 ? r/length : default;
                    if (d.Kind == FieldKind.Vortex) a=Cross(d.Axis,n)*d.Strength-n*d.Inward;
                    else a=n*(d.Kind == FieldKind.RadialGravity ? -Falloff(d,length) : Falloff(d,length));
                }
                var v=bodies.LinearVelocityOf(b); var w=bodies.AngularVelocityOf(b);
                var velocity=new Vector3F(v[0],v[1],v.Length==3?v[2]:0);
                var omega=w.Length==1?new Vector3F(0,0,w[0]):new Vector3F(w[0],w[1],w[2]);
                bool acceleration=d.Acceleration||d.Kind is FieldKind.RadialGravity or FieldKind.CustomGravity or FieldKind.Vortex;
                // Convert force fields to acceleration before applying mass-proportional drag.
                if (!acceleration) a*=bodies.InverseMass[b];
                a+=(d.Flow-velocity)*(d.Drag/(1+dt*d.Drag));
                Apply(b,a,omega*(-d.AngularDrag/(1+dt*d.AngularDrag)),dt,true);
            }
            if (d.Kind == FieldKind.RegionalGravity) zonesApplied=true;
        }
    }
    /// <summary>Applications dropped in the last field step because their result was non-finite or beyond ±1e12 (a callback that returned garbage, or a definition whose terms multiply past the limit).</summary>
    internal int Skipped;
    // Definitions are validated when they are created, but what a field produces at a body depends on distances, velocities, masses and host
    // callbacks. A result outside the limit is dropped before anything changes (no force, no wake), so the step carries on with the other
    // fields and bodies; the count says how often. Throwing here would leave a half-stepped world.
    internal static bool Usable(Vector3F v)
        => float.IsFinite(v.X)&&float.IsFinite(v.Y)&&float.IsFinite(v.Z)&&Math.Abs(v.X)<=1e12f&&Math.Abs(v.Y)<=1e12f&&Math.Abs(v.Z)<=1e12f;
    internal void Apply(int b, Vector3F value, Vector3F angularAcceleration, float dt, bool acceleration)
    {
        if(!Usable(value)||!Usable(angularAcceleration)){Skipped++;return;}
        var core=Host.FieldCore; var bodies=core.Bodies;
        var a=acceleration?value:value*bodies.InverseMass[b];
        var force=acceleration?value/bodies.InverseMass[b]:value;var torque=AngularTorque(b,angularAcceleration);
        if(!float.IsFinite(force.X+force.Y+force.Z)||!float.IsFinite(torque.X+torque.Y+torque.Z)||!float.IsFinite(a.X+a.Y+a.Z)){Skipped++;return;}
        if(!Wake(b,a,angularAcceleration,dt))return;
        Host.AddForce(b,force,torque);
    }
    internal bool Wake(int b,Vector3F acceleration,Vector3F angularAcceleration,float dt)
    {
        var core=Host.FieldCore;var bodies=core.Bodies;
        if(bodies.IsAwake(b))return true;
        var sleep=core.GetBodySleepSettings(core.HandleOfBody(b));
        float linear=sleep.IsOverride?sleep.LinearThreshold:core.Options.SleepThreshold;
        float angular=sleep.IsOverride?sleep.AngularThreshold:core.Options.SleepThreshold;
        if(Dot(acceleration,acceleration)*dt*dt<=linear*linear && Dot(angularAcceleration,angularAcceleration)*dt*dt<=angular*angular)return false;
        core.WakeIsland(bodies.Island[b]);bodies.SleepTime[b]=0;return true;
    }
    private Vector3F AngularTorque(int b, Vector3F alpha)
    {
        var bodies=Host.FieldCore.Bodies; var inertia=bodies.InverseInertiaOf(b);
        if (inertia.Length==1) return new(0,0,inertia[0]>0?alpha.Z/inertia[0]:0);
        // The spatial host uses world angular drag independently of principal axes.
        return Host.AngularTorque(b,alpha);
    }
    internal void Shift(Vector3D offset)
    {
        for(int i=0;i<Definitions.Length;i++)
        {
            var d=Definitions[i]; if(d==null) continue;
            d.Center-=offset; d.Minimum-=offset; d.Maximum-=offset;
            d.SurfaceHeight-=d.Axis.X*offset.X+d.Axis.Y*offset.Y+d.Axis.Z*offset.Z;
        }
    }
    public void Reset() { Definitions=[]; Generations=[]; Count=0; _moments=[]; _momentGenerations=[]; _buoyancyDelta=[]; }
    public ISnapshotSection Fork(WorldCore destination)
    {
        var h=(IFieldHost)destination.GetSnapshotSection(Host.EngineSectionId);
        var copy=h.Fields;
        copy.Definitions=new FieldDefinition?[Definitions.Length];
        for(int i=0;i<Definitions.Length;i++) copy.Definitions[i]=Definitions[i]?.Clone();
        copy.Generations=(int[])Generations.Clone();copy.Count=Count;
        copy._buoyancyDelta=(float[])_buoyancyDelta.Clone();copy._moments=(Vector3F[])_moments.Clone();copy._momentGenerations=(int[])_momentGenerations.Clone();
        return copy;
    }
    public void Write(StateWriter w)
    {
        w.NamedInt32(1,"version");w.NamedInt32(Definitions.Length,"fieldSlots");
        for(int i=0;i<Definitions.Length;i++)
        {
            w.DescribeEntity("field",i,Generations[i]);w.NamedInt32(Generations[i],"generation");w.NamedBool(Definitions[i]!=null,"live");
            if(Definitions[i] is {} d) WriteDefinition(w,d);
        }
        w.NamedInt32(_moments.Length,"magneticSlots");
        for(int i=0;i<_moments.Length;i++) { w.DescribeEntity("magneticBody",i,_momentGenerations[i]);w.NamedInt32(_momentGenerations[i]);WriteVector(w,_moments[i]);w.NamedSingle(BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(_buoyancyDelta[i])^0x3F800000)); }
    }
    public void Read(StateReader r)
    {
        if(r.ReadInt32()!=1) throw r.Error("unsupported fields version");
        int n=r.ReadCount(5,"field slots");var defs=new FieldDefinition?[n];var generations=new int[n];int count=0;
        for(int i=0;i<n;i++)
        {
            generations[i]=r.ReadInt32();if(generations[i]<=0) throw r.Error("invalid field generation");
            if(r.ReadBool()) { defs[i]=ReadDefinition(r); try {ValidateDimension(defs[i]!);}catch(ArgumentException ex){throw r.Error(ex.Message);} count++; }
        }
        int m=r.ReadCount(20,"magnetic slots"); var moments=new Vector3F[m];var mg=new int[m];var bd=new float[m];
        for(int i=0;i<m;i++) { mg[i]=r.ReadInt32();moments[i]=ReadVector(r);bd[i]=r.ReadSingle();if(!float.IsFinite(bd[i])||bd[i]<0||bd[i]>1e6f)throw r.Error("invalid buoyancy factor");bd[i]=BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(bd[i])^0x3F800000);try { FieldDefinition.CheckVector(moments[i],"magnetic moment");if(mg[i]<0)throw r.Error("negative magnetic generation"); } catch(ArgumentException ex) { throw r.Error(ex.Message); } }
        Definitions=defs;Generations=generations;Count=count;_moments=moments;_momentGenerations=mg;_buoyancyDelta=bd;
    }
    private static void WriteVector(StateWriter w,Vector3F v) { w.NamedSingle(v.X);w.NamedSingle(v.Y);w.NamedSingle(v.Z); }
    private static void WritePoint(StateWriter w,Vector3D v) { w.NamedDouble(v.X);w.NamedDouble(v.Y);w.NamedDouble(v.Z); }
    private static Vector3F ReadVector(StateReader r)=>new(r.ReadSingle(),r.ReadSingle(),r.ReadSingle());
    private static Vector3D ReadPoint(StateReader r)=>new(r.ReadDouble(),r.ReadDouble(),r.ReadDouble());
    private static void WriteDefinition(StateWriter w,FieldDefinition d)
    {
        w.NamedInt32((int)d.Kind);WritePoint(w,d.Minimum);WritePoint(w,d.Maximum);WritePoint(w,d.Center);
        WriteVector(w,d.Direction);WriteVector(w,d.Axis);WriteVector(w,d.Flow);
        w.NamedSingle(d.Strength);w.NamedSingle(d.Radius);w.NamedSingle(d.Softening);w.NamedInt32((int)d.Falloff);w.NamedBool(d.Acceleration);w.NamedBool(d.Line);
        w.NamedSingle(d.Drag);w.NamedSingle(d.QuadraticDrag);w.NamedSingle(d.AngularDrag);w.NamedSingle(d.Density);w.NamedDouble(d.SurfaceHeight);w.NamedSingle(d.Inward);
        w.NamedInt32(d.Priority);w.NamedSingle(d.Weight);w.NamedUInt32(d.Layers);w.NamedInt32(d.Sensor);w.NamedInt32(d.SensorGeneration);w.NamedInt32(d.CallbackId);w.NamedSingle(d.SideFriction);w.NamedSingle(d.SideBounce);
    }
    private static FieldDefinition ReadDefinition(StateReader r)
    {
        var d=new FieldDefinition { Kind=(FieldKind)r.ReadInt32(),Minimum=ReadPoint(r),Maximum=ReadPoint(r),Center=ReadPoint(r),Direction=ReadVector(r),Axis=ReadVector(r),Flow=ReadVector(r),Strength=r.ReadSingle(),Radius=r.ReadSingle(),Softening=r.ReadSingle(),Falloff=(FieldFalloff)r.ReadInt32(),Acceleration=r.ReadBool(),Line=r.ReadBool(),Drag=r.ReadSingle(),QuadraticDrag=r.ReadSingle(),AngularDrag=r.ReadSingle(),Density=r.ReadSingle(),SurfaceHeight=r.ReadDouble(),Inward=r.ReadSingle(),Priority=r.ReadInt32(),Weight=r.ReadSingle(),Layers=r.ReadUInt32(),Sensor=r.ReadInt32(),SensorGeneration=r.ReadInt32(),CallbackId=r.ReadInt32(),SideFriction=r.ReadSingle(),SideBounce=r.ReadSingle() };
        try { d.ValidateStored(); } catch(ArgumentException ex) { throw r.Error("invalid field: "+ex.Message); }return d;
    }
}
