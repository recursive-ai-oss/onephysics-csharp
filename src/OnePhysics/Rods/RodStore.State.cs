using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    public void Write(StateWriter w)
    {
        w.NamedInt32(2,"version");
        w.NamedSingle(Statistics.MaximumSpeed);w.NamedDouble(Statistics.MaximumStretch);w.NamedSingle(Statistics.MaximumPenetration);w.NamedInt32(Statistics.Contacts);w.NamedInt32(Statistics.ConservativeStops);
        w.NamedInt32(Data.Length,"rodSlots");
        for (int i = 0; i < Data.Length; i++)
        {
            w.DescribeEntity("rod",i,Generations[i]); w.NamedInt32(Generations[i],"generation"); w.NamedBool(Data[i] != null,"live");
            if (Data[i] is not {} d) continue;
            var m = d.Material;
            w.NamedSingle(m.YoungModulus); w.NamedSingle(m.ShearModulus); w.NamedSingle(m.Radius); w.NamedSingle(m.Density);
            w.NamedInt32(d.Substeps); w.NamedInt32(d.Iterations); w.NamedSingle(d.Damping); w.NamedSingle(d.GravityScale); w.NamedBool(d.ClampStart); WriteFrame(w,d.RootFrame);
            w.NamedSingle(d.Friction);w.NamedSingle(d.Restitution);w.NamedBool(d.CollisionEnabled);w.NamedBool(d.SelfCollision);w.NamedBool(d.CollideWithAttachedBodies);w.NamedUInt32(d.Category);w.NamedUInt32(d.Mask);w.NamedInt32(d.Layer);w.NamedInt32(d.Group);
            w.NamedInt32(d.PositionData.Length,"particleCount");
            for (int p = 0; p < d.PositionData.Length; p++)
            {

                WritePoint(w,d.PositionData[p]); WritePoint(w,d.PinTarget[p]); WriteVector(w,d.VelocityData[p]); w.NamedBool(d.Pinned[p]); w.NamedSingle(d.InverseMass[p]);
                w.NamedInt32(d.AttachmentBody[p]);w.NamedInt32(d.AttachmentGeneration[p]);WriteVector(w,d.AttachmentLocal[p]);w.NamedSingle(d.AttachmentBreak[p]);WriteVector(w,d.AttachmentForce[p]);w.NamedBool(d.AttachmentBroken[p]);
            }
            for (int s = 0; s < d.Length.Length; s++)
            {

                WriteFrame(w,d.FrameData[s]); WriteVector(w,d.AngularVelocity[s]); w.NamedSingle(d.Length[s]); WriteVector(w,d.RestBend[s]);
                w.NamedBool(d.Clamped[s]); w.NamedSingle(d.InverseBendInertia[s]); w.NamedSingle(d.InverseTwistInertia[s]);
            }
            w.NamedDoubles(d.LambdaData);
        }
    }
    public void Read(StateReader r)
    {
        int version=r.ReadInt32();if (version != 1 && version != 2) throw r.Error("Unsupported rod section version.");
        RodStepStatistics statistics=default;
        if(version>=2)
        {
            float speed=r.ReadSingle();double stretch=r.ReadDouble();float penetration=r.ReadSingle();int contacts=r.ReadInt32(),stops=r.ReadInt32();
            if(!(speed>=0&&float.IsFinite(speed)&&stretch>=0&&double.IsFinite(stretch)&&penetration>=0&&float.IsFinite(penetration))||contacts<0||contacts>1048576||stops<0)throw r.Error("Invalid rod diagnostics.");
            statistics=new(speed,stretch,penetration,contacts,stops);
        }
        int slots = r.ReadCount(5,"rod slots"); if (slots > 65536) throw r.Error("Rod slot limit 65536 exceeded.");
        var data = new RodParticles?[slots]; var generations = new int[slots]; int count = 0, particles = 0;
        for (int i = 0; i < slots; i++)
        {
            int gen = r.ReadInt32(); if (gen < 1) throw r.Error("Rod generation must be positive."); generations[i] = gen;
            if (!r.ReadBool()) continue;
            RodMaterial material;
            try { material = new RodMaterial(r.ReadSingle(),r.ReadSingle(),r.ReadSingle(),r.ReadSingle()); }
            catch (ArgumentException ex) { throw r.Error("Rod material: "+ex.Message); }
            int substeps = r.ReadInt32(), iterations = r.ReadInt32(); float damping = r.ReadSingle(), gravity = r.ReadSingle();
            bool clampStart = r.ReadBool(); var rootFrame = ReadFrame(r);
            double rootNorm = (double)rootFrame.X*rootFrame.X+(double)rootFrame.Y*rootFrame.Y+(double)rootFrame.Z*rootFrame.Z+(double)rootFrame.W*rootFrame.W;
            if (!(rootNorm >= 0.999 && rootNorm <= 1.001) || (Host.Planar && (rootFrame.X != 0 || rootFrame.Y != 0))) throw r.Error("Invalid rod boundary frame.");
            if (substeps < 1 || substeps > 64 || iterations < 1 || iterations > 32 || !(damping >= 0 && damping <= 1000) || !(gravity >= -1000 && gravity <= 1000)) throw r.Error("Invalid rod solve options.");
            float friction=0.4f,restitution=0;bool collide=true,self=true,collideAttached=false;uint category=1,mask=uint.MaxValue;int layer=0,group=0;
            if(version>=2)
            {
                friction=r.ReadSingle();restitution=r.ReadSingle();collide=r.ReadBool();self=r.ReadBool();collideAttached=r.ReadBool();category=r.ReadUInt32();mask=r.ReadUInt32();layer=r.ReadInt32();group=r.ReadInt32();
                if(!(friction>=0&&friction<=100&&restitution>=0&&restitution<=1)||(uint)layer>31)throw r.Error("Invalid rod contact properties.");
            }
            int n = r.ReadCount(version>=2?102:65,"rod particles");
            if (n < 2 || n > 4097 || (particles += n) > 1048576) throw r.Error("Invalid rod particle count (per rod 2..4097, world maximum 1048576).");
            var attached=new int[n];Array.Fill(attached,-1);var generationsOfBody=new int[n];var anchors=new Vector3F[n];var breaking=new float[n];Array.Fill(breaking,float.MaxValue);var forces=new Vector3F[n];var broken=new bool[n];
            var points = new Vector3D[n]; var targets = new Vector3D[n]; var velocity = new Vector3F[n]; var pins = new bool[n]; var masses = new float[n];
            for (int p = 0; p < n; p++)
            {
                points[p] = ReadPoint(r); targets[p] = ReadPoint(r); velocity[p] = ReadVector(r); pins[p] = r.ReadBool(); masses[p] = r.ReadSingle();
                if(version>=2)
                {
                    attached[p]=r.ReadInt32();generationsOfBody[p]=r.ReadInt32();anchors[p]=ReadVector(r);breaking[p]=r.ReadSingle();forces[p]=ReadVector(r);broken[p]=r.ReadBool();
                    if(attached[p]<-1 || (attached[p]>=0&&(attached[p]>=Host.RodCore.Bodies.HighWater||generationsOfBody[p]<=0)) || !Finite(anchors[p]) || Math.Abs(anchors[p].X)>2097152 || Math.Abs(anchors[p].Y)>2097152 || Math.Abs(anchors[p].Z)>2097152 || !(breaking[p]>0&&float.IsFinite(breaking[p])) || !Finite(forces[p]) || (Host.Planar&&(anchors[p].Z!=0||forces[p].Z!=0)) || (pins[p]&&attached[p]>=0))throw r.Error("Invalid rod attachment.");
                }
                var x = points[p]; var target = targets[p];
                if (!(Math.Abs(target.X) <= 4194304 && Math.Abs(target.Y) <= 4194304 && Math.Abs(target.Z) <= 4194304)) throw r.Error("Invalid rod pin target.");
                if (!(Math.Abs(x.X) <= 4194304 && Math.Abs(x.Y) <= 4194304 && Math.Abs(x.Z) <= 4194304) || !Finite(velocity[p]) || !float.IsFinite(masses[p]) || masses[p] < 0 || (pins[p] != (masses[p] == 0))) throw r.Error("Invalid rod particle position, velocity or inverse mass.");
                if (Host.Planar && (x.Z != 0 || velocity[p].Z != 0 || target.Z != 0)) throw r.Error("Planar rod has a nonzero Z component.");
            }
            // Rebuild immutable allocation sizes from a safe synthetic rest line; actual rest geometry is stored below.
            var rest = new Vector3D[n]; for (int p = 0; p < n; p++) rest[p] = new Vector3D(p*0.01,0,0);
            var d = new RodParticles(rest,material,Host.Planar,[],[],default,substeps,iterations,damping,gravity,clampStart);
            d.RootFrame = rootFrame;d.Friction=friction;d.Restitution=restitution;d.CollisionEnabled=collide;d.SelfCollision=self;d.CollideWithAttachedBodies=collideAttached;d.Category=category;d.Mask=mask;d.Layer=layer;d.Group=group;
            d.AttachmentBody=attached;d.AttachmentGeneration=generationsOfBody;d.AttachmentLocal=anchors;d.AttachmentBreak=breaking;d.AttachmentForce=forces;d.AttachmentBroken=broken;
            d.PinTarget = targets; d.PinStart = (Vector3D[])points.Clone(); d.PositionData = points; d.Previous = (Vector3D[])points.Clone(); d.VelocityData = velocity; d.Pinned = pins; d.InverseMass = masses;
            for (int s = 0; s < n-1; s++)
            {
                d.FrameData[s] = ReadFrame(r); d.OldFrame[s] = d.FrameData[s]; d.AngularVelocity[s] = ReadVector(r);
                d.Length[s] = r.ReadSingle(); d.RestBend[s] = ReadVector(r); d.Clamped[s] = r.ReadBool();
                d.InverseBendInertia[s] = r.ReadSingle(); d.InverseTwistInertia[s] = r.ReadSingle();
                var q = d.FrameData[s]; double norm = (double)q.X*q.X+(double)q.Y*q.Y+(double)q.Z*q.Z+(double)q.W*q.W;
                if (!(norm >= 0.999 && norm <= 1.001) || !Finite(d.AngularVelocity[s]) || !(d.Length[s] >= 0.0001f && d.Length[s] <= 100) || !Finite(d.RestBend[s]) || !(d.InverseBendInertia[s] >= 0) || !float.IsFinite(d.InverseBendInertia[s]) || !(d.InverseTwistInertia[s] >= 0) || !float.IsFinite(d.InverseTwistInertia[s])) throw r.Error("Invalid rod segment frame, velocity, rest length or inertia.");
                if (Host.Planar && (q.X != 0 || q.Y != 0 || d.AngularVelocity[s].X != 0 || d.AngularVelocity[s].Y != 0)) throw r.Error("Planar rod has a nonplanar frame.");
            }
            r.ReadDoubles(d.LambdaData,true,"rod multipliers");
            foreach (double value in d.LambdaData) if (!double.IsFinite(value)) throw r.Error("Non-finite rod multiplier.");
            data[i] = d; count++;
        }
        Data = data; Generations = generations; Count = count;Statistics=statistics;
    }
    private static void WritePoint(StateWriter w, Vector3D p) { w.NamedDouble(p.X); w.NamedDouble(p.Y); w.NamedDouble(p.Z); }
    private static void WriteVector(StateWriter w, Vector3F p) { w.NamedSingle(p.X); w.NamedSingle(p.Y); w.NamedSingle(p.Z); }
    private static void WriteFrame(StateWriter w, QuaternionF q) { w.NamedSingle(q.X); w.NamedSingle(q.Y); w.NamedSingle(q.Z); w.NamedSingle(q.W); }
    private static Vector3D ReadPoint(StateReader r) => new(r.ReadDouble(),r.ReadDouble(),r.ReadDouble());
    private static Vector3F ReadVector(StateReader r) => new(r.ReadSingle(),r.ReadSingle(),r.ReadSingle());
    private static QuaternionF ReadFrame(StateReader r) => new(r.ReadSingle(),r.ReadSingle(),r.ReadSingle(),r.ReadSingle());
}

public sealed partial class RodParticles
{
    internal RodParticles Copy()
    {
        int n = PositionData.Length;
        var line = new Vector3D[n]; for (int i = 0; i < n; i++) line[i] = new Vector3D(i*0.01,0,0);
        var c = new RodParticles(line,Material,Planar,[],[],default,Substeps,Iterations,Damping,GravityScale,ClampStart);
        c.RootFrame = RootFrame;c.Friction=Friction;c.Restitution=Restitution;c.CollisionEnabled=CollisionEnabled;c.SelfCollision=SelfCollision;c.CollideWithAttachedBodies=CollideWithAttachedBodies;c.Category=Category;c.Mask=Mask;c.Layer=Layer;c.Group=Group;
        c.AttachmentBody=(int[])AttachmentBody.Clone();c.AttachmentGeneration=(int[])AttachmentGeneration.Clone();c.AttachmentLocal=(Vector3F[])AttachmentLocal.Clone();c.AttachmentBreak=(float[])AttachmentBreak.Clone();c.AttachmentForce=(Vector3F[])AttachmentForce.Clone();c.AttachmentBroken=(bool[])AttachmentBroken.Clone();
        c.PinTarget = (Vector3D[])PinTarget.Clone(); c.PinStart = (Vector3D[])PinStart.Clone(); c.PositionData = (Vector3D[])PositionData.Clone(); c.Previous = (Vector3D[])Previous.Clone(); c.VelocityData = (Vector3F[])VelocityData.Clone();
        c.FrameData = (QuaternionF[])FrameData.Clone(); c.OldFrame = (QuaternionF[])OldFrame.Clone(); c.AngularVelocity = (Vector3F[])AngularVelocity.Clone();
        c.Length = (float[])Length.Clone(); c.RestBend = (Vector3F[])RestBend.Clone(); c.InverseMass = (float[])InverseMass.Clone();
        c.InverseBendInertia = (float[])InverseBendInertia.Clone(); c.InverseTwistInertia = (float[])InverseTwistInertia.Clone(); c.Pinned = (bool[])Pinned.Clone(); c.Clamped = (bool[])Clamped.Clone();
        LambdaData.CopyTo(c.LambdaData,0);
        return c;
    }
}
