using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Rods;

internal sealed partial class RodStore
{
    public void Write(StateWriter w)
    {
        w.NamedInt32(1,"version"); w.NamedInt32(Data.Length,"rodSlots");
        for (int i = 0; i < Data.Length; i++)
        {
            w.DescribeEntity("rod",i,Generations[i]); w.NamedInt32(Generations[i],"generation"); w.NamedBool(Data[i] != null,"live");
            if (Data[i] is not {} d) continue;
            var m = d.Material;
            w.NamedSingle(m.YoungModulus); w.NamedSingle(m.ShearModulus); w.NamedSingle(m.Radius); w.NamedSingle(m.Density);
            w.NamedInt32(d.Substeps); w.NamedInt32(d.Iterations); w.NamedSingle(d.Damping); w.NamedSingle(d.GravityScale); w.NamedBool(d.ClampStart); WriteFrame(w,d.RootFrame);
            w.NamedInt32(d.PositionData.Length,"particleCount");
            for (int p = 0; p < d.PositionData.Length; p++)
            {

                WritePoint(w,d.PositionData[p]); WritePoint(w,d.PinTarget[p]); WriteVector(w,d.VelocityData[p]); w.NamedBool(d.Pinned[p]); w.NamedSingle(d.InverseMass[p]);
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
        if (r.ReadInt32() != 1) throw r.Error("Unsupported rod section version.");
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
            int n = r.ReadCount(65,"rod particles");
            if (n < 2 || n > 4097 || (particles += n) > 1048576) throw r.Error("Invalid rod particle count (per rod 2..4097, world maximum 1048576).");
            var points = new Vector3D[n]; var targets = new Vector3D[n]; var velocity = new Vector3F[n]; var pins = new bool[n]; var masses = new float[n];
            for (int p = 0; p < n; p++)
            {
                points[p] = ReadPoint(r); targets[p] = ReadPoint(r); velocity[p] = ReadVector(r); pins[p] = r.ReadBool(); masses[p] = r.ReadSingle();
                var x = points[p]; var target = targets[p];
                if (!(Math.Abs(target.X) <= 4194304 && Math.Abs(target.Y) <= 4194304 && Math.Abs(target.Z) <= 4194304)) throw r.Error("Invalid rod pin target.");
                if (!(Math.Abs(x.X) <= 4194304 && Math.Abs(x.Y) <= 4194304 && Math.Abs(x.Z) <= 4194304) || !Finite(velocity[p]) || !float.IsFinite(masses[p]) || masses[p] < 0 || (pins[p] != (masses[p] == 0))) throw r.Error("Invalid rod particle position, velocity or inverse mass.");
                if (Host.Planar && (x.Z != 0 || velocity[p].Z != 0 || target.Z != 0)) throw r.Error("Planar rod has a nonzero Z component.");
            }
            // Rebuild immutable allocation sizes from a safe synthetic rest line; actual rest geometry is stored below.
            var rest = new Vector3D[n]; for (int p = 0; p < n; p++) rest[p] = new Vector3D(p*0.01,0,0);
            var d = new RodParticles(rest,material,Host.Planar,[],[],default,substeps,iterations,damping,gravity,clampStart);
            d.RootFrame = rootFrame;
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
        Data = data; Generations = generations; Count = count;
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
        c.RootFrame = RootFrame;
        c.PinTarget = (Vector3D[])PinTarget.Clone(); c.PinStart = (Vector3D[])PinStart.Clone(); c.PositionData = (Vector3D[])PositionData.Clone(); c.Previous = (Vector3D[])Previous.Clone(); c.VelocityData = (Vector3F[])VelocityData.Clone();
        c.FrameData = (QuaternionF[])FrameData.Clone(); c.OldFrame = (QuaternionF[])OldFrame.Clone(); c.AngularVelocity = (Vector3F[])AngularVelocity.Clone();
        c.Length = (float[])Length.Clone(); c.RestBend = (Vector3F[])RestBend.Clone(); c.InverseMass = (float[])InverseMass.Clone();
        c.InverseBendInertia = (float[])InverseBendInertia.Clone(); c.InverseTwistInertia = (float[])InverseTwistInertia.Clone(); c.Pinned = (bool[])Pinned.Clone(); c.Clamped = (bool[])Clamped.Clone();
        LambdaData.CopyTo(c.LambdaData,0);
        return c;
    }
}
