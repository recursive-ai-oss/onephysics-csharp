using System.Globalization;
using RecursiveAI.OneMath;

namespace Bar;

internal static class FieldStability
{
    internal static int Run()
    {
        var pool=FieldScenes.Planar(1);double maxSpeed=0,maxDraftError=0,restEnergy=0;
        for(int i=0;i<600;i++)
        {
            pool.Step(1f/60);
            for(int b=1;b<pool.Core.Bodies.HighWater;b++)
            {
                var body=pool.GetBody(pool.Core.HandleOfBody(b));var v=body.LinearVelocity;
                double speed=Math.Sqrt(v.X*v.X+v.Y*v.Y);maxSpeed=Math.Max(maxSpeed,speed);
                if(i==599){maxDraftError=Math.Max(maxDraftError,Math.Abs(body.Position.Y));restEnergy+=0.5*body.Mass*speed*speed+0.5*body.Inertia*body.AngularVelocity*body.AngularVelocity;}
            }
        }
        var fields=FieldScenes.Spatial(1);double maxOrbitError=0,maxFieldSpeed=0;
        for(int i=0;i<600;i++)
        {
            fields.Step(1f/60);var orbit=fields.GetBody(fields.Core.HandleOfBody(0));var p=orbit.Position;
            maxOrbitError=Math.Max(maxOrbitError,Math.Abs(Math.Sqrt(p.X*p.X+p.Y*p.Y+p.Z*p.Z)-10));
            for(int b=0;b<fields.Core.Bodies.HighWater;b++){var v=fields.GetBody(fields.Core.HandleOfBody(b)).LinearVelocity;maxFieldSpeed=Math.Max(maxFieldSpeed,Math.Sqrt(v.X*v.X+v.Y*v.Y+v.Z*v.Z));}
        }
        bool buoyancyOk=double.IsFinite(restEnergy)&&maxSpeed<10&&maxDraftError<0.02&&restEnergy<0.05,fieldsOk=maxOrbitError<0.03&&maxFieldSpeed<20;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"buoyancy-2d steps=600 maxSpeed={maxSpeed:R} equilibriumDepthError={maxDraftError:R} restKE={restEnergy:R} {(buoyancyOk?"PASS":"FAIL")}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"fields-3d steps=600 maxSpeed={maxFieldSpeed:R} orbitRadiusError={maxOrbitError:R} {(fieldsOk?"PASS":"FAIL")}"));
        return buoyancyOk&&fieldsOk?0:1;
    }
}
