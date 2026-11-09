using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private (int Constraint,int Generation,float Friction,float Bounce)[] _fieldMaterials=[];
    private int _fieldMaterialCount;
    internal void EndFieldContacts()
    {
        var cs=Core.Constraints;
        for(int i=_fieldMaterialCount-1;i>=0;i--)
        {
            var saved=_fieldMaterials[i];
            if(!cs.Handles.IsAlive(saved.Constraint,saved.Generation))continue;
            var pl=cs.Payload(saved.Constraint);pl[ManifoldLayout.Friction]=saved.Friction;pl[ManifoldLayout.Restitution]=saved.Bounce;
        }
        _fieldMaterialCount=0;
    }
    internal void ApplyFieldContacts()
    {
        if(_fields==null||_fields.Count==0)return;
        var cs=Core.Constraints;var bodies=Core.Bodies;
        for(int f=0;f<_fields.Definitions.Length;f++)
        {
            var d=_fields.Definitions[f];if(d==null||d.Kind is not (FieldKind.Platform or FieldKind.Surface))continue;
            for(int c=0;c<cs.HighWater;c++)
            {
                if(!cs.IsLive(c)||cs.KindOf(c)!=ConstraintKind.Contact||cs.PointCount[c]==0)continue;
                int a=cs.BodyA[c],b=cs.BodyB[c];int surface=!bodies.IsDynamic(a)?a:!bodies.IsDynamic(b)?b:-1;
                if(surface<0||!_fields.Matches(surface,d))continue;
                int body=surface==a?b:a;if(!bodies.IsDynamic(body))continue;
                var pl=cs.Payload(c);float sign=surface==a?1:-1;
                var n=new Vector3F(sign*pl[ManifoldLayout.Normal],sign*pl[ManifoldLayout.Normal+1],0);
                if(d.Kind==FieldKind.Platform)
                {
                    float alignment=FieldStore.Dot(n,d.Axis);
                    if(alignment < -0.5f) {cs.PointCount[c]=0;pl.Clear();}
                    else if(alignment<0.5f)
                    {
                        if(_fieldMaterialCount==_fieldMaterials.Length)Array.Resize(ref _fieldMaterials,Math.Max(8,2*_fieldMaterials.Length));
                        _fieldMaterials[_fieldMaterialCount++]=(c,cs.Handles.GenerationOf(c),pl[ManifoldLayout.Friction],pl[ManifoldLayout.Restitution]);
                        pl[ManifoldLayout.Friction]=d.SideFriction;pl[ManifoldLayout.Restitution]=d.SideBounce;
                    }
                }
                else
                {
                    // Contact-only conveyor drive. The tangent projection leaves normal response to the solver.
                    var tangent=d.Direction-n*FieldStore.Dot(d.Direction,n);var v=bodies.LinearVelocityOf(body);
                    var velocity=new Vector3F(v[0],v[1],0);
                    var acceleration=(tangent*d.Strength-(velocity-n*FieldStore.Dot(velocity,n)))*d.Drag;
                    if(!bodies.IsAwake(body)) {Core.WakeIsland(bodies.Island[body]);bodies.SleepTime[body]=0;}
                    ((IFieldHost)this).AddForce(body,acceleration/bodies.InverseMass[body],default);
                }
            }
        }
    }
}
