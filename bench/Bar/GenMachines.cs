namespace Bar;

using Scene = RecursiveAI.OnePhysics.Scene.Scene;
using JointDef = RecursiveAI.OnePhysics.Scene.JointDef;

/// <summary>Seeded, separated machine fixtures exercising the joint families without external geometry.</summary>
public static class GenMachines
{
    public static Scene Build(int dimension)
    {
        var s = Gen.Base(dimension == 2 ? "joint-machines-2d" : "joint-machines-3d", 600, "maxSpeed", "nan", "escapes");
        s.Dim = dimension; s.Gx = s.Gy = s.Gz = 0; s.DumpEvery = 1;
        s.Params["engines"] = ["onephysics"];
        s.Params["container"] = ["-5", "-5", "170", "15"];
        s.Params["world-id"] = [dimension == 2 ? "482" : "483"];
        var random = new Gen.Rng(0x4d414348494e45);
        int Body(string type, double x, double y)
        {
            var b = Gen.Add(s, type, x, y);
            if (dimension == 2) Gen.Shape(b, "circle", 1, 0.1, 0, 0);
            else Gen.Shape(b, "sphere", 1, 0.1, 0, 0, 0);
            return s.Bodies.Count - 1;
        }
        void Joint(string kind, int a, int b, double ax = 0, double ay = 0, double bx = 0, double by = 0, params double[] data)
            => s.Joints.Add(new JointDef { Kind = kind, A = a, B = b, Ax = ax, Ay = ay, Bx = bx, By = by, Ux = 1, Data = data });
        for (int i = 0; i < 8; i++)
        {
            double x = i * 20; int g = Body("static", x, 0);
            int a = Body("dynamic", x, 2), b = Body("dynamic", x, 4);
            Joint(dimension == 2 ? "revolute" : "hinge", g, a, 0, 2);
            Joint(dimension == 2 ? "revolute" : "hinge", g, b, 0, 4);
            Joint("gear", a, b, data: [2]);
            Joint("filter", a, b);
            s.Joints[^1].LifetimeSteps = 150;
            s.Joints[^1].BreakForce = 0; // A filter applies no impulse, so only lifetime expires it.
            if (dimension == 2) s.Bodies[a].W = random.Range(0.5, 1);
            else s.Bodies[a].Wx = random.Range(0.5, 1);
            int c = Body("dynamic", x + 2, 2), d = Body("dynamic", x + 4, 2);
            s.Bodies[c].Vy = 0.1;
            Joint("pulley", c, d, data: dimension == 2 ? [1, 0, x + 2, 6, x + 4, 6] : [1, x + 2, 6, 0, x + 4, 6, 0]);
            int e = Body("dynamic", x + 6, 2);
            s.Bodies[e].Vx = 0.2;
            Joint("wheel", g, e, 6, 2, data: dimension == 2 ? [1, 0, 3, 0.7] : [3, 0.7]);
            if (dimension == 2)
            {
                int f = Body("dynamic", x + 8, 2);
                Joint("configurable", g, f, 8, 2, data: [0, 0, 1, -0.5, 0.5, 1, 0.2, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
                int h = Body("dynamic", x, 6);
                Joint("path", g, h, data: [0, 1, 0, 6, 2, 6, 4, 7]);
                s.Joints[^1].HasMotor = true; s.Joints[^1].MotorSpeed = 0.2; s.Joints[^1].MaxTorque = 10;
            }
            else
            {
                int f = Body("dynamic", x + 8.05, 2.05);
                Joint("motor", g, f, 8, 2, data: [0, 0, 0, 1, 10, 10, 0.3]);
                int h = Body("dynamic", x + 10.1, 2.1);
                Joint("mouse", g, h, data: [x + 10, 2, 0, 5, 0.7, 10]);
            }
        }
        return s;
    }
}
