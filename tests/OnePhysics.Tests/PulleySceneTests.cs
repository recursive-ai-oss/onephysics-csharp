using System;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class PulleySceneTests(ITestOutputHelper helper)
{
    private void Log(string s){helper.WriteLine(s);var f=Environment.GetEnvironmentVariable("PULLEY_LOG");if(f!=null)System.IO.File.AppendAllText(f,s+"\n");}

    [Fact]
    public void Debug()
    {
        foreach(double amp in new[]{.1,.15,.2}){PulleyScene.SlewAmplitude=amp;var sc=PulleyScene.Create("crane-3d",1);double mx=0;for(int i=0;i<600;i++){sc.Step(i);mx=Math.Max(mx,sc.MaxLoadedStretch());}Log($"slew {amp}: max total stretch {mx:E2}");}
        PulleyScene.SlewAmplitude=.15;
    }

    [Theory]
    [InlineData("tackle-2d")]
    [InlineData("belt-2d")]
    [InlineData("chain-drive-3d")]
    [InlineData("crane-3d")]
    public void SceneRuns(string name)
    {
        var scene=PulleyScene.Create(name,1);
        double speed=0,stretch=0,pen=0;int tunnelled=0;
        for(int i=0;i<600;i++)
        {
            scene.Step(i);
            speed=Math.Max(speed,scene.MaxSpeed());stretch=Math.Max(stretch,scene.MaxLoadedStretch());pen=Math.Max(pen,scene.Penetration());tunnelled=Math.Max(tunnelled,scene.Tunnelled());
            if(i%100==99)Log($"{name} step {i}: {scene.Extra()} speed={scene.MaxSpeed():F3} stretch={scene.MaxLoadedStretch():E2}");
        }
        Log($"{name}: maxSpeed={speed:F4} maxLoadedStretch={stretch:E3} lengthDrift={scene.LengthDrift():E2} penetration={pen:E2} tunnelled={tunnelled} hash={scene.Hash():x16}");
    }
}
