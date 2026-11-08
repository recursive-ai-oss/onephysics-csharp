using System;
using System.IO;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Replay;

var world=new PhysicsWorld2D(new WorldOptions2D { WorldId=106 });
world.AddCircle(0.5f,at:new(0,3));
using var stream=new MemoryStream();
using(var recorder=world.StartRecording(stream))world.Step(1f/60);
stream.Position=0;var player=new ReplayPlayer(stream);var copy=player.CreateWorld2D();
if(!player.ReadNextStep(copy) || copy.ComputeHash()!=world.ComputeHash())return 1;
string text=copy.SaveScene();var loaded=PhysicsWorld2D.LoadScene(new StringReader(text));
if(loaded.SaveScene()!=text)return 2;
using var history=loaded.StartHistory(new HistoryOptions { KeyframeInterval=2, MemoryBudgetBytes=4*1024*1024 });
ulong hash=loaded.ComputeHash();loaded.Step(1f/60);history.Seek(0,loaded);
if(loaded.ComputeHash()!=hash)return 3;
Console.WriteLine("OK planar formats");return 0;
