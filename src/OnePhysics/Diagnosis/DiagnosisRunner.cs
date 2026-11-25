using System;
using System.IO;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal static class DiagnosisRunner
{
    /// <summary>Diagnoses a live world: its current state, then (unless <see cref="DiagnoseOptions.ProbeSteps"/> is 0 or everything sleeps) a probe run in a copy.</summary>
    public static DiagnosisReport Live(DiagnosisHost host, DiagnoseOptions? options)
    {
        var o = options ?? new DiagnoseOptions();
        o.Validate();
        var d = new Diagnoser(o);
        d.StaticPass(host);
        var state = host;
        if (o.ProbeSteps > 0 && AnyAwakeDynamic(host))
        {
            var copy = host.Fork();
            copy.DetailedInspection = true;
            d.BeginObservation(copy, o.ProbeSteps);
            for (int i = 0; i < o.ProbeSteps; i++)
            {
                try { copy.Step(o.Dt); }
                catch (Exception e) when (e is InvalidOperationException or ArgumentException) { d.ProbeStopped(e); break; }
                d.Observe(copy, o.Dt);
            }

            state = copy;
        }

        d.Finish(state);
        return d.Report();
    }

    private static bool AnyAwakeDynamic(DiagnosisHost h)
    {
        var bodies = h.Core.Bodies;
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (bodies.IsLive(s) && bodies.TypeOf(s) == BodyType.Dynamic && bodies.IsAwake(s)) { return true; }
        }

        return false;
    }

    /// <summary>Diagnoses a window of a recording by playing it: the state at <paramref name="firstStep"/>, then each step up to <paramref name="lastStep"/>.</summary>
    public static DiagnosisReport Window(Stream replay, long firstStep, long lastStep, DiagnoseOptions? options, int threadCount)
    {
        ArgumentNullException.ThrowIfNull(replay);
        var o = options ?? new DiagnoseOptions();
        o.Validate();
        if (firstStep < 0) { throw new ArgumentOutOfRangeException(nameof(firstStep), firstStep, "firstStep must not be negative."); }
        if (lastStep < firstStep) { throw new ArgumentOutOfRangeException(nameof(lastStep), lastStep, "lastStep " + lastStep + " is before firstStep " + firstStep + "."); }
        if (lastStep - firstStep > int.MaxValue) { throw new ArgumentOutOfRangeException(nameof(lastStep), lastStep, "The window is longer than " + int.MaxValue + " steps."); }
        if (threadCount < 1) { throw new ArgumentOutOfRangeException(nameof(threadCount), threadCount, "threadCount must be at least 1."); }
        var player = new ReplayPlayer(replay);
        if (player.Dimension == 2)
        {
            var world = player.CreateWorld2D(threadCount);
            return Play(new Host2D(world), () => player.ReadNextStep(world), () => player.LastStepSeconds, player.CompletedSteps, firstStep, lastStep, o);
        }
        else
        {
            var world = player.CreateWorld3D(threadCount);
            return Play(new Host3D(world), () => player.ReadNextStep(world), () => player.LastStepSeconds, player.CompletedSteps, firstStep, lastStep, o);
        }
    }

    private static DiagnosisReport Play(DiagnosisHost host, Func<bool> next, Func<float> lastDt, long start, long firstStep, long lastStep, DiagnoseOptions o)
    {
        if (firstStep < start) { throw new ArgumentOutOfRangeException(nameof(firstStep), firstStep, "The recording starts at step " + start + "; firstStep " + firstStep + " is before it."); }
        while (host.StepCount < firstStep)
        {
            if (!next()) { throw new ArgumentOutOfRangeException(nameof(firstStep), firstStep, "The recording ends at step " + host.StepCount + ", before firstStep " + firstStep + "."); }
        }

        var d = new Diagnoser(o);
        d.StaticPass(host);
        host.DetailedInspection = true;
        int total = (int)(lastStep - firstStep);
        d.BeginObservation(host, total);
        while (host.StepCount < lastStep)
        {
            if (!next()) { throw new ArgumentOutOfRangeException(nameof(lastStep), lastStep, "The recording ends at step " + host.StepCount + ", before lastStep " + lastStep + "."); }
            d.Observe(host, lastDt());
        }

        d.Finish(host);
        return d.Report();
    }
}

/// <summary>Diagnosis over a recorded run: plays a replay file and reports what a live diagnosis would have found over the same window.</summary>
public static class ReplayDiagnostics
{
    /// <summary>
    /// Plays <paramref name="replay"/> (a recording written by <c>StartRecording</c>, or any replay file) into a new world, examines the world at <paramref name="firstStep"/> and watches every step up to
    /// <paramref name="lastStep"/>, then returns the findings. The window's length plays the part of <see cref="DiagnoseOptions.ProbeSteps"/>, each step's recorded time step the part of <see cref="DiagnoseOptions.Dt"/>;
    /// the other options apply. Steps are the world's absolute step counts, as <c>world.StepCount</c> reports them. The recorded hashes are checked as the replay plays, so a recording that does not reproduce throws.
    /// Handles in the findings carry the recorded world's id.
    /// </summary>
    /// <param name="replay">The recording; read from its start. The stream stays open.</param>
    /// <param name="firstStep">The step count at which the window starts (at least the recording's first step).</param>
    /// <param name="lastStep">The step count at which the window ends, not before <paramref name="firstStep"/>. Equal to examine one state only.</param>
    /// <param name="options">Settings; null means the defaults.</param>
    /// <param name="threadCount">Threads the playback world steps with. Findings do not depend on it.</param>
    /// <exception cref="ArgumentOutOfRangeException">A step is outside the recording, <paramref name="lastStep"/> is before <paramref name="firstStep"/>, or an option is out of range.</exception>
    /// <exception cref="ReplayFormatException">The recording is malformed.</exception>
    /// <exception cref="ReplayHashMismatchException">The recording does not reproduce on this build.</exception>
    public static DiagnosisReport Diagnose(Stream replay, long firstStep, long lastStep, DiagnoseOptions? options = null, int threadCount = 1)
        => DiagnosisRunner.Window(replay, firstStep, lastStep, options, threadCount);
}
