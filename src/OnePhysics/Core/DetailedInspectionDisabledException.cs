using System;

namespace RecursiveAI.OnePhysics;

/// <summary>A detail getter was used without collection enabled for the latest step.</summary>
public sealed class DetailedInspectionDisabledException : InvalidOperationException
{
    /// <summary>Creates an error directing the caller to enable detailed inspection before stepping.</summary>
    public DetailedInspectionDisabledException() : base("Set Options.DetailedInspection = true before stepping (and before applying direct impulses) to collect body impulses and whole-step joint inspection.") { }
}
