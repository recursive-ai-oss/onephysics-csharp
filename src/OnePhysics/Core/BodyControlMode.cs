namespace RecursiveAI.OnePhysics.LowLevel;

// Struct specialization removes control-only loads from worlds using the original contact layout.
internal interface IBodyControlMode { static abstract bool Enabled { get; } }
internal readonly struct StandardBodyControls : IBodyControlMode { public static bool Enabled => false; }
internal readonly struct ConfiguredBodyControls : IBodyControlMode { public static bool Enabled => true; }
