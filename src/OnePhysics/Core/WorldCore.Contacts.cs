namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    internal readonly ContactModificationPipeline ContactModifications = new();

    /// <summary>Raw-tier pre-solve manifold callback, null by default. The built-in engines invoke it serially after narrowphase and before solving, in shape-pair/subshape order. The view directly edits the constraint payload. Do not mutate other world storage from this callback. Not serialized or copied by Fork.</summary>
    public ContactModifier? ContactModifier
    {
        get => ContactModifications.Callback;
        set => ContactModifications.Callback = value;
    }
}
