using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Read-only current contacts, including speculative contacts, in constraint-slot order. Allocation-free; read between mutations. Throws InvalidHandleException once this body is destroyed.</summary>
    public BodyContactEnumerable Contacts { get { var (w, _) = Resolve(); return w.Core.GetContacts(_handle); } }
}
