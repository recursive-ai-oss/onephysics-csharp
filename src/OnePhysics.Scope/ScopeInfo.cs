using OneScope.Sdk;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>What a OnePhysics source declares to the viewer.</summary>
internal static class ScopeInfo
{
    public const string TrackContact = "contact";
    public const string TrackSleep = "sleep";
    public const string TrackJointBreak = "joint_break";
    public const string TrackAwake = "awake";
    public const string TrackContacts = "contacts";

    public const string PokeImpulse = "impulse";
    public const string PokeSetVelocity = "set_velocity";
    public const string PokeSetPosition = "set_position";
    public const string PokeWake = "wake";

    public const int OverlayShapes = 0;
    public const int OverlayBounds = 1;
    public const int OverlayContacts = 2;
    public const int OverlayJoints = 3;
    public const int OverlayCentres = 4;
    public const int OverlayLimits = 5;

    public static SourceInfo Make(string name, int dimension) => new()
    {
        ModuleId = OnePhysicsModule.ModuleId,
        Name = name,
        Dimensions = dimension,
        Unit = LengthUnit.Metre,
        Axes = [new AxisInfo(AxisIds.Time, "Time", 0)],
        EntityKinds =
        [
            new EntityKindInfo("world", "World"),
            new EntityKindInfo("body", "Body"),
            new EntityKindInfo("joint", "Joint"),
        ],
        Overlays =
        [
            new OverlayInfo(OverlayShapes, "Shapes", true),
            new OverlayInfo(OverlayBounds, "Bounds", false),
            new OverlayInfo(OverlayContacts, "Contacts", true),
            new OverlayInfo(OverlayJoints, "Joints", true),
            new OverlayInfo(OverlayCentres, "Centres of mass", false),
            new OverlayInfo(OverlayLimits, "Joint limits", true),
        ],
        Pokes =
        [
            new PokeInfo(PokeImpulse, "Impulse", [new ParamInfo("target", ValueKind.Entity), new ParamInfo("impulse", ValueKind.Vector)]) { Role = PokeRole.DragToPoint },
            new PokeInfo(PokeSetVelocity, "Set velocity", [new ParamInfo("target", ValueKind.Entity), new ParamInfo("velocity", ValueKind.Vector)]),
            new PokeInfo(PokeSetPosition, "Set position", [new ParamInfo("target", ValueKind.Entity), new ParamInfo("position", ValueKind.Vector)]) { Role = PokeRole.DragToPoint },
            new PokeInfo(PokeWake, "Wake", [new ParamInfo("target", ValueKind.Entity)]),
        ],
        Tracks =
        [
            new TrackInfo(TrackContact, "Contacts begun", TrackKind.Event),
            new TrackInfo(TrackSleep, "Sleep and wake", TrackKind.Event),
            new TrackInfo(TrackJointBreak, "Joints broken", TrackKind.Event),
            new TrackInfo(TrackAwake, "Awake bodies", TrackKind.Series),
            new TrackInfo(TrackContacts, "Contact points", TrackKind.Series),
        ],
    };
}
