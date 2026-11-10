using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>How a map object takes part in the simulation (the <c>body</c> property of an object or entity).</summary>
public enum MapBodyKind
{
    /// <summary>Fixed in the world (the default for objects on collision layers).</summary>
    Static,

    /// <summary>Moved by the host, not by forces.</summary>
    Kinematic,

    /// <summary>Simulated.</summary>
    Dynamic,

    /// <summary>A static volume that reports overlaps and pushes nothing.</summary>
    Sensor,
}

/// <summary>The geometry of a <see cref="MapObject2D"/>.</summary>
public enum MapShapeKind
{
    /// <summary>A rectangle: <see cref="MapObject2D.Size"/> in the object's frame.</summary>
    Box,

    /// <summary>A circle of <see cref="MapObject2D.Radius"/>.</summary>
    Circle,

    /// <summary>A convex polygon, counter-clockwise, at most 8 points, in the object's frame.</summary>
    Polygon,

    /// <summary>An open polyline (two-sided segments) in the object's frame.</summary>
    Chain,

    /// <summary>A closed polyline loop (a concave static outline) in the object's frame.</summary>
    Loop,
}

/// <summary>A tile layer turned into a grid of collision tiles. Positions are metres, y up; <see cref="Origin"/> is the grid's bottom-left corner.</summary>
public sealed class MapTileLayer
{
    internal MapTileLayer(string name, int width, int height, float tileSize, Vector2D origin, TileKind2D[] tiles, bool oneWay)
    {
        Name = name; Width = width; Height = height; TileSize = tileSize; Origin = origin; Tiles = tiles; OneWay = oneWay;
    }

    /// <summary>The source layer's name (with ":oneway" appended for the one-way part of a layer).</summary>
    public string Name { get; }

    /// <summary>Width in tiles, 1 to 1024.</summary>
    public int Width { get; }

    /// <summary>Height in tiles, 1 to 1024.</summary>
    public int Height { get; }

    /// <summary>Tile edge length in metres.</summary>
    public float TileSize { get; }

    /// <summary>World position of the grid's bottom-left corner.</summary>
    public Vector2D Origin { get; }

    /// <summary>Tile kinds, row by row from the bottom row (x fastest), as <see cref="PhysicsWorld2D.AddTileGrid"/> takes them.</summary>
    public TileKind2D[] Tiles { get; }

    /// <summary>True for tiles that can be jumped up through and landed on from above.</summary>
    public bool OneWay { get; }
}

/// <summary>A shape placed on the map: a collision object or an entity, with the properties it carried.</summary>
public sealed class MapObject2D
{
    internal MapObject2D(string name, string type, MapShapeKind shape, MapBodyKind body, Vector2D position, float rotation, Vector2F size, float radius, Vector2F[] points)
    {
        Name = name; Type = type; Shape = shape; Body = body; Position = position; Rotation = rotation; Size = size; Radius = radius; Points = points;
    }

    /// <summary>The object's name; may be empty.</summary>
    public string Name { get; }

    /// <summary>The object's class or type string (Tiled's class, LDtk's entity identifier); may be empty.</summary>
    public string Type { get; }

    /// <summary>The geometry.</summary>
    public MapShapeKind Shape { get; }

    /// <summary>How the object takes part in the simulation.</summary>
    public MapBodyKind Body { get; }

    /// <summary>World position of the body: the centre of a box or circle, the origin of a polygon's or polyline's points.</summary>
    public Vector2D Position { get; }

    /// <summary>Counter-clockwise rotation in radians.</summary>
    public float Rotation { get; }

    /// <summary>Full width and height of a box.</summary>
    public Vector2F Size { get; }

    /// <summary>Radius of a circle.</summary>
    public float Radius { get; }

    /// <summary>Points of a polygon, chain or loop, relative to <see cref="Position"/>, before rotation.</summary>
    public Vector2F[] Points { get; }

    /// <summary>Density in kg/m² for a dynamic body (property <c>density</c>); default 1.</summary>
    public float Density { get; internal set; } = 1f;

    /// <summary>Friction (property <c>friction</c>); default 0.6.</summary>
    public float Friction { get; internal set; } = 0.6f;

    /// <summary>Restitution (property <c>restitution</c>); default 0.</summary>
    public float Restitution { get; internal set; }

    /// <summary>True when the shape is a one-way platform (property <c>oneway</c>): passable from below and landable from above.</summary>
    public bool OneWay { get; internal set; }

    /// <summary>Every custom property of the object as text, by name (booleans "true"/"false", numbers in invariant culture).</summary>
    public IReadOnlyDictionary<string, string> Properties { get; internal set; } = new Dictionary<string, string>();
}

/// <summary>A named point of the map (a spawn position or marker).</summary>
/// <param name="Name">The object's name.</param>
/// <param name="Type">Its class or entity identifier.</param>
/// <param name="Position">World position, metres, y up.</param>
/// <param name="Properties">Its custom properties as text.</param>
public sealed record MapPoint2D(string Name, string Type, Vector2D Position, IReadOnlyDictionary<string, string> Properties);

/// <summary>The bodies <see cref="Map2D.Apply"/> created.</summary>
public sealed class MapInstance2D
{
    internal MapInstance2D(Body[] tiles, Body[] objects) { TileBodies = tiles; ObjectBodies = objects; }

    /// <summary>One static body per entry of <see cref="Map2D.TileLayers"/>, in order.</summary>
    public Body[] TileBodies { get; }

    /// <summary>One body per entry of <see cref="Map2D.Objects"/>, in order.</summary>
    public Body[] ObjectBodies { get; }
}

/// <summary>A 2D level imported from Tiled or LDtk: collision tile grids, collision and entity shapes, and marker points, in metres with y up.</summary>
public sealed class Map2D
{
    internal Map2D(string name, Vector2D size, List<MapTileLayer> layers, List<MapObject2D> objects, List<MapPoint2D> points, List<InterchangeWarning> warnings)
    {
        Name = name; Size = size; TileLayers = layers; Objects = objects; Points = points; Warnings = warnings;
    }

    /// <summary>The map's or level's name.</summary>
    public string Name { get; }

    /// <summary>Width and height of the map in metres.</summary>
    public Vector2D Size { get; }

    /// <summary>Collision tile grids, in layer order (a layer with one-way tiles gives a second grid right after it).</summary>
    public IReadOnlyList<MapTileLayer> TileLayers { get; }

    /// <summary>Collision shapes and entities with a <c>body</c> property, in file order.</summary>
    public IReadOnlyList<MapObject2D> Objects { get; }

    /// <summary>Point markers (spawn positions and the like), in file order.</summary>
    public IReadOnlyList<MapPoint2D> Points { get; }

    /// <summary>Everything the importer left out or approximated.</summary>
    public IReadOnlyList<InterchangeWarning> Warnings { get; }

    /// <summary>
    /// Creates the map's bodies in a world: one static tile-grid body per tile layer (one-way grids get a one-way surface) and one body per object, in the order of the lists.
    /// </summary>
    /// <param name="world">The world to add to.</param>
    /// <param name="defaultFriction">Friction of tile grids, default 0.6.</param>
    /// <exception cref="ArgumentNullException"><paramref name="world"/> is null.</exception>
    public MapInstance2D Apply(PhysicsWorld2D world, float defaultFriction = 0.6f)
    {
        ArgumentNullException.ThrowIfNull(world);
        var tiles = new Body[TileLayers.Count];
        for (int i = 0; i < tiles.Length; i++)
        {
            var l = TileLayers[i];
            var grid = new TileGrid2D(l.Width, l.Height, l.Tiles, l.TileSize);
            var body = world.CreateBody(BodyType.Static, l.Origin);
            var def = new ShapeDef { Friction = defaultFriction };
            if (l.OneWay) { def.Surface = new SurfaceProperties { OneWayDirection = new Vector3F(0, 1, 0) }; }
            world.CreateTileGrid(body, grid, def);
            tiles[i] = body;
        }

        var objects = new Body[Objects.Count];
        for (int i = 0; i < objects.Length; i++)
        {
            var o = Objects[i];
            var type = o.Body == MapBodyKind.Dynamic ? BodyType.Dynamic : o.Body == MapBodyKind.Kinematic ? BodyType.Kinematic : BodyType.Static;
            var body = world.CreateBody(type, o.Position, o.Rotation);
            var def = new ShapeDef { Density = o.Density, Friction = o.Friction, Restitution = o.Restitution, IsSensor = o.Body == MapBodyKind.Sensor };
            if (o.OneWay) { def.Surface = new SurfaceProperties { OneWayDirection = new Vector3F(0, 1, 0) }; }
            switch (o.Shape)
            {
                case MapShapeKind.Box: world.CreateBox(body, o.Size, default, 0f, 0f, def); break;
                case MapShapeKind.Circle: world.CreateCircle(body, o.Radius, default, def); break;
                case MapShapeKind.Polygon: world.CreatePolygon(body, o.Points, 0f, def); break;
                case MapShapeKind.Chain: world.CreateChain(body, o.Points, false, true, def); break;
                default: world.CreateChain(body, o.Points, true, true, def); break;
            }

            objects[i] = body;
        }

        return new MapInstance2D(tiles, objects);
    }
}

/// <summary>Settings shared by <see cref="TiledImporter"/> and <see cref="LdtkImporter"/>.</summary>
public sealed class MapOptions
{
    /// <summary>Pixels per metre; a 16-pixel tile with 16 pixels per metre is one metre wide. Above 0. Default 16.</summary>
    public double PixelsPerMeter { get; set; } = 16.0;

    /// <summary>World position of the map's bottom-left corner. Default zero.</summary>
    public Vector2D Origin { get; set; }

    /// <summary>Returns the text of an external tileset file (Tiled's .tsx or .tsj) by the name the map gives, or null if the host does not have it. Tiles of an unavailable tileset count as solid.</summary>
    public Func<string, string?>? Tilesets { get; set; }

    /// <summary>Names of layers to treat as collision layers in addition to those marked by the <c>collision</c> property or named "collision". Compared without case.</summary>
    public IList<string> CollisionLayers { get; } = new List<string>();
}
