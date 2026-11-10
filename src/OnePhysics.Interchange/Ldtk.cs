using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>
/// Reads one level of an LDtk project (JSON) into a <see cref="Map2D"/>.
/// <b>Collision layers:</b> IntGrid layers named "collision" (or listed in <see cref="MapOptions.CollisionLayers"/>). The identifier of each IntGrid value picks the tile kind, compared without case, dashes, underscores or spaces:
/// solid; half or halfbottom; slopeupright or sloperight; slopeupleft or slopeleft; oneway, onewayplatform or platform (a solid tile on the one-way grid); none or empty. Any other value is solid, with a warning.
/// <b>Entities:</b> an entity with a string field <c>body</c> (static, dynamic, kinematic, sensor) becomes a body shaped by its box (or a circle when its boolean field <c>circle</c> is true); entities without one become <see cref="Map2D.Points"/>. The fields <c>density</c>, <c>friction</c>, <c>restitution</c> and <c>oneway</c> set the shape.
/// </summary>
public static class LdtkImporter
{
    private const string F = "LDtk";

    /// <summary>Imports a level of an LDtk project given as one JSON text.</summary>
    /// <param name="projectJson">The project file's text.</param>
    /// <param name="levelIdentifier">The level's identifier; null for the first level.</param>
    /// <param name="options">Settings; null for the defaults.</param>
    /// <exception cref="InterchangeException">The project is malformed, the level is not found or holds its layers in a separate file (use <see cref="ImportLevel"/>), or a layer is larger than 1024 × 1024 cells.</exception>
    public static Map2D Import(string projectJson, string? levelIdentifier = null, MapOptions? options = null)
    {
        options ??= new MapOptions();
        MapCore.CheckOptions(options, F);
        ArgumentNullException.ThrowIfNull(projectJson);
        var root = Js.Parse(projectJson, F);
        if (!root.IsObject) { throw root.Fail("the project must be a JSON object."); }
        var levels = root.ArrOpt("levels") ?? throw root.Fail("'levels' is required.");
        Js? chosen = null;
        for (int i = 0; i < levels.Length; i++)
        {
            var l = levels.At(i);
            if (levelIdentifier == null ? i == 0 : l.Str("identifier") == levelIdentifier) { chosen = l; break; }
        }

        if (chosen == null) { throw levels.Fail(levelIdentifier == null ? "the project has no levels." : "no level named '" + Io.Clip(levelIdentifier) + "'."); }
        return Build(root, chosen.Value, options);
    }

    /// <summary>Imports a level whose layers are stored in a separate level file.</summary>
    /// <param name="projectJson">The project file's text (for the layer definitions).</param>
    /// <param name="levelJson">The level file's text.</param>
    /// <param name="options">Settings; null for the defaults.</param>
    public static Map2D ImportLevel(string projectJson, string levelJson, MapOptions? options = null)
    {
        options ??= new MapOptions();
        MapCore.CheckOptions(options, F);
        ArgumentNullException.ThrowIfNull(projectJson); ArgumentNullException.ThrowIfNull(levelJson);
        return Build(Js.Parse(projectJson, F), Js.Parse(levelJson, F), options);
    }

    private static Map2D Build(Js project, Js level, MapOptions o)
    {
        var warnings = new List<InterchangeWarning>();
        if (!level.IsObject) { throw level.Fail("the level must be an object."); }
        string name = level.Str("identifier") ?? "";
        int wid = level.Need("pxWid").Int(), hei = level.Need("pxHei").Int();
        if (wid < 1 || hei < 1 || wid > 100_000_000 || hei > 100_000_000) { throw level.Fail("the level size is out of range."); }
        var instances = level.ArrOpt("layerInstances");
        if (instances == null) { throw level.Fail("the level has no layerInstances: it is stored in a separate file; use LdtkImporter.ImportLevel."); }
        double ppm = o.PixelsPerMeter;
        Vector2D W(double px, double py) => new(o.Origin.X + px / ppm, o.Origin.Y + (hei - py) / ppm);

        // Layer definitions give the identifier of each IntGrid value.
        var defs = new Dictionary<int, Dictionary<int, string>>();
        if (project.ObjOpt("defs") is { } d && d.ArrOpt("layers") is { } dl)
        {
            for (int i = 0; i < dl.Length; i++)
            {
                var ld = dl.At(i);
                var map = new Dictionary<int, string>();
                if (ld.ArrOpt("intGridValues") is { } iv)
                {
                    for (int k = 0; k < iv.Length && k < 4096; k++) { var v = iv.At(k); map[v.Need("value").Int()] = v.Str("identifier") ?? ""; }
                }

                defs[ld.Need("uid").Int()] = map;
            }
        }

        var layers = new List<MapTileLayer>(); var objects = new List<MapObject2D>(); var points = new List<MapPoint2D>();
        // LDtk lists layers top to bottom; order does not matter for physics, so file order is kept.
        for (int li = 0; li < instances.Value.Length; li++)
        {
            var l = instances.Value.At(li);
            string type = l.Str("__type") ?? "";
            string ident = l.Str("__identifier") ?? "";
            double offX = l.Num("__pxTotalOffsetX", 0), offY = l.Num("__pxTotalOffsetY", 0);
            if (type == "IntGrid")
            {
                if (!MapCore.IsCollisionName(ident, o)) { continue; }
                BuildIntGrid(l, ident, defs, offX, offY, o, W, warnings, layers);
            }
            else if (type == "Entities") { BuildEntities(l, offX, offY, ppm, W, warnings, objects, points); }
        }

        return new Map2D(name, new Vector2D(wid / ppm, hei / ppm), layers, objects, points, warnings);
    }

    private static string Normalize(string s) => new(s.Where(c => c is not ('-' or '_' or ' ')).Select(char.ToLowerInvariant).ToArray());

    private static (TileKind2D Kind, bool OneWay, bool Known) KindOf(string identifier) => Normalize(identifier) switch
    {
        "solid" => (TileKind2D.Solid, false, true),
        "half" or "halftile" or "halfbottom" => (TileKind2D.HalfBottom, false, true),
        "slopeupright" or "sloperight" => (TileKind2D.SlopeUpRight, false, true),
        "slopeupleft" or "slopeleft" => (TileKind2D.SlopeUpLeft, false, true),
        "oneway" or "onewayplatform" or "platform" => (TileKind2D.Solid, true, true),
        "none" or "empty" => (TileKind2D.Empty, false, true),
        _ => (TileKind2D.Solid, false, false),
    };

    private static void BuildIntGrid(Js l, string ident, Dictionary<int, Dictionary<int, string>> defs, double offX, double offY, MapOptions o, Func<double, double, Vector2D> world, List<InterchangeWarning> warnings, List<MapTileLayer> layers)
    {
        int cw = l.Need("__cWid").Int(), ch = l.Need("__cHei").Int(), gs = l.Need("__gridSize").Int();
        if (cw < 1 || ch < 1 || cw > 1024 || ch > 1024 || (long)cw * ch > TileGrid2D.MaxTiles) { throw l.Fail("the layer is " + cw + " x " + ch + " cells; a collision grid holds 1 to 1024 per side and at most " + TileGrid2D.MaxTiles + " tiles."); }
        if (gs < 1 || gs > 1_000_000) { throw l.Need("__gridSize").Fail("the grid size is out of range."); }
        var csv = l.ArrOpt("intGridCsv") ?? throw l.Fail("'intGridCsv' is required on an IntGrid layer.");
        if (csv.Length != (long)cw * ch) { throw csv.Fail("expected " + (long)cw * ch + " cells, found " + csv.Length + "."); }
        defs.TryGetValue(l.Int("layerDefUid", -1), out var names);
        var solid = new TileKind2D[cw * ch]; var one = new TileKind2D[cw * ch];
        bool anySolid = false, anyOne = false;
        var warned = new HashSet<int>();
        for (int i = 0; i < csv.Length; i++)
        {
            int v = csv.At(i).Int();
            if (v == 0) { continue; }
            TileKind2D kind; bool oneWay;
            if (names != null && names.TryGetValue(v, out var id) && id.Length > 0)
            {
                var k = KindOf(id);
                if (!k.Known && warned.Add(v)) { warnings.Add(new InterchangeWarning("intgrid-value", l.Path, "IntGrid value " + v + " ('" + Io.Clip(id) + "') has no known tile kind; it is imported as solid.")); }
                (kind, oneWay) = (k.Kind, k.OneWay);
            }
            else
            {
                if (v != 1 && warned.Add(v)) { warnings.Add(new InterchangeWarning("intgrid-value", l.Path, "IntGrid value " + v + " has no identifier; it is imported as solid.")); }
                (kind, oneWay) = (TileKind2D.Solid, false);
            }

            if (kind == TileKind2D.Empty) { continue; }
            int row = i / cw, col = i % cw;
            int outIndex = (ch - 1 - row) * cw + col;
            if (oneWay) { one[outIndex] = kind; anyOne = true; } else { solid[outIndex] = kind; anySolid = true; }
        }

        var origin = world(offX, offY + (double)ch * gs);
        float size = (float)(gs / o.PixelsPerMeter);
        if (anySolid) { layers.Add(new MapTileLayer(ident, cw, ch, size, origin, solid, false)); }
        if (anyOne) { layers.Add(new MapTileLayer(ident + ":oneway", cw, ch, size, origin, one, true)); }
    }

    private static string? FieldText(Js f)
    {
        var v = f.Get("__value");
        if (v == null) { return null; }
        return v.Value.E.ValueKind switch
        {
            JsonValueKind.True => "true", JsonValueKind.False => "false",
            JsonValueKind.Number => v.Value.E.TryGetDouble(out double d) && double.IsFinite(d) ? d.ToString("R", CultureInfo.InvariantCulture) : null,
            JsonValueKind.String => v.Value.Str(),
            _ => null,
        };
    }

    private static void BuildEntities(Js l, double offX, double offY, double ppm, Func<double, double, Vector2D> world, List<InterchangeWarning> warnings, List<MapObject2D> objects, List<MapPoint2D> points)
    {
        if (l.ArrOpt("entityInstances") is not { } ents) { return; }
        if (ents.Length > 1_000_000) { throw ents.Fail("more than a million entities."); }
        for (int i = 0; i < ents.Length; i++)
        {
            var e = ents.At(i);
            string ident = e.Str("__identifier") ?? "";
            var px = e.Nums("px", 2) ?? throw e.Fail("'px' is required.");
            var pivot = e.Nums("__pivot", 2) ?? new[] { 0.0, 0.0 };
            double w = e.Num("width", 0), h = e.Num("height", 0);
            var props = new Dictionary<string, string>(StringComparer.Ordinal);
            if (e.ArrOpt("fieldInstances") is { } fields)
            {
                for (int k = 0; k < fields.Length && k < 4096; k++)
                {
                    var f = fields.At(k);
                    if (FieldText(f) is { } text) { props[f.Str("__identifier") ?? ""] = text; }
                }
            }

            double x = px[0] + offX, y = px[1] + offY;
            var kind = MapCore.BodyKind(props, F, e.Path);
            if (kind == null) { points.Add(new MapPoint2D(ident, ident, world(x, y), props)); continue; }
            if (!(w > 0 && h > 0)) { warnings.Add(new InterchangeWarning("degenerate-shape", e.Path, "an entity with a body property needs a width and height; it is left out.")); continue; }
            // The pivot is where px points inside the entity box; the centre is half a box from the top-left corner.
            double cx = x + (0.5 - pivot[0]) * w, cy = y + (0.5 - pivot[1]) * h;
            MapObject2D made = MapCore.IsTrue(props, "circle")
                ? new MapObject2D(ident, ident, MapShapeKind.Circle, kind.Value, world(cx, cy), 0f, default, (float)(Math.Min(w, h) / 2 / ppm), Array.Empty<Vector2F>())
                : new MapObject2D(ident, ident, MapShapeKind.Box, kind.Value, world(cx, cy), 0f, new Vector2F((float)(w / ppm), (float)(h / ppm)), 0, Array.Empty<Vector2F>());
            MapCore.Material(made, props, F, e.Path);
            objects.Add(made);
        }
    }

    /// <summary>Imports a level from streams of UTF-8 text.</summary>
    public static Map2D Import(Stream project, string? levelIdentifier = null, MapOptions? options = null) => Import(Io.ReadAll(project, F, Io.MaxChars), levelIdentifier, options);
}
