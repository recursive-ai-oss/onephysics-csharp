using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text.Json;
using System.Xml.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>
/// Reads Tiled maps (TMX XML and TMJ JSON, orthogonal, finite) into a <see cref="Map2D"/>.
/// <b>Collision layers:</b> a tile layer or object layer is a collision layer when it has the boolean property <c>collision</c>, is named "collision", or is listed in <see cref="MapOptions.CollisionLayers"/>; other tile layers are visual and skipped.
/// <b>Tiles:</b> a tile's collision comes from its string property <c>kind</c> (solid, half, slope-right, slope-left, none), else from its collision editor shapes (a full-tile rectangle is solid, a bottom-half rectangle a half tile, a right triangle a slope), else the tile is solid.
/// Tiles with other collision shapes become static polygon objects. A boolean tile property <c>oneway</c> (or the same property on the layer) puts tiles on a separate one-way grid. Horizontal flips mirror slopes; other flips turn a tile into polygon objects.
/// <b>Objects:</b> rectangles, ellipses, polygons and polylines on a collision layer become static shapes; on any object layer an object with a <c>body</c> property (static, dynamic, kinematic, sensor) becomes a body of that kind; points become <see cref="Map2D.Points"/>. <c>density</c>, <c>friction</c>, <c>restitution</c> and <c>oneway</c> properties set the shape.
/// </summary>
public static class TiledImporter
{
    private const string F = "Tiled";

    private sealed class TObj
    {
        public double X, Y, W, H, Rot;
        public string Name = "", Type = "";
        public bool Point, Ellipse, HasPoly, HasLine;
        public List<(double X, double Y)> Poly = new();
        public int Gid;
        public Dictionary<string, string> Props = new(StringComparer.Ordinal);
        public string Where = "";
    }

    private sealed class TTile
    {
        public Dictionary<string, string> Props = new(StringComparer.Ordinal);
        public bool HasCollisionGroup;
        public List<TObj> Objects = new();
    }

    private sealed class TSet
    {
        public long FirstGid;
        public int TileCount = int.MaxValue;
        public bool Missing;
        public string Source = "";
        public Dictionary<int, TTile> Tiles = new();
    }

    private sealed class TLayer
    {
        public bool IsTile, IsObject;
        public string Name = "", Where = "";
        public int W, H;
        public uint[] Data = Array.Empty<uint>();
        public List<TObj> Objects = new();
        public Dictionary<string, string> Props = new(StringComparer.Ordinal);
        public double OffsetX, OffsetY;
        public bool InCollisionGroup;
    }

    private sealed record TileClass(TileKind2D Kind, bool OneWay, List<List<(double X, double Y)>>? Fallback);

    private sealed class TMap
    {
        public string Name = "";
        public int W, H, TW, TH;
        public List<TLayer> Layers = new();
        public List<TSet> Sets = new();
    }

    /// <summary>Imports a TMX (XML) or TMJ (JSON) map; the format is recognised from the first character.</summary>
    /// <exception cref="InterchangeException">The map is malformed or not supported (an isometric or infinite map, non-square tiles, a grid larger than 1024 × 1024); the message names the element.</exception>
    public static Map2D Import(string text, MapOptions? options = null)
    {
        options ??= new MapOptions();
        MapCore.CheckOptions(options, F);
        ArgumentNullException.ThrowIfNull(text);
        var warnings = new List<InterchangeWarning>();
        string trimmed = text.TrimStart();
        var map = trimmed.StartsWith('<') ? ReadTmx(text, options, warnings) : ReadTmj(text, options, warnings);
        return ToMap(map, options, warnings);
    }

    /// <summary>Imports a map from a stream of UTF-8 text.</summary>
    public static Map2D Import(Stream stream, MapOptions? options = null) => Import(Io.ReadAll(stream, F, Io.MaxChars), options);

    // ---------------------------------------------------------------- TMJ

    private static TMap ReadTmj(string text, MapOptions options, List<InterchangeWarning> warnings)
    {
        var root = Js.Parse(text, F);
        if (!root.IsObject) { throw root.Fail("the map must be a JSON object."); }
        var m = new TMap { Name = "" };
        CheckMapHeader(root.Str("orientation") ?? "orthogonal", root.Bool("infinite", false), root.Path);
        m.W = root.Need("width").Int(); m.H = root.Need("height").Int(); m.TW = root.Need("tilewidth").Int(); m.TH = root.Need("tileheight").Int();
        CheckSize(m, root.Path);
        if (root.ArrOpt("tilesets") is { } sets)
        {
            for (int i = 0; i < sets.Length; i++) { m.Sets.Add(ReadTmjSet(sets.At(i), options, warnings)); }
        }

        if (root.ArrOpt("layers") is { } layers) { ReadTmjLayers(layers, m, 0, 0, 0, false, 0); }
        return m;
    }

    private static void ReadTmjLayers(Js layers, TMap m, double ox, double oy, int depth, bool inCollision, int count)
    {
        if (depth > 16) { throw layers.Fail("layer groups are nested more than 16 deep."); }
        for (int i = 0; i < layers.Length; i++)
        {
            var l = layers.At(i);
            string type = l.Str("type") ?? "";
            var props = ReadTmjProps(l);
            double lx = ox + l.Num("offsetx", 0), ly = oy + l.Num("offsety", 0);
            string name = l.Str("name") ?? "";
            if (type == "group")
            {
                bool coll = inCollision || MapCore.IsTrue(props, "collision");
                if (l.ArrOpt("layers") is { } kids) { ReadTmjLayers(kids, m, lx, ly, depth + 1, coll, count); }
                continue;
            }

            if (m.Layers.Count >= 4096) { throw l.Fail("more than 4096 layers."); }
            var layer = new TLayer { Name = name, Where = l.Path + " (" + Io.Clip(name) + ")", Props = props, OffsetX = lx, OffsetY = ly, InCollisionGroup = inCollision };
            if (type == "tilelayer")
            {
                layer.IsTile = true;
                if (l.Get("chunks") != null) { throw l.Fail("infinite maps (chunked layers) are not supported."); }
                layer.W = l.Need("width").Int(); layer.H = l.Need("height").Int();
                if (layer.W != m.W || layer.H != m.H) { throw l.Fail("a tile layer must be as large as the map (" + m.W + " x " + m.H + ")."); }
                var data = l.Need("data");
                layer.Data = data.IsArray ? ReadGidArray(data, (long)m.W * m.H) : DecodeData(data.Str(), l.Str("encoding") ?? "", l.Str("compression") ?? "", (long)m.W * m.H, l);
            }
            else if (type == "objectgroup")
            {
                layer.IsObject = true;
                if (l.ArrOpt("objects") is { } objs)
                {
                    if (objs.Length > 1_000_000) { throw objs.Fail("more than a million objects."); }
                    for (int k = 0; k < objs.Length; k++) { layer.Objects.Add(ReadTmjObject(objs.At(k))); }
                }
            }
            else { continue; }
            m.Layers.Add(layer);
        }
    }

    private static uint[] ReadGidArray(Js data, long expected)
    {
        if (data.Length != expected) { throw data.Fail("expected " + expected + " tile ids, found " + data.Length + "."); }
        var r = new uint[expected];
        for (int i = 0; i < r.Length; i++)
        {
            var v = data.At(i);
            if (v.E.ValueKind != JsonValueKind.Number || !v.E.TryGetUInt32(out uint g)) { throw v.Fail("expected a tile id (0 to 2^32-1)."); }
            r[i] = g;
        }

        return r;
    }

    private static uint[] DecodeData(string text, string encoding, string compression, long expected, Js where)
    {
        if (encoding != "base64") { throw where.Fail("string tile data must be base64 encoded."); }
        byte[] raw;
        try { raw = Convert.FromBase64String(text.Trim()); }
        catch (FormatException) { throw where.Fail("the tile data is not valid base64."); }
        return Inflate(raw, compression, expected, (m) => where.Fail(m));
    }

    private static uint[] Inflate(byte[] raw, string compression, long expected, Func<string, InterchangeException> fail)
    {
        long bytes = expected * 4;
        byte[] data;
        if (compression == "" ) { data = raw; }
        else if (compression is "zlib" or "gzip")
        {
            using var input = new MemoryStream(raw);
            using Stream z = compression == "zlib" ? new ZLibStream(input, CompressionMode.Decompress) : new GZipStream(input, CompressionMode.Decompress);
            var buf = new byte[bytes + 1];
            int total = 0, n;
            try { while (total < buf.Length && (n = z.Read(buf, total, buf.Length - total)) > 0) { total += n; } }
            catch (InvalidDataException) { throw fail("the compressed tile data is corrupt."); }
            data = buf.AsSpan(0, total).ToArray();
        }
        else { throw fail("tile data compression '" + Io.Clip(compression) + "' is not supported (use none, zlib or gzip)."); }
        if (data.Length != bytes) { throw fail("the tile data holds " + data.Length + " bytes but the layer needs " + bytes + "."); }
        var r = new uint[expected];
        for (int i = 0; i < r.Length; i++) { r[i] = System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan(4 * i)); }
        return r;
    }

    private static Dictionary<string, string> ReadTmjProps(Js owner)
    {
        var d = new Dictionary<string, string>(StringComparer.Ordinal);
        if (owner.ArrOpt("properties") is not { } arr) { return d; }
        for (int i = 0; i < arr.Length && i < 4096; i++)
        {
            var p = arr.At(i);
            string name = p.Str("name") ?? throw p.Fail("'name' is required.");
            if (p.Get("value") is not { } v) { continue; }
            string? s = v.E.ValueKind switch
            {
                JsonValueKind.True => "true", JsonValueKind.False => "false",
                JsonValueKind.Number => v.Num().ToString("R", CultureInfo.InvariantCulture),
                JsonValueKind.String => v.Str(),
                _ => null,
            };
            if (s != null) { d[name] = s; }
        }

        return d;
    }

    private static TObj ReadTmjObject(Js o)
    {
        var t = new TObj
        {
            Where = o.Path, Name = o.Str("name") ?? "", Type = o.Str("class") ?? o.Str("type") ?? "", X = o.Num("x", 0), Y = o.Num("y", 0), W = o.Num("width", 0), H = o.Num("height", 0), Rot = o.Num("rotation", 0),
            Point = o.Bool("point", false), Ellipse = o.Bool("ellipse", false), Gid = (int)Math.Min(int.MaxValue, o.Num("gid", 0)), Props = ReadTmjProps(o),
        };
        var poly = o.ArrOpt("polygon"); var line = o.ArrOpt("polyline");
        var pts = poly ?? line;
        if (pts != null)
        {
            if (pts.Value.Length > 100_000) { throw pts.Value.Fail("more than 100000 points."); }
            t.HasPoly = poly != null; t.HasLine = line != null;
            for (int i = 0; i < pts.Value.Length; i++) { var p = pts.Value.At(i); t.Poly.Add((p.Need("x").Num(), p.Need("y").Num())); }
        }

        return t;
    }

    private static TSet ReadTmjSet(Js s, MapOptions options, List<InterchangeWarning> warnings)
    {
        var set = new TSet { FirstGid = s.Need("firstgid").Int() };
        if (set.FirstGid < 1) { throw s.Need("firstgid").Fail("firstgid must be at least 1."); }
        if (s.Str("source") is { } src)
        {
            set.Source = src;
            string? ext = options.Tilesets?.Invoke(src);
            if (ext == null) { set.Missing = true; warnings.Add(new InterchangeWarning("tileset-missing", s.Path, "the external tileset '" + Io.Clip(src) + "' was not provided (MapOptions.Tilesets); its tiles are treated as solid.")); return set; }
            string t = ext.TrimStart();
            if (t.StartsWith('<')) { ReadTsx(Io.LoadXml(ext, F).Root!, set); }
            else { ReadTsj(Js.Parse(ext, F), set); }
            return set;
        }

        ReadTsj(s, set);
        return set;
    }

    private static void ReadTsj(Js s, TSet set)
    {
        set.TileCount = s.Int("tilecount", int.MaxValue);
        if (s.ArrOpt("tiles") is not { } tiles) { return; }
        for (int i = 0; i < tiles.Length; i++)
        {
            var t = tiles.At(i);
            var tile = new TTile { Props = ReadTmjProps(t) };
            if (t.ObjOpt("objectgroup") is { } og)
            {
                tile.HasCollisionGroup = true;
                if (og.ArrOpt("objects") is { } objs) { for (int k = 0; k < objs.Length && k < 4096; k++) { tile.Objects.Add(ReadTmjObject(objs.At(k))); } }
            }

            set.Tiles[t.Need("id").Int()] = tile;
        }
    }

    // ---------------------------------------------------------------- TMX

    private static TMap ReadTmx(string text, MapOptions options, List<InterchangeWarning> warnings)
    {
        var doc = Io.LoadXml(text, F);
        var root = doc.Root;
        if (root == null || root.Name.LocalName != "map") { throw new InterchangeException(F, "document", "the root element must be <map>."); }
        string w = Io.Where(root);
        CheckMapHeader((string?)root.Attribute("orientation") ?? "orthogonal", (string?)root.Attribute("infinite") == "1", w);
        var m = new TMap
        {
            W = Int(root, "width", w), H = Int(root, "height", w), TW = Int(root, "tilewidth", w), TH = Int(root, "tileheight", w),
        };
        CheckSize(m, w);
        foreach (var ts in root.Elements("tileset")) { m.Sets.Add(ReadTmxSet(ts, options, warnings)); }
        ReadTmxLayers(root, m, 0, 0, 0, false);
        return m;
    }

    private static int Int(XElement e, string a, string where)
    {
        if (!int.TryParse((string?)e.Attribute(a), NumberStyles.Integer, CultureInfo.InvariantCulture, out int v)) { throw new InterchangeException(F, where, "attribute '" + a + "' must be an integer."); }
        return v;
    }

    private static double Dbl(XElement e, string a, double fallback, string where)
    {
        string? s = (string?)e.Attribute(a);
        return s == null ? fallback : Io.Number(s, F, where, a);
    }

    private static Dictionary<string, string> ReadTmxProps(XElement owner)
    {
        var d = new Dictionary<string, string>(StringComparer.Ordinal);
        var ps = owner.Element("properties");
        if (ps == null) { return d; }
        int n = 0;
        foreach (var p in ps.Elements("property"))
        {
            if (++n > 4096) { break; }
            string? name = (string?)p.Attribute("name");
            if (name == null) { throw new InterchangeException(F, Io.Where(p), "'name' is required."); }
            d[name] = (string?)p.Attribute("value") ?? p.Value;
        }

        return d;
    }

    private static void ReadTmxLayers(XElement parent, TMap m, double ox, double oy, int depth, bool inCollision)
    {
        if (depth > 16) { throw new InterchangeException(F, Io.Where(parent), "layer groups are nested more than 16 deep."); }
        foreach (var l in parent.Elements())
        {
            string kind = l.Name.LocalName;
            if (kind is not ("layer" or "objectgroup" or "group")) { continue; }
            string w = Io.Where(l);
            var props = ReadTmxProps(l);
            double lx = ox + Dbl(l, "offsetx", 0, w), ly = oy + Dbl(l, "offsety", 0, w);
            if (kind == "group") { ReadTmxLayers(l, m, lx, ly, depth + 1, inCollision || MapCore.IsTrue(props, "collision")); continue; }
            if (m.Layers.Count >= 4096) { throw new InterchangeException(F, w, "more than 4096 layers."); }
            var layer = new TLayer { Name = (string?)l.Attribute("name") ?? "", Where = w, Props = props, OffsetX = lx, OffsetY = ly, InCollisionGroup = inCollision };
            if (kind == "layer")
            {
                layer.IsTile = true;
                layer.W = Int(l, "width", w); layer.H = Int(l, "height", w);
                if (layer.W != m.W || layer.H != m.H) { throw new InterchangeException(F, w, "a tile layer must be as large as the map (" + m.W + " x " + m.H + ")."); }
                var data = l.Element("data") ?? throw new InterchangeException(F, w, "<data> is required.");
                if (data.Element("chunk") != null) { throw new InterchangeException(F, w, "infinite maps (chunked layers) are not supported."); }
                layer.Data = ReadTmxData(data, (long)m.W * m.H);
            }
            else
            {
                layer.IsObject = true;
                int n = 0;
                foreach (var o in l.Elements("object"))
                {
                    if (++n > 1_000_000) { throw new InterchangeException(F, w, "more than a million objects."); }
                    layer.Objects.Add(ReadTmxObject(o));
                }
            }

            m.Layers.Add(layer);
        }
    }

    private static uint[] ReadTmxData(XElement data, long expected)
    {
        string w = Io.Where(data);
        string enc = (string?)data.Attribute("encoding") ?? "";
        string compression = (string?)data.Attribute("compression") ?? "";
        if (enc == "csv")
        {
            var parts = data.Value.Split(CsvSeparators, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != expected) { throw new InterchangeException(F, w, "expected " + expected + " tile ids, found " + parts.Length + "."); }
            var r = new uint[expected];
            for (int i = 0; i < r.Length; i++) { if (!uint.TryParse(parts[i], NumberStyles.None, CultureInfo.InvariantCulture, out r[i])) { throw new InterchangeException(F, w, "tile id '" + Io.Clip(parts[i]) + "' is not a number."); } }
            return r;
        }

        if (enc == "base64")
        {
            byte[] raw;
            try { raw = Convert.FromBase64String(data.Value.Trim()); }
            catch (FormatException) { throw new InterchangeException(F, w, "the tile data is not valid base64."); }
            return Inflate(raw, compression, expected, m => new InterchangeException(F, w, m));
        }

        if (enc == "" && data.Elements("tile").Any())
        {
            var gids = data.Elements("tile").Select(t => (string?)t.Attribute("gid")).ToList();
            if (gids.Count != expected) { throw new InterchangeException(F, w, "expected " + expected + " <tile> elements, found " + gids.Count + "."); }
            var r = new uint[expected];
            for (int i = 0; i < r.Length; i++) { if (!uint.TryParse(gids[i] ?? "0", NumberStyles.None, CultureInfo.InvariantCulture, out r[i])) { throw new InterchangeException(F, w, "tile gid '" + Io.Clip(gids[i] ?? "") + "' is not a number."); } }
            return r;
        }

        throw new InterchangeException(F, w, "tile data encoding must be csv or base64.");
    }

    private static TObj ReadTmxObject(XElement o)
    {
        string w = Io.Where(o);
        var t = new TObj
        {
            Where = w, Name = (string?)o.Attribute("name") ?? "", Type = (string?)o.Attribute("class") ?? (string?)o.Attribute("type") ?? "", X = Dbl(o, "x", 0, w), Y = Dbl(o, "y", 0, w), W = Dbl(o, "width", 0, w), H = Dbl(o, "height", 0, w),
            Rot = Dbl(o, "rotation", 0, w), Point = o.Element("point") != null, Ellipse = o.Element("ellipse") != null, Props = ReadTmxProps(o),
        };
        if (uint.TryParse((string?)o.Attribute("gid"), NumberStyles.None, CultureInfo.InvariantCulture, out uint gid)) { t.Gid = (int)Math.Min(int.MaxValue, gid); }
        var poly = o.Element("polygon"); var line = o.Element("polyline");
        var pe = poly ?? line;
        if (pe != null)
        {
            t.HasPoly = poly != null; t.HasLine = line != null;
            var parts = ((string?)pe.Attribute("points") ?? "").Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length > 100_000) { throw new InterchangeException(F, w, "more than 100000 points."); }
            foreach (var part in parts)
            {
                var xy = part.Split(',');
                if (xy.Length != 2) { throw new InterchangeException(F, w, "points must be 'x,y' pairs, found '" + Io.Clip(part) + "'."); }
                t.Poly.Add((Io.Number(xy[0], F, w, "points"), Io.Number(xy[1], F, w, "points")));
            }
        }

        return t;
    }

    private static TSet ReadTmxSet(XElement ts, MapOptions options, List<InterchangeWarning> warnings)
    {
        string w = Io.Where(ts);
        var set = new TSet { FirstGid = Int(ts, "firstgid", w) };
        if (set.FirstGid < 1) { throw new InterchangeException(F, w, "firstgid must be at least 1."); }
        if ((string?)ts.Attribute("source") is { } src)
        {
            set.Source = src;
            string? ext = options.Tilesets?.Invoke(src);
            if (ext == null) { set.Missing = true; warnings.Add(new InterchangeWarning("tileset-missing", w, "the external tileset '" + Io.Clip(src) + "' was not provided (MapOptions.Tilesets); its tiles are treated as solid.")); return set; }
            if (ext.TrimStart().StartsWith('<')) { ReadTsx(Io.LoadXml(ext, F).Root!, set); } else { ReadTsj(Js.Parse(ext, F), set); }
            return set;
        }

        ReadTsx(ts, set);
        return set;
    }

    private static void ReadTsx(XElement ts, TSet set)
    {
        if (int.TryParse((string?)ts.Attribute("tilecount"), NumberStyles.Integer, CultureInfo.InvariantCulture, out int tc)) { set.TileCount = tc; }
        foreach (var t in ts.Elements("tile"))
        {
            string w = Io.Where(t);
            var tile = new TTile { Props = ReadTmxProps(t) };
            if (t.Element("objectgroup") is { } og)
            {
                tile.HasCollisionGroup = true;
                foreach (var o in og.Elements("object")) { if (tile.Objects.Count < 4096) { tile.Objects.Add(ReadTmxObject(o)); } }
            }

            set.Tiles[Int(t, "id", w)] = tile;
        }
    }

    private static void CheckMapHeader(string orientation, bool infinite, string where)
    {
        if (orientation != "orthogonal") { throw new InterchangeException(F, where, "only orthogonal maps are supported, found '" + Io.Clip(orientation) + "'."); }
        if (infinite) { throw new InterchangeException(F, where, "infinite maps are not supported; export the map with a fixed size."); }
    }

    private static void CheckSize(TMap m, string where)
    {
        if (m.W < 1 || m.H < 1 || m.W > 1024 || m.H > 1024 || (long)m.W * m.H > TileGrid2D.MaxTiles) { throw new InterchangeException(F, where, "the map is " + m.W + " x " + m.H + " tiles; a collision grid holds 1 to 1024 per side and at most " + TileGrid2D.MaxTiles + " tiles."); }
        if (m.TW < 1 || m.TW != m.TH || m.TW > 1_000_000) { throw new InterchangeException(F, where, "tiles must be square (tilewidth " + m.TW + ", tileheight " + m.TH + "); collision grids have one tile size."); }
    }

    // ---------------------------------------------------------------- conversion

    private static readonly char[] CsvSeparators = { ',', '\n', '\r', ' ', '\t' };

    private const uint FlipH = 0x80000000u, FlipV = 0x40000000u, FlipD = 0x20000000u, FlipHex = 0x10000000u;

    private static Map2D ToMap(TMap m, MapOptions o, List<InterchangeWarning> warnings)
    {
        double ppm = o.PixelsPerMeter;
        double mapHeightPx = (double)m.H * m.TH;
        Vector2D W(double px, double py) => new(o.Origin.X + px / ppm, o.Origin.Y + (mapHeightPx - py) / ppm);
        var layers = new List<MapTileLayer>();
        var objects = new List<MapObject2D>();
        var points = new List<MapPoint2D>();
        var tileCache = new Dictionary<long, TileClass>();
        foreach (var l in m.Layers)
        {
            bool collision = l.InCollisionGroup || MapCore.IsTrue(l.Props, "collision") || MapCore.IsCollisionName(l.Name, o);
            if (l.IsTile)
            {
                if (!collision) { continue; }
                BuildTileLayer(l, m, o, warnings, tileCache, layers, objects, W);
            }
            else if (l.IsObject) { BuildObjects(l, m, o, collision, warnings, objects, points, W); }
        }

        return new Map2D("", new Vector2D(m.W * (double)m.TW / ppm, mapHeightPx / ppm), layers, objects, points, warnings);
    }

    private static void BuildTileLayer(TLayer l, TMap m, MapOptions o, List<InterchangeWarning> warnings, Dictionary<long, TileClass> cache,
        List<MapTileLayer> layers, List<MapObject2D> objects, Func<double, double, Vector2D> world)
    {
        double ppm = o.PixelsPerMeter;
        var solid = new TileKind2D[m.W * m.H]; var oneway = new TileKind2D[m.W * m.H];
        bool anySolid = false, anyOne = false;
        bool layerOneWay = MapCore.IsTrue(l.Props, "oneway");
        var missingReported = new HashSet<int>();
        for (int row = 0; row < m.H; row++)
        {
            for (int col = 0; col < m.W; col++)
            {
                uint raw = l.Data[row * m.W + col];
                if (raw == 0) { continue; }
                uint flags = raw & (FlipH | FlipV | FlipD | FlipHex);
                long gid = raw & ~(FlipH | FlipV | FlipD | FlipHex);
                if (gid == 0) { continue; }
                int si = -1;
                for (int i = 0; i < m.Sets.Count; i++) { if (m.Sets[i].FirstGid <= gid && (si < 0 || m.Sets[i].FirstGid > m.Sets[si].FirstGid)) { si = i; } }
                var set = si >= 0 ? m.Sets[si] : null;
                int local = set == null ? -1 : (int)(gid - set.FirstGid);
                if (set != null && local >= set.TileCount) { set = null; }
                if (set == null && gid != 0) { if (si < 0 && missingReported.Add(-1)) { warnings.Add(new InterchangeWarning("no-tileset", l.Where, "a tile id belongs to no tileset; it is treated as solid.")); } }
                long key = ((long)(si + 1) << 40) ^ ((long)(local + 1) << 8) ^ (flags >> 28);
                if (!cache.TryGetValue(key, out var cls))
                {
                    cls = Classify(set != null && !set.Missing && set.Tiles.TryGetValue(local, out var td) ? td : null, flags, m.TW, m.TH, warnings, l.Where);
                    cache[key] = cls;
                }

                bool one = layerOneWay || cls.OneWay;
                int outIndex = (m.H - 1 - row) * m.W + col;
                if (cls.Fallback != null)
                {
                    foreach (var poly in cls.Fallback)
                    {
                        double bx = col * (double)m.TW, by = row * (double)m.TH;
                        var wp = poly.Select(p => world(bx + p.X, by + p.Y)).ToList();
                        var rel = wp.Select(p => new Vector2D(p.X - wp[0].X, p.Y - wp[0].Y)).ToList();
                        var po = MapCore.PolygonObject("", "tile", MapBodyKind.Static, wp[0], 0f, rel, warnings, l.Where);
                        if (po != null) { po.OneWay = one; objects.Add(po); }
                    }

                    continue;
                }

                if (cls.Kind == TileKind2D.Empty) { continue; }
                if (one) { oneway[outIndex] = cls.Kind; anyOne = true; } else { solid[outIndex] = cls.Kind; anySolid = true; }
            }
        }

        var origin2 = world(0, (double)m.H * m.TH);
        float size = (float)(m.TW / ppm);
        if (anySolid) { layers.Add(new MapTileLayer(l.Name, m.W, m.H, size, origin2, solid, false)); }
        if (anyOne) { layers.Add(new MapTileLayer(l.Name + ":oneway", m.W, m.H, size, origin2, oneway, true)); }
    }

    private static TileClass Classify(TTile? tile, uint flags, int tw, int th, List<InterchangeWarning> warnings, string where)
    {
        bool oneway = tile != null && MapCore.IsTrue(tile.Props, "oneway");
        bool h = (flags & FlipH) != 0, v = (flags & FlipV) != 0, d = (flags & FlipD) != 0;
        TileKind2D kind;
        if (tile != null && tile.Props.TryGetValue("kind", out var k))
        {
            kind = k switch
            {
                "solid" => TileKind2D.Solid, "half" => TileKind2D.HalfBottom, "slope-right" => TileKind2D.SlopeUpRight, "slope-left" => TileKind2D.SlopeUpLeft, "none" or "empty" => TileKind2D.Empty,
                _ => throw new InterchangeException(F, where, "tile property 'kind' must be solid, half, slope-right, slope-left or none, found '" + Io.Clip(k) + "'."),
            };
        }
        else if (tile == null || !tile.HasCollisionGroup) { kind = TileKind2D.Solid; }
        else if (tile.Objects.Count == 0) { kind = TileKind2D.Empty; }
        else if (tile.Objects.Count == 1) { kind = ShapeKind(tile.Objects[0], tw, th); }
        else { kind = (TileKind2D)255; }

        if (kind == TileKind2D.Empty) { return new TileClass(kind, oneway, null); }
        if (kind != (TileKind2D)255)
        {
            if (!v && !d)
            {
                if (h) { kind = kind == TileKind2D.SlopeUpRight ? TileKind2D.SlopeUpLeft : kind == TileKind2D.SlopeUpLeft ? TileKind2D.SlopeUpRight : kind; }
                return new TileClass(kind, oneway, null);
            }

            if (kind == TileKind2D.Solid) { return new TileClass(kind, oneway, null); }
        }

        // Anything else keeps its own collision outline as polygon objects.
        var outlines = new List<List<(double, double)>>();
        var sources = tile != null && tile.HasCollisionGroup ? tile.Objects : null;
        if (sources == null)
        {
            outlines.Add(Flip(KindOutline(kind, tw, th), h, v, d, tw, th));
        }
        else
        {
            foreach (var o in sources)
            {
                var pts = ObjectOutline(o, warnings, where);
                if (pts != null) { outlines.Add(Flip(pts, h, v, d, tw, th)); }
            }
        }

        return new TileClass(TileKind2D.Empty, oneway, outlines);
    }

    /// <summary>The outline of a grid tile kind in tile pixels, y down.</summary>
    private static List<(double X, double Y)> KindOutline(TileKind2D kind, int tw, int th) => kind switch
    {
        TileKind2D.HalfBottom => new() { (0, th / 2.0), (tw, th / 2.0), (tw, th), (0, th) },
        TileKind2D.SlopeUpRight => new() { (0, th), (tw, th), (tw, 0) },
        TileKind2D.SlopeUpLeft => new() { (0, th), (tw, th), (0, 0) },
        _ => new() { (0, 0), (tw, 0), (tw, th), (0, th) },
    };

    private static List<(double X, double Y)> Flip(List<(double X, double Y)> pts, bool h, bool v, bool d, int tw, int th)
    {
        var r = new List<(double X, double Y)>();
        foreach (var (x0, y0) in pts)
        {
            double x = x0, y = y0;
            if (d) { (x, y) = (y, x); }
            if (h) { x = tw - x; }
            if (v) { y = th - y; }
            r.Add((x, y));
        }

        return r;
    }

    private static TileKind2D ShapeKind(TObj o, int tw, int th)
    {
        double eps = 1e-3;
        if (o.Point || o.Ellipse) { return (TileKind2D)255; }
        if (o.HasPoly && !o.HasLine && o.Poly.Count == 3)
        {
            // In unit coordinates with y up: (0,0), (1,0), then (1,1) for a slope rising right or (0,1) for one rising left.
            var u = o.Poly.Select(p => ((o.X + p.X) / tw, 1 - (o.Y + p.Y) / th)).ToList();
            bool Has(double x, double y) => u.Any(p => MapCore.Near(p.Item1, x, eps) && MapCore.Near(p.Item2, y, eps));
            if (Has(0, 0) && Has(1, 0) && Has(1, 1)) { return TileKind2D.SlopeUpRight; }
            if (Has(0, 0) && Has(1, 0) && Has(0, 1)) { return TileKind2D.SlopeUpLeft; }
            return (TileKind2D)255;
        }

        if (!o.HasPoly && !o.HasLine && o.W > 0 && o.H > 0 && Math.Abs(o.Rot) < 1e-9)
        {
            double x0 = o.X / tw, x1 = (o.X + o.W) / tw, yTop = 1 - o.Y / th, yBot = 1 - (o.Y + o.H) / th;
            if (MapCore.Near(x0, 0, eps) && MapCore.Near(x1, 1, eps) && MapCore.Near(yBot, 0, eps))
            {
                if (MapCore.Near(yTop, 1, eps)) { return TileKind2D.Solid; }
                if (MapCore.Near(yTop, 0.5, eps)) { return TileKind2D.HalfBottom; }
            }
        }

        return (TileKind2D)255;
    }

    private static List<(double X, double Y)>? ObjectOutline(TObj o, List<InterchangeWarning> warnings, string where)
    {
        if (o.Point) { return null; }
        var (s, c) = Math64.SinCos(o.Rot * (Math.PI / 180.0));
        (double, double) Rot(double x, double y) => (o.X + x * c - y * s, o.Y + x * s + y * c);
        if (o.HasPoly && !o.HasLine) { return o.Poly.Select(p => Rot(p.X, p.Y)).ToList(); }
        if (o.HasLine) { warnings.Add(new InterchangeWarning("tile-polyline", where, "a polyline in a tile's collision shapes has no area and is left out.")); return null; }
        if (!(o.W > 0 && o.H > 0)) { return null; }
        if (o.Ellipse) { return MapCore.Ellipse(o.W / 2, o.H / 2).Select(p => Rot(o.W / 2 + p.X, o.H / 2 - p.Y)).ToList(); }
        return new List<(double, double)> { Rot(0, 0), Rot(o.W, 0), Rot(o.W, o.H), Rot(0, o.H) };
    }

    private static void BuildObjects(TLayer l, TMap m, MapOptions o, bool collision, List<InterchangeWarning> warnings, List<MapObject2D> objects, List<MapPoint2D> points, Func<double, double, Vector2D> world)
    {
        double ppm = o.PixelsPerMeter;
        foreach (var ob in l.Objects)
        {
            var kind = MapCore.BodyKind(ob.Props, F, ob.Where);
            double x = ob.X + l.OffsetX, y = ob.Y + l.OffsetY;
            if (ob.Point)
            {
                points.Add(new MapPoint2D(ob.Name, ob.Type, world(x, y), ob.Props));
                continue;
            }

            if (!collision && kind == null) { continue; }
            var body = kind ?? MapBodyKind.Static;
            if (ob.Gid != 0) { warnings.Add(new InterchangeWarning("tile-object", ob.Where, "tile objects carry no collision shape here and are left out.")); continue; }
            var (s, c) = Math64.SinCos(ob.Rot * (Math.PI / 180.0));
            float rotation = (float)(-ob.Rot * (Math.PI / 180.0));
            MapObject2D? made;
            if (ob.HasPoly || ob.HasLine)
            {
                if (ob.Poly.Count < 2 || (ob.HasPoly && ob.Poly.Count < 3)) { warnings.Add(new InterchangeWarning("degenerate-polygon", ob.Where, "a polygon or polyline with too few points is left out.")); continue; }
                var pts = ob.Poly.Select(p => new Vector2D(p.X / ppm, -p.Y / ppm)).ToList();
                if (ob.HasPoly) { made = MapCore.PolygonObject(ob.Name, ob.Type, body, world(x, y), rotation, pts, warnings, ob.Where); }
                else
                {
                    var arr = pts.Select(p => new Vector2F((float)p.X, (float)p.Y)).ToArray();
                    made = new MapObject2D(ob.Name, ob.Type, MapShapeKind.Chain, body, world(x, y), rotation, default, 0, arr);
                }
            }
            else
            {
                if (!(ob.W > 0 && ob.H > 0)) { warnings.Add(new InterchangeWarning("degenerate-shape", ob.Where, "an object without a width and height or a point list is left out.")); continue; }
                // The rectangle's centre: its top-left corner plus half its size turned by the rotation (clockwise in the file's y-down frame).
                double cx = x + (ob.W / 2) * c - (ob.H / 2) * s, cy = y + (ob.W / 2) * s + (ob.H / 2) * c;
                if (ob.Ellipse)
                {
                    if (MapCore.Near(ob.W, ob.H, 1e-9)) { made = new MapObject2D(ob.Name, ob.Type, MapShapeKind.Circle, body, world(cx, cy), rotation, default, (float)(ob.W / 2 / ppm), Array.Empty<Vector2F>()); }
                    else
                    {
                        warnings.Add(new InterchangeWarning("ellipse", ob.Where, "an ellipse is imported as a 16-sided convex polygon."));
                        made = MapCore.PolygonObject(ob.Name, ob.Type, body, world(cx, cy), rotation, MapCore.Ellipse(ob.W / 2 / ppm, ob.H / 2 / ppm), warnings, ob.Where);
                    }
                }
                else { made = new MapObject2D(ob.Name, ob.Type, MapShapeKind.Box, body, world(cx, cy), rotation, new Vector2F((float)(ob.W / ppm), (float)(ob.H / ppm)), 0, Array.Empty<Vector2F>()); }
            }

            if (made == null) { continue; }
            MapCore.Material(made, ob.Props, F, ob.Where);
            objects.Add(made);
        }

        _ = m;
    }
}
