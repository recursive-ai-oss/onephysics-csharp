using System;
using System.Globalization;
using System.IO;
using System.Text.Json;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Typed access to a JSON document: every miss names the path and what was expected.</summary>
internal readonly struct Js
{
    public readonly JsonElement E;
    public readonly string Path;
    private readonly string _format;

    public Js(JsonElement e, string path, string format) { E = e; Path = path; _format = format; }

    public static Js Parse(string text, string format)
    {
        if (text.Length > Io.MaxChars) { throw new InterchangeException(format, "document", "the text is longer than " + Io.MaxChars + " characters."); }
        try
        {
            var doc = JsonDocument.Parse(text, new JsonDocumentOptions { MaxDepth = 64, AllowTrailingCommas = false, CommentHandling = JsonCommentHandling.Disallow });
            return new Js(doc.RootElement, "$", format);
        }
        catch (JsonException e)
        {
            throw new InterchangeException(format, "line " + (e is { LineNumber: not null } ? e.LineNumber + 1 : 0), "not valid JSON (" + e.Message + ").");
        }
        catch (ArgumentException)
        {
            // A lone UTF-16 surrogate cannot be written as UTF-8 for the reader.
            throw new InterchangeException(format, "document", "the text holds an invalid UTF-16 sequence (a lone surrogate).");
        }
    }

    public InterchangeException Fail(string message) => new(_format, Path, message);

    public bool IsObject => E.ValueKind == JsonValueKind.Object;

    public bool IsArray => E.ValueKind == JsonValueKind.Array;

    public Js? Get(string key)
    {
        if (!IsObject) { throw Fail("expected an object."); }
        return E.TryGetProperty(key, out var v) && v.ValueKind != JsonValueKind.Null ? new Js(v, Path + "." + key, _format) : null;
    }

    public Js Need(string key) => Get(key) ?? throw Fail("'" + key + "' is required.");

    public Js Obj(string key)
    {
        var v = Need(key);
        if (!v.IsObject) { throw v.Fail("expected an object."); }
        return v;
    }

    public Js? ObjOpt(string key)
    {
        var v = Get(key);
        if (v != null && !v.Value.IsObject) { throw v.Value.Fail("expected an object."); }
        return v;
    }

    public int Length
    {
        get
        {
            if (!IsArray) { throw Fail("expected an array."); }
            return E.GetArrayLength();
        }
    }

    public Js At(int i) => new(E[i], Path + "[" + i.ToString(CultureInfo.InvariantCulture) + "]", _format);

    public Js? ArrOpt(string key)
    {
        var v = Get(key);
        if (v != null && !v.Value.IsArray) { throw v.Value.Fail("expected an array."); }
        return v;
    }

    public double Num()
    {
        if (E.ValueKind != JsonValueKind.Number || !E.TryGetDouble(out double d) || !double.IsFinite(d) || Math.Abs(d) > Io.Bound) { throw Fail("expected a finite number within ±1e9."); }
        return d;
    }

    public double Num(string key, double fallback) => Get(key) is { } v ? v.Num() : fallback;

    public int Int()
    {
        if (E.ValueKind != JsonValueKind.Number || !E.TryGetInt32(out int i)) { throw Fail("expected an integer."); }
        return i;
    }

    public int Int(string key, int fallback) => Get(key) is { } v ? v.Int() : fallback;

    public bool Bool(string key, bool fallback)
    {
        if (Get(key) is not { } v) { return fallback; }
        if (v.E.ValueKind is not (JsonValueKind.True or JsonValueKind.False)) { throw v.Fail("expected true or false."); }
        return v.E.GetBoolean();
    }

    public string Str()
    {
        if (E.ValueKind != JsonValueKind.String) { throw Fail("expected a string."); }
        return E.GetString() ?? "";
    }

    public string? Str(string key) => Get(key) is { } v ? v.Str() : null;

    public double[]? Nums(string key, int count)
    {
        if (Get(key) is not { } v) { return null; }
        if (!v.IsArray || v.E.GetArrayLength() != count) { throw v.Fail("expected an array of " + count + " numbers."); }
        var r = new double[count];
        for (int i = 0; i < count; i++) { r[i] = v.At(i).Num(); }
        return r;
    }
}

internal static class StreamText
{
    public static byte[] ReadBytes(Stream stream, long limit, string format)
    {
        ArgumentNullException.ThrowIfNull(stream);
        using var ms = new MemoryStream();
        var buffer = new byte[16384];
        int n;
        while ((n = stream.Read(buffer, 0, buffer.Length)) > 0)
        {
            ms.Write(buffer, 0, n);
            if (ms.Length > limit) { throw new InterchangeException(format, "document", "the input is longer than " + limit + " bytes."); }
        }

        return ms.ToArray();
    }
}
