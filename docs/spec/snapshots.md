# Snapshot byte transforms

These transforms wrap arbitrary byte sequences. In particular, a full snapshot
remains snapshot format v1, with every optional engine section intact. The
transforms neither interpret nor discard sections. A decoder returns bytes; the
world's existing atomic restore validates those bytes separately.

## Delta version 1

All integers are unsigned 32-bit little-endian. Header, in order:

| Offset | Value |
| --- | --- |
| 0 | Magic `0x4441504f` (`OPAD`) |
| 4 | Version `1` |
| 8 | Base byte length |
| 12 | Target byte length |
| 16 | SHA-256 of the entire base (32 bytes) |
| 48 | SHA-256 of the entire target (32 bytes) |

Records follow until end of input: target byte offset, positive byte count, then
that many literal target bytes. Records have ascending, nonoverlapping ranges
inside the target. The target starts as the base truncated to the target length;
all bytes beyond the base must be explicitly covered by records. There is no
terminator or padding. An unchanged target uses only the 80-byte header. A
shortened unchanged prefix also needs only the header.

The canonical writer scans offsets from zero. At the first changed byte it
starts a record, extends through subsequent changed bytes, and absorbs equal
gaps of at most eight bytes between changes. Equal bytes after the last change
are not included. Every byte past the end of the base counts as changed.

Apply checks magic/version, output limit, base length/digest, and every record
range before allocating the target. It then copies and checks the target digest.
Wrong bases, malformed records and checksum failures raise
`SnapshotEncodingException`. Checksums detect accidental damage and base mixups;
they are not authentication. Authenticate untrusted messages at the host boundary.

## Compression version 1

Header: magic `0x4341504f` (`OPAC`), version `1`, decoded byte length, then
SHA-256 of decoded bytes. Header length: 44 bytes. This envelope may contain a
full snapshot, a delta, or any other bytes. No native compression library is used.

Each packet starts with one byte `t`:

- `t < 128`: copy the following `t + 1` literal bytes.
- `t >= 128`: repeat the following single byte `t - 128 + 3` times.

The canonical writer takes the longest equal-byte run at the current offset,
capped at 130. A run of at least three becomes a repeat packet. Otherwise it
starts a literal packet and extends until the next run of at least three, end of
input, or 128 literals. A run of 131 bytes therefore becomes a 130-byte repeat
and a one-byte literal. Empty input has only the header. Encoded bytes depend
only on input bytes, not CPU, thread count, runtime codec version or OS.

Decode validates all packets, their total output length and the output limit
before allocation, then checks the decoded digest. Truncation, trailing packets,
unknown versions and damaged checksums raise `SnapshotEncodingException`.
The default output limit is 256 MiB; callers can pass a nonnegative byte limit
on each decode/apply call. Applications should choose a limit appropriate for
their worlds. There is no recursive decoding. Compressed deltas are explicitly
decompressed, then applied, so both limits can be enforced independently.

## APIs and state

`SnapshotEncoding` is the raw byte API: `CreateDelta`, `ApplyDelta`, `Compress`,
`Decompress`. Both worlds expose identical `SnapshotDelta`, `RestoreDelta`,
`SnapshotCompressed` and `RestoreCompressed` convenience members. A host can keep
a base keyframe and encode later steps against it without retaining a world.
The transforms keep no persistent world state and add no registered snapshot
sections. They allocate output buffers and are intended for host tooling, not the
allocation-free simulation loop. Delegates remain excluded just as in full
snapshots. Decode failure leaves a world untouched.

Compression is designed for sparse binary snapshots, especially their zero
lanes, not for general-purpose archival ratios. For highly fragmented changes
a delta can exceed a full snapshot; hosts can compare lengths and send the
smaller representation. Compression also has bounded literal overhead (one byte
per 128 bytes, plus its header).
