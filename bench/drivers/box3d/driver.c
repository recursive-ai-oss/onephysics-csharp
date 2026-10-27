/* Box3D driver: runs an engine-neutral 3D scene through the library's documented C API. */
#include <math.h>
#include "box3d/box3d.h"
#include "../common/scene3.h"

#define DRIVER_ENGINE "box3d"
#ifndef DRIVER_VERSION
#define DRIVER_VERSION "0.1.0"
#endif

typedef struct { long count; } OverlapCtx;
static bool OverlapCb(b3ShapeId shapeId, void* context) { (void)shapeId; ((OverlapCtx*)context)->count++; return true; }
typedef struct { long hit; } CastCtx;
static float CastCb(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId, int triangleIndex, int childIndex, void* context)
{
    (void)shapeId; (void)point; (void)normal; (void)userMaterialId; (void)triangleIndex; (void)childIndex;
    ((CastCtx*)context)->hit = 1; return fraction; /* clip to the closest hit */
}

static b3Vec3 vec3(const double* p) { b3Vec3 v = { (float)p[0], (float)p[1], (float)p[2] }; return v; }

/* Terrain shapes keep their data alive for the whole run; the world copies nothing it does not document. */
static b3MeshData* make_mesh(const double* p, int flip)
{
    int nv = (int)p[0], ni = (int)p[1];
    b3Vec3* verts = malloc((size_t)nv * sizeof(b3Vec3));
    int32_t* idx = malloc((size_t)ni * sizeof(int32_t));
    for (int i = 0; i < nv; i++) verts[i] = vec3(p + 2 + 3 * i);
    for (int t = 0; t < ni; t += 3)
    {
        idx[t] = (int32_t)p[2 + 3 * nv + t]; idx[t + 1] = (int32_t)p[2 + 3 * nv + t + (flip ? 2 : 1)]; idx[t + 2] = (int32_t)p[2 + 3 * nv + t + (flip ? 1 : 2)];
    }
    b3MeshDef def = { 0 };
    def.vertices = verts; def.vertexCount = nv; def.indices = idx; def.triangleCount = ni / 3;
    def.identifyEdges = true; /* triangle adjacency, the documented way to avoid snagging on shared edges */
    b3MeshData* mesh = b3CreateMesh(&def, NULL, 0);
    return mesh;
}

static b3HeightFieldData* make_height_field(const double* p, int transpose, int clockwise, float* minH, float* maxH)
{
    int nx = (int)p[0], nz = (int)p[1]; double cell = p[2];
    float* h = malloc((size_t)nx * nz * sizeof(float));
    *minH = 1e30f; *maxH = -1e30f;
    for (int j = 0; j < nz; j++)
        for (int i = 0; i < nx; i++)
        {
            float v = (float)p[6 + j * nx + i];
            h[transpose ? i * nz + j : j * nx + i] = v;
            if (v < *minH) *minH = v;
            if (v > *maxH) *maxH = v;
        }
    b3HeightFieldDef def = { 0 };
    def.heights = h; def.countX = transpose ? nz : nx; def.countZ = transpose ? nx : nz;
    def.scale = (b3Vec3){ (float)cell, 1.0f, (float)cell };
    def.globalMinimumHeight = *minH; def.globalMaximumHeight = *maxH; def.clockwiseWinding = clockwise != 0;
    return b3CreateHeightField(&def);
}

static float terrain_height_at(b3WorldId world, float x, float z, int* hit)
{
    b3QueryFilter f = b3DefaultQueryFilter();
    b3RayResult r = b3World_CastRayClosest(world, (b3Pos){ x, 1000.0f, z }, (b3Vec3){ 0.0f, -2000.0f, 0.0f }, f);
    *hit = r.hit ? 1 : 0;
    return r.hit ? r.point.y : 0.0f;
}

static void create_shapes(b3WorldId world, b3BodyId body, const Op3Scene* s, const Op3Body* ob, const char** note)
{
    for (int k = 0; k < ob->shapeN; k++)
    {
        const Op3Shape* sh = &s->shapes[ob->shape0 + k];
        b3ShapeDef def = b3DefaultShapeDef();
        def.density = (float)sh->density;
        def.baseMaterial.friction = (float)s->friction[sh->material];
        def.baseMaterial.restitution = (float)s->restitution[sh->material];
        const double* p = sh->p;
        if (!strcmp(sh->kind, "sphere"))
        {
            b3Sphere c = { vec3(p), (float)sh->radius };
            b3CreateSphereShape(body, &def, &c);
        }
        else if (!strcmp(sh->kind, "capsule"))
        {
            b3Capsule c = { vec3(p), vec3(p + 3), (float)sh->radius };
            b3CreateCapsuleShape(body, &def, &c);
        }
        else if (!strcmp(sh->kind, "box"))
        {
            b3Transform t = { vec3(p + 3), { { (float)p[6], (float)p[7], (float)p[8] }, (float)p[9] } };
            b3BoxHull box = b3MakeTransformedBoxHull((float)p[0], (float)p[1], (float)p[2], t);
            b3CreateHullShape(body, &def, &box.base);
        }
        else if (!strcmp(sh->kind, "hull"))
        {
            int n = sh->np / 3;
            b3Vec3* pts = malloc((size_t)n * sizeof(b3Vec3));
            for (int i = 0; i < n; i++) pts[i] = vec3(p + 3 * i);
            b3HullData* hull = b3CreateHull(pts, n, n);
            if (!hull) { fprintf(stderr, "hull creation failed\n"); exit(2); }
            b3CreateHullShape(body, &def, hull);
            free(pts);
        }
        else if (!strcmp(sh->kind, "mesh"))
        {
            /* The winding is confirmed with a ray from above once the shape exists; a miss flips it. */
            int nv = (int)p[0];
            b3MeshData* mesh = make_mesh(p, 0);
            b3ShapeId id = b3CreateMeshShape(body, &def, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f });
            *note = "mesh winding as given";
            int ok = 0;
            for (int v = 0; v < nv && v < 400; v += (nv > 40 ? nv / 40 : 1))
            {
                int hit; float y = terrain_height_at(world, (float)(p[2 + 3 * v] + ob->pos[0]), (float)(p[4 + 3 * v] + ob->pos[2]), &hit);
                if (hit && fabsf(y - (float)(p[3 + 3 * v] + ob->pos[1])) < 0.5f) ok++; /* a vertex's own height, give or take the slope around it */
            }
            if (ok == 0)
            {
                b3DestroyShape(id, false);
                mesh = make_mesh(p, 1);
                b3CreateMeshShape(body, &def, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f });
                *note = "mesh winding flipped";
            }
        }
        else if (!strcmp(sh->kind, "heightfield"))
        {
            /* The layout (row order, winding) and the origin are settled by black-box probing: the shape's bounds and a ray from above. */
            int nx = (int)p[0], nz = (int)p[1]; double cell = p[2];
            int ok = 0;
            for (int variant = 0; variant < 4 && !ok; variant++)
            {
                int transpose = variant & 1, clockwise = variant >> 1;
                float minH, maxH;
                b3HeightFieldData* hf = make_height_field(p, transpose, clockwise, &minH, &maxH);
                b3ShapeId id = b3CreateHeightFieldShape(body, &def, hf);
                /* Where the field sits relative to the body is not stated; the bounds' centre gives x and z (any margin is symmetric)
                   and a ray at one grid point gives y. */
                b3AABB box = b3Shape_GetAABB(id);
                b3Pos cur = b3Body_GetPosition(body);
                float wantCx = (float)(ob->pos[0] + p[3] + 0.5 * (nx - 1) * cell), wantCz = (float)(ob->pos[2] + p[5] + 0.5 * (nz - 1) * cell);
                float dx = wantCx - 0.5f * (box.lowerBound.x + box.upperBound.x), dz = wantCz - 0.5f * (box.lowerBound.z + box.upperBound.z);
                b3Quat none = { { 0, 0, 0 }, 1 };
                b3Body_SetTransform(body, (b3Pos){ cur.x + dx, cur.y, cur.z + dz }, none);
                {
                    int i0 = nx / 2, j0 = nz / 2; int hit0;
                    float y0 = terrain_height_at(world, (float)(ob->pos[0] + p[3] + i0 * cell), (float)(ob->pos[2] + p[5] + j0 * cell), &hit0);
                    float want0 = (float)(ob->pos[1] + p[4] + p[6 + j0 * nx + i0]);
                    b3Pos c2 = b3Body_GetPosition(body);
                    if (hit0) b3Body_SetTransform(body, (b3Pos){ c2.x, c2.y + (want0 - y0), c2.z }, none);
                }
                ok = 1;
                for (int t = 0; t < 12 && ok; t++)
                {
                    int i = (t * 7 + 3) % nx, j = (t * 11 + 5) % nz;
                    float x = (float)(ob->pos[0] + p[3] + i * cell), z = (float)(ob->pos[2] + p[5] + j * cell);
                    int hit; float y = terrain_height_at(world, x, z, &hit);
                    float want = (float)(ob->pos[1] + p[4] + p[6 + j * nx + i]);
                    if (!hit || fabsf(y - want) > 0.02f) { if (getenv("OP_DEBUG")) fprintf(stderr, "variant %d sample (%d,%d): hit %d y %f want %f; aabb x %f..%f y %f..%f z %f..%f\n", variant, i, j, hit, y, want, box.lowerBound.x, box.upperBound.x, box.lowerBound.y, box.upperBound.y, box.lowerBound.z, box.upperBound.z); ok = 0; }
                }
                /* Cell centres tell which way each cell is split: the scene splits from (i, j+1) to (i+1, j). */
                int agree = 0, other = 0, probes = 0;
                if (ok)
                    for (int t = 0; t < 24; t++)
                    {
                        int i = (t * 13 + 2) % (nx - 1), j = (t * 17 + 1) % (nz - 1);
                        float x = (float)(ob->pos[0] + p[3] + (i + 0.5) * cell), z = (float)(ob->pos[2] + p[5] + (j + 0.5) * cell);
                        int hit; float y = terrain_height_at(world, x, z, &hit);
                        float h00 = (float)p[6 + j * nx + i], h10 = (float)p[6 + j * nx + i + 1], h01 = (float)p[6 + (j + 1) * nx + i], h11 = (float)p[6 + (j + 1) * nx + i + 1];
                        float scene = 0.5f * (h01 + h10), alt = 0.5f * (h00 + h11), base = (float)(ob->pos[1] + p[4]);
                        if (fabsf(scene - alt) < 0.01f) continue;
                        probes++;
                        if (hit && fabsf(y - base - scene) < 0.01f) agree++; else if (hit && fabsf(y - base - alt) < 0.01f) other++;
                    }
                static char msg[160];
                snprintf(msg, sizeof msg, "height field transpose=%d clockwise=%d, cell split probes: %d of %d as the scene, %d the other way", transpose, clockwise, agree, probes, other);
                *note = msg;
                if (!ok) b3DestroyShape(id, false);
            }
            if (!ok) { fprintf(stderr, "height field could not be matched to the scene\n"); exit(2); }
        }
        else { fprintf(stderr, "unknown shape %s\n", sh->kind); exit(2); }
    }
}

static void snapshot(const b3BodyId* ids, int n, double* d)
{
    for (int i = 0; i < n; i++)
    {
        b3Pos pos = b3Body_GetPosition(ids[i]); b3Quat q = b3Body_GetRotation(ids[i]);
        b3Vec3 v = b3Body_GetLinearVelocity(ids[i]), w = b3Body_GetAngularVelocity(ids[i]);
        double* o = d + i * 14;
        o[0] = pos.x; o[1] = pos.y; o[2] = pos.z; o[3] = q.v.x; o[4] = q.v.y; o[5] = q.v.z; o[6] = q.s;
        o[7] = v.x; o[8] = v.y; o[9] = v.z; o[10] = w.x; o[11] = w.y; o[12] = w.z;
        o[13] = b3Body_IsAwake(ids[i]) ? 1.0 : 0.0;
    }
}

int main(int argc, char** argv)
{
    Op3Args args; Op3Scene s;
    if (!op3_parse_args(argc, argv, &args) || !op3_load_scene(args.scene, &s)) return 2;

    b3WorldDef wd = b3DefaultWorldDef();
    wd.gravity = (b3Vec3){ (float)s.g[0], (float)s.g[1], (float)s.g[2] };
    wd.workerCount = (uint32_t)args.threads; /* the library's own task system when no task callbacks are supplied */
    b3WorldId world = b3CreateWorld(&wd);
    int subSteps = s.substeps > 0 ? s.substeps : 4; /* the documented recommendation */

    const char* note = "";
    b3BodyId* ids = malloc((size_t)s.nBody * sizeof(b3BodyId));
    for (int i = 0; i < s.nBody; i++)
    {
        const Op3Body* ob = &s.bodies[i];
        b3BodyDef bd = b3DefaultBodyDef();
        int shapeless = ob->shapeN == 0;
        bd.type = ob->type == 0 ? b3_staticBody : (ob->type == 1 || shapeless) ? b3_kinematicBody : b3_dynamicBody;
        bd.position = (b3Pos){ (float)ob->pos[0], (float)ob->pos[1], (float)ob->pos[2] };
        bd.rotation = (b3Quat){ { (float)ob->q[0], (float)ob->q[1], (float)ob->q[2] }, (float)ob->q[3] };
        bd.linearVelocity = vec3(ob->v);
        bd.angularVelocity = vec3(ob->w);
        bd.isBullet = ob->bullet != 0;
        bd.isAwake = !ob->asleep;
        ids[i] = b3CreateBody(world, &bd);
        create_shapes(world, ids[i], &s, ob, &note);
    }
    for (int i = 0; i < s.nJoint; i++)
    {
        const Op3Joint* oj = &s.joints[i];
        if (s.bodies[oj->a].type != 2 && s.bodies[oj->b].type != 2) continue; /* not simulated by any engine */
        b3SphericalJointDef jd = b3DefaultSphericalJointDef();
        jd.base.bodyIdA = ids[oj->a]; jd.base.bodyIdB = ids[oj->b];
        jd.base.localFrameA.p = vec3(oj->pa); jd.base.localFrameB.p = vec3(oj->pb); /* measured from the body origins, as the scene gives them */
        jd.base.collideConnected = false;
        b3CreateSphericalJoint(world, &jd);
    }

    double* d = malloc((size_t)s.nBody * 14 * sizeof(double));
    double* stepNs = malloc((size_t)(s.steps + 1) * sizeof(double));
    FILE* dump = args.timing ? NULL : op3_dump_open(args.dump, s.nBody);
    if (dump) { snapshot(ids, s.nBody, d); op3_dump_frame(dump, 0, d, s.nBody); }

    for (int step = 0; step < s.steps; step++)
    {
        for (int w = 0; w < s.nWake; w++)
            if (s.wakes[w].step == step) for (int i = 0; i < s.nBody; i++) if (s.bodies[i].type == 2) b3Body_SetAwake(ids[i], true);
        double t0 = op3_now_ns();
        b3World_Step(world, (float)s.dt, subSteps);
        stepNs[step] = op3_now_ns() - t0;
        if (dump && ((step + 1) % s.dumpEvery == 0 || step + 1 == s.steps)) { snapshot(ids, s.nBody, d); op3_dump_frame(dump, step + 1, d, s.nBody); }
    }
    if (dump) fclose(dump);

    Op3QueryTimes qt; memset(&qt, 0, sizeof qt);
    if (s.nQuery > 0)
    {
        b3QueryFilter filter = b3DefaultQueryFilter();
        int reps = s.queryReps;
        for (int rep = 0; rep < reps; rep++)
        {
            int measure = reps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long hits = 0; int count = 0; double t0 = op3_now_ns();
                for (int i = 0; i < s.nQuery; i++)
                {
                    const Op3Query* q = &s.queries[i]; if (q->kind != kind) continue;
                    count++;
                    if (kind == 0)
                    {
                        b3RayResult r = b3World_CastRayClosest(world, (b3Pos){ (float)q->p[0], (float)q->p[1], (float)q->p[2] },
                                                               (b3Vec3){ (float)(q->p[3] * q->p[6]), (float)(q->p[4] * q->p[6]), (float)(q->p[5] * q->p[6]) }, filter);
                        if (r.hit) hits++;
                    }
                    else
                    {
                        b3Vec3 zero = { 0.0f, 0.0f, 0.0f };
                        b3ShapeProxy proxy; proxy.points = &zero; proxy.count = 1; proxy.radius = (float)q->p[3];
                        b3Pos at = { (float)q->p[0], (float)q->p[1], (float)q->p[2] };
                        if (kind == 1)
                        {
                            OverlapCtx ctx = { 0 };
                            b3World_OverlapShape(world, at, &proxy, filter, OverlapCb, &ctx); hits += ctx.count;
                        }
                        else
                        {
                            CastCtx ctx = { 0 };
                            b3World_CastShape(world, at, &proxy, (b3Vec3){ (float)q->p[4], (float)q->p[5], (float)q->p[6] }, filter, CastCb, &ctx); hits += ctx.hit;
                        }
                    }
                }
                if (measure) { qt.ns[kind] += op3_now_ns() - t0; qt.count[kind] += count; qt.hits[kind] = hits; }
            }
        }
    }
    char cfg[320];
    snprintf(cfg, sizeof cfg, "default world definition, %d sub-steps (documented recommendation), workerCount %d (built-in task system), sleeping on, shared mesh edges identified%s%s", subSteps, args.threads, note[0] ? ", " : "", note);
    op3_write_result(args.result, DRIVER_ENGINE, DRIVER_VERSION, cfg, &s, args.threads, stepNs, s.nQuery > 0 ? &qt : NULL);
    b3DestroyWorld(world);
    return 0;
}
