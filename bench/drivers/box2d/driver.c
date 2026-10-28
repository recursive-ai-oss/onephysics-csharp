/* Box2D v3 driver: runs an engine-neutral scene through the library's documented C API. */
#include <math.h>
#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "../common/scene.h"

#define DRIVER_ENGINE "box2d"
#ifndef DRIVER_VERSION
#define DRIVER_VERSION "3.2.0"
#endif

typedef struct { long count; } OverlapCtx;
static bool OverlapCb(b2ShapeId shapeId, void* context) { (void)shapeId; ((OverlapCtx*)context)->count++; return true; }
typedef struct { long hit; } CastCtx;
static float CastCb(b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context)
{
    (void)shapeId; (void)point; (void)normal; ((CastCtx*)context)->hit = 1; return fraction; /* clip to the closest hit */
}

static void create_shapes(b2BodyId body, const OpScene* s, const OpBody* ob)
{
    for (int k = 0; k < ob->shapeN; k++)
    {
        const OpShape* sh = &s->shapes[ob->shape0 + k];
        b2ShapeDef def = b2DefaultShapeDef();
        def.density = (float)sh->density;
        def.material.friction = (float)s->friction[sh->material];
        def.material.restitution = (float)s->restitution[sh->material];
        const double* p = sh->p;
        if (!strcmp(sh->kind, "circle"))
        {
            b2Circle c = { { (float)p[0], (float)p[1] }, (float)sh->radius };
            b2CreateCircleShape(body, &def, &c);
        }
        else if (!strcmp(sh->kind, "capsule"))
        {
            b2Capsule c = { { (float)p[0], (float)p[1] }, { (float)p[2], (float)p[3] }, (float)sh->radius };
            b2CreateCapsuleShape(body, &def, &c);
        }
        else if (!strcmp(sh->kind, "box"))
        {
            b2Polygon poly = b2MakeOffsetRoundedBox((float)p[0], (float)p[1], (b2Vec2){ (float)p[2], (float)p[3] }, b2MakeRot((float)p[4]), (float)sh->radius);
            b2CreatePolygonShape(body, &def, &poly);
        }
        else if (!strcmp(sh->kind, "polygon"))
        {
            b2Vec2 pts[8]; int n = sh->np / 2; if (n > 8) n = 8;
            for (int i = 0; i < n; i++) pts[i] = (b2Vec2){ (float)p[2 * i], (float)p[2 * i + 1] };
            b2Hull hull = b2ComputeHull(pts, n);
            b2Polygon poly = b2MakePolygon(&hull, (float)sh->radius);
            b2CreatePolygonShape(body, &def, &poly);
        }
        else if (!strcmp(sh->kind, "segment"))
        {
            b2Segment seg = { { (float)p[0], (float)p[1] }, { (float)p[2], (float)p[3] } };
            b2CreateSegmentShape(body, &def, &seg);
        }
        else if (!strcmp(sh->kind, "chain")) /* a two-sided polyline: one plain segment per link */
        {
            for (int i = 0; i + 3 < sh->np; i += 2)
            {
                b2Segment seg = { { (float)p[i], (float)p[i + 1] }, { (float)p[i + 2], (float)p[i + 3] } };
                b2CreateSegmentShape(body, &def, &seg);
            }
        }
        else { fprintf(stderr, "unknown shape %s\n", sh->kind); exit(2); }
    }
}

static void snapshot(const b2BodyId* ids, int n, double* d)
{
    for (int i = 0; i < n; i++)
    {
        b2Pos pos = b2Body_GetPosition(ids[i]); b2Rot rot = b2Body_GetRotation(ids[i]);
        b2Vec2 v = b2Body_GetLinearVelocity(ids[i]);
        d[i * 7 + 0] = pos.x; d[i * 7 + 1] = pos.y; d[i * 7 + 2] = b2Rot_GetAngle(rot);
        d[i * 7 + 3] = v.x; d[i * 7 + 4] = v.y; d[i * 7 + 5] = b2Body_GetAngularVelocity(ids[i]);
        d[i * 7 + 6] = b2Body_IsAwake(ids[i]) ? 1.0 : 0.0;
    }
}

int main(int argc, char** argv)
{
    OpArgs args; OpScene s;
    if (!op_parse_args(argc, argv, &args) || !op_load_scene(args.scene, &s)) return 2;

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = (b2Vec2){ (float)s.gx, (float)s.gy };
    wd.workerCount = args.threads; /* Box2D runs its own worker threads when no task callbacks are supplied */
    b2WorldId world = b2CreateWorld(&wd);
    int subSteps = s.substeps > 0 ? s.substeps : 4; /* the documented recommendation */

    b2BodyId* ids = malloc((size_t)s.nBody * sizeof(b2BodyId));
    for (int i = 0; i < s.nBody; i++)
    {
        const OpBody* ob = &s.bodies[i];
        b2BodyDef bd = b2DefaultBodyDef();
        bd.type = ob->type == 0 ? b2_staticBody : ob->type == 1 ? b2_kinematicBody : b2_dynamicBody;
        bd.position = (b2Pos){ (float)ob->x, (float)ob->y };
        bd.rotation = b2MakeRot((float)ob->angle);
        bd.linearVelocity = (b2Vec2){ (float)ob->vx, (float)ob->vy };
        bd.angularVelocity = (float)ob->w;
        bd.isBullet = ob->bullet != 0;
        bd.isAwake = !ob->asleep;
        ids[i] = b2CreateBody(world, &bd);
        create_shapes(ids[i], &s, ob);
    }
    for (int i = 0; i < s.nJoint; i++)
    {
        const OpJoint* oj = &s.joints[i];
        if (s.bodies[oj->a].type != 2 && s.bodies[oj->b].type != 2) continue; /* not simulated by any engine */
        b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
        jd.base.bodyIdA = ids[oj->a]; jd.base.bodyIdB = ids[oj->b];
        jd.base.localFrameA.p = (b2Vec2){ (float)oj->ax, (float)oj->ay };
        jd.base.localFrameB.p = (b2Vec2){ (float)oj->bx, (float)oj->by };
        /* the joint angle is zero in the initial pose */
        jd.base.localFrameA.q = b2MakeRot(0.0f);
        jd.base.localFrameB.q = b2MakeRot((float)(s.bodies[oj->a].angle - s.bodies[oj->b].angle));
        jd.base.collideConnected = false;
        if (oj->hasLimit) { jd.enableLimit = true; jd.lowerAngle = (float)oj->lower; jd.upperAngle = (float)oj->upper; }
        if (oj->hasMotor) { jd.enableMotor = true; jd.motorSpeed = (float)oj->speed; jd.maxMotorTorque = (float)oj->maxTorque; }
        b2CreateRevoluteJoint(world, &jd);
    }

    double* d = malloc((size_t)s.nBody * 7 * sizeof(double));
    double* stepNs = malloc((size_t)(s.steps + 1) * sizeof(double));
    FILE* dump = args.timing ? NULL : op_dump_open(args.dump, s.nBody);
    if (dump) { snapshot(ids, s.nBody, d); op_dump_frame(dump, 0, d, s.nBody); }

    // Optional per-stage profile (b2World_GetProfile) and counters (b2World_GetCounters), averaged over the measured steps; written only when OP_PROFILE is set.
    int profiling = getenv("OP_PROFILE") != NULL;
    double prof[23] = { 0 }, cnt[6] = { 0 };
    int profSteps = 0;
    for (int step = 0; step < s.steps; step++)
    {
        for (int w = 0; w < s.nWake; w++)
            if (s.wakes[w].step == step) for (int i = 0; i < s.nBody; i++) if (s.bodies[i].type == 2) b2Body_SetAwake(ids[i], true);
        double t0 = op_now_ns();
        b2World_Step(world, (float)s.dt, subSteps);
        stepNs[step] = op_now_ns() - t0;
        if (profiling && step >= s.warmup)
        {
            b2Profile p = b2World_GetProfile(world); b2Counters c = b2World_GetCounters(world);
            float v[23] = { p.step, p.pairs, p.collide, p.solve, p.solverSetup, p.constraints, p.prepareConstraints, p.integrateVelocities, p.warmStart, p.solveImpulses, p.integratePositions, p.relaxImpulses, p.restitution, p.storeImpulses, p.splitIslands, p.transforms, p.hitEvents, p.refit, p.bullets, p.sleepIslands, p.sensors, p.sensorHits, p.jointEvents };
            for (int k = 0; k < 23; k++) prof[k] += v[k];
            int touching = 0; for (int k = 0; k < 24; k++) touching += c.colorCounts[k];
            cnt[0] += c.contactCount; cnt[1] += c.awakeContactCount; cnt[2] += c.recycledContactCount; cnt[3] += touching; cnt[4] += c.bodyCount; cnt[5] += c.islandCount;
            profSteps++;
        }
        if (dump && ((step + 1) % s.dumpEvery == 0 || step + 1 == s.steps)) { snapshot(ids, s.nBody, d); op_dump_frame(dump, step + 1, d, s.nBody); }
    }
    if (dump) fclose(dump);

    OpQueryTimes qt; memset(&qt, 0, sizeof qt);
    if (s.nQuery > 0)
    {
        b2QueryFilter filter = b2DefaultQueryFilter();
        int reps = s.queryReps, timed = reps > 1 ? reps - 1 : 1;
        for (int rep = 0; rep < reps; rep++)
        {
            int measure = reps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long hits = 0; int count = 0; double t0 = op_now_ns();
                for (int i = 0; i < s.nQuery; i++)
                {
                    const OpQuery* q = &s.queries[i]; if (q->kind != kind) continue;
                    count++;
                    if (kind == 0)
                    {
                        b2RayResult r = b2World_CastRayClosest(world, (b2Pos){ (float)q->p[0], (float)q->p[1] }, (b2Vec2){ (float)(q->p[2] * q->p[4]), (float)(q->p[3] * q->p[4]) }, filter);
                        hits += r.hit ? 1 : 0;
                    }
                    else if (kind == 1)
                    {
                        b2Vec2 c = { (float)q->p[0], (float)q->p[1] }; b2ShapeProxy proxy = b2MakeProxy(&c, 1, (float)q->p[2]);
                        OverlapCtx ctx = { 0 }; b2World_OverlapShape(world, b2Pos_zero, &proxy, filter, OverlapCb, &ctx); hits += ctx.count;
                    }
                    else
                    {
                        b2Vec2 c = { (float)q->p[0], (float)q->p[1] }; b2ShapeProxy proxy = b2MakeProxy(&c, 1, (float)q->p[2]);
                        CastCtx ctx = { 0 }; b2World_CastShape(world, b2Pos_zero, &proxy, (b2Vec2){ (float)q->p[3], (float)q->p[4] }, filter, CastCb, &ctx); hits += ctx.hit;
                    }
                }
                if (measure) { qt.ns[kind] += op_now_ns() - t0; qt.count[kind] += count; qt.hits[kind] = hits; }
            }
        }
        (void)timed;
    }

    FILE* r = fopen(args.result, "w");
    fprintf(r, "engine=%s\nversion=%s\nconfig=workerCount=%d substeps=%d native-task-system\nscene=%s\nthreads=%d\nsteps=%d\nwarmup=%d\nbodies=%d\n",
            DRIVER_ENGINE, DRIVER_VERSION, args.threads, subSteps, s.name, args.threads, s.steps, s.warmup, s.nBody);
    fprintf(r, "step_ns="); for (int i = 0; i < s.steps; i++) fprintf(r, i ? ",%.0f" : "%.0f", stepNs[i]); fprintf(r, "\n");
    const char* names[3] = { "ray", "overlap", "cast" };
    for (int k = 0; k < 3 && s.nQuery > 0; k++) fprintf(r, "query_%s_ns=%.0f\nquery_%s_count=%d\nquery_%s_hits=%ld\n", names[k], qt.ns[k], names[k], qt.count[k], names[k], qt.hits[k]);
    if (profiling && profSteps > 0)
    {
        const char* pn[23] = { "step", "pairs", "collide", "solve", "solverSetup", "constraints", "prepareConstraints", "integrateVelocities", "warmStart", "solveImpulses", "integratePositions", "relaxImpulses", "restitution", "storeImpulses", "splitIslands", "transforms", "hitEvents", "refit", "bullets", "sleepIslands", "sensors", "sensorHits", "jointEvents" };
        for (int k = 0; k < 23; k++) fprintf(r, "prof_%s_ms=%.4f\n", pn[k], prof[k] / profSteps);
        const char* cn[6] = { "contactCount", "awakeContactCount", "recycledContactCount", "touchingConstraints", "bodyCount", "islandCount" };
        for (int k = 0; k < 6; k++) fprintf(r, "count_%s=%.1f\n", cn[k], cnt[k] / profSteps);
    }
    fclose(r);
    b2DestroyWorld(world);
    return 0;
}
