/* Reader for the engine-neutral 3D scene format (docs/scenes.md). Header-only; compiles as C99 and as C++. */
#ifndef OP_SCENE3_H
#define OP_SCENE3_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { char kind[16]; int material; double density, radius; int np; double* p; } Op3Shape;
typedef struct { int type; /* 0 static 1 kinematic 2 dynamic */ double pos[3], q[4], v[3], w[3]; int bullet, asleep; int shape0, shapeN; } Op3Body;
typedef struct { char kind[8]; int a, b; double pa[3], pb[3], axis[3]; int hasLimit; double lower, upper; } Op3Joint;
typedef struct { int kind; /* 0 ray 1 overlap 2 cast */ double p[8]; } Op3Query;
typedef struct { int step; } Op3Wake;
typedef struct {
    char name[64]; double dt; int steps; double g[3]; int substeps, dumpEvery, warmup, settle, queryReps;
    int nMat; double* friction; double* restitution;
    int nBody; Op3Body* bodies; int nShape; Op3Shape* shapes;
    int nJoint; Op3Joint* joints; int nQuery; Op3Query* queries; int nWake; Op3Wake* wakes;
} Op3Scene;

#define OP3_GROW(T, arr, n, cap) do { if ((n) >= (cap)) { (cap) = (cap) ? (cap) * 2 : 64; (arr) = (T*)realloc((arr), (size_t)(cap) * sizeof(T)); } } while (0)

static int op3_load_scene(const char* path, Op3Scene* s)
{
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 0; }
    memset(s, 0, sizeof *s);
    s->dt = 1.0 / 60.0; s->steps = 600; s->g[1] = -10; s->dumpEvery = 30; s->warmup = 60; s->queryReps = 1;
    int capM = 0, capB = 0, capS = 0, capJ = 0, capQ = 0, capW = 0;
    char* line = NULL; size_t cap = 0;
    while (getline(&line, &cap, f) > 0)
    {
        char* h = strchr(line, '#'); if (h) *h = 0;
        char* save; char* t = strtok_r(line, " \t\r\n", &save);
        if (!t) continue;
#define NEXT() strtok_r(NULL, " \t\r\n", &save)
#define NUM() strtod(NEXT(), NULL)
#define INT() ((int)strtol(NEXT(), NULL, 10))
        if (!strcmp(t, "name")) { strncpy(s->name, NEXT(), 63); }
        else if (!strcmp(t, "dim")) { if (INT() != 3) { fprintf(stderr, "dim 3 only\n"); return 0; } }
        else if (!strcmp(t, "dt")) s->dt = NUM();
        else if (!strcmp(t, "steps")) s->steps = INT();
        else if (!strcmp(t, "gravity")) { s->g[0] = NUM(); s->g[1] = NUM(); s->g[2] = NUM(); }
        else if (!strcmp(t, "substeps")) s->substeps = INT();
        else if (!strcmp(t, "dump_every")) s->dumpEvery = INT();
        else if (!strcmp(t, "warmup")) s->warmup = INT();
        else if (!strcmp(t, "settle")) s->settle = INT();
        else if (!strcmp(t, "query_reps")) s->queryReps = INT();
        else if (!strcmp(t, "material"))
        {
            NEXT(); OP3_GROW(double, s->friction, s->nMat, capM); s->restitution = (double*)realloc(s->restitution, (size_t)capM * sizeof(double));
            s->friction[s->nMat] = NUM(); s->restitution[s->nMat] = NUM(); s->nMat++;
        }
        else if (!strcmp(t, "body"))
        {
            OP3_GROW(Op3Body, s->bodies, s->nBody, capB);
            Op3Body* b = &s->bodies[s->nBody++]; memset(b, 0, sizeof *b);
            char* ty = NEXT(); b->type = !strcmp(ty, "static") ? 0 : !strcmp(ty, "kinematic") ? 1 : 2;
            for (int i = 0; i < 3; i++) b->pos[i] = NUM();
            for (int i = 0; i < 4; i++) b->q[i] = NUM();
            for (int i = 0; i < 3; i++) b->v[i] = NUM();
            for (int i = 0; i < 3; i++) b->w[i] = NUM();
            char* fl = NEXT(); b->bullet = strstr(fl, "bullet") != NULL; b->asleep = strstr(fl, "asleep") != NULL;
            b->shape0 = s->nShape; b->shapeN = 0;
        }
        else if (!strcmp(t, "shape"))
        {
            OP3_GROW(Op3Shape, s->shapes, s->nShape, capS);
            Op3Shape* sh = &s->shapes[s->nShape++]; memset(sh, 0, sizeof *sh);
            strncpy(sh->kind, NEXT(), 15); sh->material = INT(); sh->density = NUM(); sh->radius = NUM();
            int cp = 0; char* tok;
            while ((tok = NEXT())) { if (sh->np >= cp) { cp = cp ? cp * 2 : 16; sh->p = (double*)realloc(sh->p, (size_t)cp * sizeof(double)); } sh->p[sh->np++] = strtod(tok, NULL); }
            s->bodies[s->nBody - 1].shapeN++;
        }
        else if (!strcmp(t, "joint"))
        {
            OP3_GROW(Op3Joint, s->joints, s->nJoint, capJ);
            Op3Joint* j = &s->joints[s->nJoint++]; memset(j, 0, sizeof *j);
            strncpy(j->kind, NEXT(), 7); j->a = INT(); j->b = INT();
            for (int i = 0; i < 3; i++) j->pa[i] = NUM();
            for (int i = 0; i < 3; i++) j->pb[i] = NUM();
            if (!strcmp(j->kind, "hinge")) for (int i = 0; i < 3; i++) j->axis[i] = NUM();
            char* tok;
            while ((tok = NEXT())) if (!strcmp(tok, "limit")) { j->hasLimit = 1; j->lower = NUM(); j->upper = NUM(); }
        }
        else if (!strcmp(t, "query"))
        {
            OP3_GROW(Op3Query, s->queries, s->nQuery, capQ);
            Op3Query* q = &s->queries[s->nQuery++]; memset(q, 0, sizeof *q);
            char* k = NEXT(); q->kind = !strcmp(k, "ray") ? 0 : !strcmp(k, "overlap") ? 1 : 2;
            int i = 0; char* tok; while ((tok = NEXT()) && i < 8) q->p[i++] = strtod(tok, NULL);
        }
        else if (!strcmp(t, "event")) { OP3_GROW(Op3Wake, s->wakes, s->nWake, capW); s->wakes[s->nWake++].step = INT(); }
    }
    free(line); fclose(f);
    return 1;
}

typedef struct { const char* scene; const char* dump; const char* result; int threads; int timing; } Op3Args;

static int op3_parse_args(int argc, char** argv, Op3Args* a)
{
    memset(a, 0, sizeof *a); a->threads = 1;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        if (!strcmp(argv[i], "--scene")) a->scene = argv[i + 1];
        else if (!strcmp(argv[i], "--dump")) a->dump = argv[i + 1];
        else if (!strcmp(argv[i], "--result")) a->result = argv[i + 1];
        else if (!strcmp(argv[i], "--threads")) a->threads = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--mode")) a->timing = !strcmp(argv[i + 1], "timing");
        else { fprintf(stderr, "unknown argument %s\n", argv[i]); return 0; }
    }
    if (!a->scene || !a->result) { fprintf(stderr, "usage: driver --scene F --result F [--dump F] [--threads N] [--mode timing|stability]\n"); return 0; }
    return 1;
}

static double op3_now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (double)t.tv_sec * 1e9 + (double)t.tv_nsec; }

/* Dump: header "OPDUMP01", body count, 3 (dimension); frames of step, 0, then 14 doubles per body:
   x y z qx qy qz qw vx vy vz wx wy wz awake. */
static FILE* op3_dump_open(const char* path, int n)
{
    if (!path) return NULL;
    FILE* f = fopen(path, "wb"); if (!f) return NULL;
    uint32_t hdr[2] = { (uint32_t)n, 3u };
    fwrite("OPDUMP01", 1, 8, f); fwrite(hdr, 4, 2, f);
    return f;
}

static void op3_dump_frame(FILE* f, int step, const double* d, int n)
{
    if (!f) return;
    uint32_t h[2] = { (uint32_t)step, 0 };
    fwrite(h, 4, 2, f); fwrite(d, sizeof(double), (size_t)n * 14, f);
}

typedef struct { double ns[3]; long hits[3]; int count[3]; } Op3QueryTimes;

static void op3_write_result(const char* path, const char* engine, const char* version, const char* config, const Op3Scene* s, int threads, const double* stepNs, const Op3QueryTimes* qt)
{
    FILE* r = fopen(path, "w");
    fprintf(r, "engine=%s\nversion=%s\nconfig=%s\nscene=%s\nthreads=%d\nsteps=%d\nwarmup=%d\nbodies=%d\n", engine, version, config, s->name, threads, s->steps, s->warmup, s->nBody);
    fprintf(r, "step_ns="); for (int i = 0; i < s->steps; i++) fprintf(r, i ? ",%.0f" : "%.0f", stepNs[i]); fprintf(r, "\n");
    const char* names[3] = { "ray", "overlap", "cast" };
    if (qt) for (int k = 0; k < 3; k++) fprintf(r, "query_%s_ns=%.0f\nquery_%s_count=%d\nquery_%s_hits=%ld\n", names[k], qt->ns[k], names[k], qt->count[k], names[k], qt->hits[k]);
    fclose(r);
}

#endif
