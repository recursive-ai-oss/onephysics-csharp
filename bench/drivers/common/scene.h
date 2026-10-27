/* Reader for the engine-neutral 2D scene format described in docs/scenes.md. Header-only, C99. */
#ifndef OP_SCENE_H
#define OP_SCENE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { char kind[16]; int material; double density, radius; int np; double* p; } OpShape;
typedef struct { int type; /* 0 static 1 kinematic 2 dynamic */ double x, y, angle, vx, vy, w; int bullet, asleep; int shape0, shapeN; } OpBody;
typedef struct { int a, b; double ax, ay, bx, by; int hasLimit; double lower, upper; int hasMotor; double speed, maxTorque; } OpJoint;
typedef struct { int kind; /* 0 ray 1 overlap 2 cast */ double p[5]; } OpQuery;
typedef struct { int step; } OpWake;
typedef struct {
    char name[64]; double dt; int steps; double gx, gy; int substeps, dumpEvery, warmup, settle, queryReps;
    int nMat; double* friction; double* restitution;
    int nBody; OpBody* bodies; int nShape; OpShape* shapes;
    int nJoint; OpJoint* joints; int nQuery; OpQuery* queries; int nWake; OpWake* wakes;
} OpScene;

#define OP_GROW(arr, n, cap) do { if ((n) >= (cap)) { (cap) = (cap) ? (cap) * 2 : 64; (arr) = realloc((arr), (size_t)(cap) * sizeof(*(arr))); } } while (0)

static int op_load_scene(const char* path, OpScene* s)
{
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 0; }
    memset(s, 0, sizeof *s);
    s->dt = 1.0 / 60.0; s->steps = 600; s->gy = -10; s->dumpEvery = 30; s->warmup = 60; s->queryReps = 1;
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
        else if (!strcmp(t, "dim")) { if (INT() != 2) { fprintf(stderr, "dim 2 only\n"); return 0; } }
        else if (!strcmp(t, "dt")) s->dt = NUM();
        else if (!strcmp(t, "steps")) s->steps = INT();
        else if (!strcmp(t, "gravity")) { s->gx = NUM(); s->gy = NUM(); }
        else if (!strcmp(t, "substeps")) s->substeps = INT();
        else if (!strcmp(t, "dump_every")) s->dumpEvery = INT();
        else if (!strcmp(t, "warmup")) s->warmup = INT();
        else if (!strcmp(t, "settle")) s->settle = INT();
        else if (!strcmp(t, "query_reps")) s->queryReps = INT();
        else if (!strcmp(t, "material")) { NEXT(); OP_GROW(s->friction, s->nMat, capM); s->restitution = realloc(s->restitution, (size_t)capM * sizeof(double)); s->friction[s->nMat] = NUM(); s->restitution[s->nMat] = NUM(); s->nMat++; }
        else if (!strcmp(t, "body"))
        {
            OP_GROW(s->bodies, s->nBody, capB);
            OpBody* b = &s->bodies[s->nBody++]; memset(b, 0, sizeof *b);
            char* ty = NEXT(); b->type = !strcmp(ty, "static") ? 0 : !strcmp(ty, "kinematic") ? 1 : 2;
            b->x = NUM(); b->y = NUM(); b->angle = NUM(); b->vx = NUM(); b->vy = NUM(); b->w = NUM();
            char* fl = NEXT(); b->bullet = strstr(fl, "bullet") != NULL; b->asleep = strstr(fl, "asleep") != NULL;
            b->shape0 = s->nShape; b->shapeN = 0;
        }
        else if (!strcmp(t, "shape"))
        {
            OP_GROW(s->shapes, s->nShape, capS);
            OpShape* sh = &s->shapes[s->nShape++]; memset(sh, 0, sizeof *sh);
            strncpy(sh->kind, NEXT(), 15); sh->material = INT(); sh->density = NUM(); sh->radius = NUM();
            int cp = 0; char* tok;
            while ((tok = NEXT())) { if (sh->np >= cp) { cp = cp ? cp * 2 : 8; sh->p = realloc(sh->p, (size_t)cp * sizeof(double)); } sh->p[sh->np++] = strtod(tok, NULL); }
            s->bodies[s->nBody - 1].shapeN++;
        }
        else if (!strcmp(t, "joint"))
        {
            OP_GROW(s->joints, s->nJoint, capJ);
            OpJoint* j = &s->joints[s->nJoint++]; memset(j, 0, sizeof *j);
            NEXT(); j->a = INT(); j->b = INT(); j->ax = NUM(); j->ay = NUM(); j->bx = NUM(); j->by = NUM();
            char* tok;
            while ((tok = NEXT()))
            {
                if (!strcmp(tok, "limit")) { j->hasLimit = 1; j->lower = NUM(); j->upper = NUM(); }
                else if (!strcmp(tok, "motor")) { j->hasMotor = 1; j->speed = NUM(); j->maxTorque = NUM(); }
            }
        }
        else if (!strcmp(t, "query"))
        {
            OP_GROW(s->queries, s->nQuery, capQ);
            OpQuery* q = &s->queries[s->nQuery++]; memset(q, 0, sizeof *q);
            char* k = NEXT(); q->kind = !strcmp(k, "ray") ? 0 : !strcmp(k, "overlap") ? 1 : 2;
            int i = 0; char* tok; while ((tok = NEXT()) && i < 5) q->p[i++] = strtod(tok, NULL);
        }
        else if (!strcmp(t, "event")) { OP_GROW(s->wakes, s->nWake, capW); s->wakes[s->nWake++].step = INT(); /* only wake_all exists */ }
    }
    free(line); fclose(f);
    return 1;
}

/* Run options shared by the C driver. */
typedef struct { const char* scene; const char* dump; const char* result; int threads; int timing; } OpArgs;

static int op_parse_args(int argc, char** argv, OpArgs* a)
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

static double op_now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (double)t.tv_sec * 1e9 + (double)t.tv_nsec; }

static FILE* op_dump_open(const char* path, int n)
{
    if (!path) return NULL;
    FILE* f = fopen(path, "wb"); if (!f) return NULL;
    uint32_t hdr[2] = { (uint32_t)n, 0 };
    fwrite("OPDUMP01", 1, 8, f); fwrite(hdr, 4, 2, f);
    return f;
}

static void op_dump_frame(FILE* f, int step, const double* d, int n)
{
    if (!f) return;
    uint32_t h[2] = { (uint32_t)step, 0 };
    fwrite(h, 4, 2, f); fwrite(d, sizeof(double), (size_t)n * 7, f);
}

typedef struct { double ns[3]; long hits[3]; int count[3]; double reps; } OpQueryTimes;

#endif
