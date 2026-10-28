// Jolt Physics driver: runs an engine-neutral 3D scene through the library's documented C++ API.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <cmath>
#include <unordered_set>
#include <vector>
#include "../common/scene3.h"

using namespace JPH;
using namespace JPH::literals;

namespace Layers { static constexpr ObjectLayer NON_MOVING = 0, MOVING = 1, NUM_LAYERS = 2; }
namespace BroadPhaseLayers { static constexpr BroadPhaseLayer NON_MOVING(0), MOVING(1); static constexpr uint NUM_LAYERS(2); }

class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
    bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override { return a == Layers::MOVING || b == Layers::MOVING; }
};

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer l) const override { return l == Layers::NON_MOVING ? BroadPhaseLayers::NON_MOVING : BroadPhaseLayers::MOVING; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override { return "layer"; }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(ObjectLayer l, BroadPhaseLayer b) const override { return l == Layers::MOVING || b == BroadPhaseLayers::MOVING; }
};

// Bodies joined by a constraint must not collide with each other: a group filter that knows the joined pairs.
class JointedPairs : public GroupFilter
{
public:
    std::unordered_set<uint64_t> pairs;
    bool CanCollide(const CollisionGroup& a, const CollisionGroup& b) const override
    {
        uint64_t x = a.GetSubGroupID(), y = b.GetSubGroupID();
        if (x > y) std::swap(x, y);
        return pairs.find((x << 32) | y) == pairs.end();
    }
};

static Vec3 V3(const double* p) { return Vec3((float)p[0], (float)p[1], (float)p[2]); }

static ShapeRefC MakeShape(const Op3Shape& sh)
{
    const double* p = sh.p;
    ShapeSettings::ShapeResult r;
    Vec3 offset = Vec3::sZero();
    if (!strcmp(sh.kind, "sphere")) { SphereShapeSettings s((float)sh.radius); s.mDensity = (float)sh.density; r = s.Create(); offset = V3(p); }
    else if (!strcmp(sh.kind, "box")) { BoxShapeSettings s(Vec3((float)p[0], (float)p[1], (float)p[2]), 0.0f); s.mDensity = (float)sh.density; r = s.Create(); offset = V3(p + 3); }
    else if (!strcmp(sh.kind, "capsule"))
    {
        // capsules in the scenes are aligned with the y axis; Jolt's capsule is centred on its origin
        Vec3 a = V3(p), b = V3(p + 3);
        if (std::fabs(a.GetX() - b.GetX()) > 1e-6f || std::fabs(a.GetZ() - b.GetZ()) > 1e-6f) { fprintf(stderr, "capsule not aligned with y\n"); exit(2); }
        CapsuleShapeSettings s(0.5f * std::fabs(b.GetY() - a.GetY()), (float)sh.radius); s.mDensity = (float)sh.density; r = s.Create();
        offset = 0.5f * (a + b);
    }
    else if (!strcmp(sh.kind, "hull"))
    {
        Array<Vec3> pts; for (int i = 0; i + 2 < sh.np; i += 3) pts.push_back(V3(p + i));
        ConvexHullShapeSettings s(pts); s.mDensity = (float)sh.density; r = s.Create();
    }
    else if (!strcmp(sh.kind, "mesh"))
    {
        int nv = (int)p[0], ni = (int)p[1];
        TriangleList tris;
        for (int t = 0; t < ni; t += 3)
        {
            Float3 v[3];
            for (int k = 0; k < 3; k++) { int o = 2 + 3 * (int)p[2 + 3 * nv + t + k]; v[k] = Float3((float)p[o], (float)p[o + 1], (float)p[o + 2]); }
            tris.push_back(Triangle(v[0], v[1], v[2]));
        }
        MeshShapeSettings s(tris); r = s.Create();
    }
    else if (!strcmp(sh.kind, "heightfield"))
    {
        int nx = (int)p[0], nz = (int)p[1]; double cell = p[2];
        std::vector<float> h((size_t)nx * nz);
        for (size_t i = 0; i < h.size(); i++) h[i] = (float)p[6 + i];
        HeightFieldShapeSettings s(h.data(), Vec3((float)p[3], (float)p[4], (float)p[5]), Vec3((float)cell, 1.0f, (float)cell), (uint32)nx);
        r = s.Create();
    }
    else { fprintf(stderr, "unknown shape %s\n", sh.kind); exit(2); }
    if (r.HasError()) { fprintf(stderr, "shape error: %s\n", r.GetError().c_str()); exit(2); }
    ShapeRefC shape = r.Get();
    if (offset.LengthSq() > 0.0f) shape = RotatedTranslatedShapeSettings(offset, Quat::sIdentity(), shape).Create().Get();
    return shape;
}

// Scripted characters are Jolt CharacterVirtual objects (documented class); their dump rows hold the position and the displacement per second of the last step.
struct CharState { Ref<CharacterVirtual> ch; RVec3 prev; Vec3 vel; float vy; };

static void snapshot(BodyInterface& bi, const std::vector<BodyID>& ids, double* d, const std::vector<int>& charOf, const std::vector<CharState>& chars)
{
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (charOf[i] >= 0)
        {
            const CharState& c = chars[(size_t)charOf[i]]; RVec3 p = c.ch->GetPosition(); double* o = d + i * 14;
            o[0] = p.GetX(); o[1] = p.GetY(); o[2] = p.GetZ(); o[3] = o[4] = o[5] = 0; o[6] = 1;
            o[7] = c.vel.GetX(); o[8] = c.vel.GetY(); o[9] = c.vel.GetZ(); o[10] = o[11] = o[12] = 0; o[13] = 1;
            continue;
        }
        RVec3 p = bi.GetPosition(ids[i]); Quat q = bi.GetRotation(ids[i]); Vec3 v = bi.GetLinearVelocity(ids[i]), w = bi.GetAngularVelocity(ids[i]);
        double* o = d + i * 14;
        o[0] = p.GetX(); o[1] = p.GetY(); o[2] = p.GetZ(); o[3] = q.GetX(); o[4] = q.GetY(); o[5] = q.GetZ(); o[6] = q.GetW();
        o[7] = v.GetX(); o[8] = v.GetY(); o[9] = v.GetZ(); o[10] = w.GetX(); o[11] = w.GetY(); o[12] = w.GetZ();
        o[13] = bi.IsActive(ids[i]) ? 1.0 : 0.0;
    }
}

// Scripted cars are Jolt VehicleConstraint objects with a WheeledVehicleController (documented classes), built with the "car" preset of docs/scenes.md: four wheels of 0.34 m radius 0.8 m either side and 1.3 m ahead and
// behind the centre, suspension 0.05 to 0.5 m at 1.5 Hz and damping 0.6, front wheels steering to 0.5 rad, rear handbrake, 400 N m engine between 1000 and 6500 rpm, six-speed automatic, one rear differential of ratio 3.4,
// anti-roll bars of 20 kN/m. Everything the preset does not name keeps the library's default (tyre curves, torque curve, clutch).
struct CarState { Ref<VehicleConstraint> vc; WheeledVehicleController* ctrl; int body; };

static CarState MakeCar(PhysicsSystem& physics, BodyInterface& bi, BodyID id)
{
    VehicleConstraintSettings vcs;
    vcs.mUp = Vec3::sAxisY(); vcs.mForward = Vec3::sAxisZ();
    const float xs[4] = { -0.8f, 0.8f, -0.8f, 0.8f }, zs[4] = { 1.3f, 1.3f, -1.3f, -1.3f };
    for (int k = 0; k < 4; k++)
    {
        WheelSettingsWV* w = new WheelSettingsWV;
        w->mPosition = Vec3(xs[k], 0.0f, zs[k]);
        w->mSuspensionDirection = Vec3(0, -1, 0); w->mSteeringAxis = Vec3(0, 1, 0); w->mWheelUp = Vec3(0, 1, 0); w->mWheelForward = Vec3(0, 0, 1);
        w->mSuspensionMinLength = 0.05f; w->mSuspensionMaxLength = 0.5f;
        w->mSuspensionSpring.mFrequency = 1.5f; w->mSuspensionSpring.mDamping = 0.6f;
        w->mRadius = 0.34f; w->mWidth = 0.25f;
        w->mMaxSteerAngle = k < 2 ? 0.5f : 0.0f;
        w->mMaxBrakeTorque = 3000.0f; w->mMaxHandBrakeTorque = k < 2 ? 0.0f : 4000.0f;
        w->mInertia = 1.0f; w->mAngularDamping = 0.2f;
        vcs.mWheels.push_back(w);
    }
    for (int a = 0; a < 2; a++) { VehicleAntiRollBar bar; bar.mLeftWheel = 2 * a; bar.mRightWheel = 2 * a + 1; bar.mStiffness = 20000.0f; vcs.mAntiRollBars.push_back(bar); }
    WheeledVehicleControllerSettings* cs = new WheeledVehicleControllerSettings;
    cs->mEngine.mMaxTorque = 400.0f; cs->mEngine.mMinRPM = 1000.0f; cs->mEngine.mMaxRPM = 6500.0f; cs->mEngine.mInertia = 0.5f;
    cs->mTransmission.mMode = ETransmissionMode::Auto;
    cs->mTransmission.mGearRatios = { 3.5f, 2.2f, 1.5f, 1.1f, 0.85f, 0.7f };
    cs->mTransmission.mReverseGearRatios = { -3.2f };
    cs->mDifferentials.resize(1);
    cs->mDifferentials[0].mLeftWheel = 2; cs->mDifferentials[0].mRightWheel = 3; cs->mDifferentials[0].mDifferentialRatio = 3.4f; cs->mDifferentials[0].mEngineTorqueRatio = 1.0f;
    vcs.mController = cs;
    CarState c; c.vc = new VehicleConstraint(*bi.TryGetBody(id), vcs);
    c.vc->SetVehicleCollisionTester(new VehicleCollisionTesterCastCylinder(Layers::MOVING));
    physics.AddConstraint(c.vc);
    physics.AddStepListener(c.vc);
    c.ctrl = static_cast<WheeledVehicleController*>(c.vc->GetController());
    c.body = 0;
    return c;
}

int main(int argc, char** argv)
{
    Op3Args args; Op3Scene s;
    if (!op3_parse_args(argc, argv, &args) || !op3_load_scene(args.scene, &s)) return 2;

    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    const uint maxBodies = (uint)s.nBody + 64;
    TempAllocatorImpl temp(512u * 1024u * 1024u);
    JobSystemThreadPool job(cMaxPhysicsJobs, cMaxPhysicsBarriers, args.threads - 1); // the calling thread works too
    BPLayerInterfaceImpl bpLayers; ObjectVsBroadPhaseLayerFilterImpl objVsBp; ObjectLayerPairFilterImpl objPair;
    PhysicsSystem physics;
    physics.Init(maxBodies, 0, 1u << 20, 1u << 20, bpLayers, objVsBp, objPair);
    physics.SetGravity(Vec3((float)s.g[0], (float)s.g[1], (float)s.g[2]));
    BodyInterface& bi = physics.GetBodyInterface();

    // Bodies that share a joint need the pair filter.
    Ref<JointedPairs> filter = new JointedPairs();
    std::vector<char> jointed((size_t)s.nBody, 0);
    for (int i = 0; i < s.nJoint; i++)
    {
        uint64_t a = (uint64_t)s.joints[i].a, b = (uint64_t)s.joints[i].b; if (a > b) std::swap(a, b);
        filter->pairs.insert((a << 32) | b); jointed[a] = jointed[b] = 1;
    }

    std::vector<BodyID> ids((size_t)s.nBody);
    std::vector<int> charOf((size_t)s.nBody, -1);
    std::vector<CharState> chars;
    for (int i = 0; i < s.nBody; i++)
    {
        const Op3Body& ob = s.bodies[i];
        if (ob.isChar)
        {
            // A capsule of the scene's size, centred on the character's position; 45 degree slope limit, 0.3 m steps, as the other engines' default humanoid.
            const Op3Shape& cap = s.shapes[ob.shape0];
            float radius = (float)cap.radius, halfCyl = 0.5f * (float)(cap.p[4] - cap.p[1]);
            CharacterVirtualSettings cvs;
            cvs.mShape = new CapsuleShape(halfCyl, radius);
            cvs.mUp = Vec3::sAxisY();
            cvs.mSupportingVolume = Plane(Vec3::sAxisY(), -radius);
            cvs.mMaxSlopeAngle = DegreesToRadians(45.0f);
            cvs.mMass = 70.0f;
            cvs.mMaxStrength = ob.cPush > 0 ? 500.0f : 0.0f;
            CharState cs; cs.ch = new CharacterVirtual(&cvs, RVec3((float)ob.pos[0], (float)ob.pos[1], (float)ob.pos[2]), Quat::sIdentity(), 0, &physics);
            cs.prev = cs.ch->GetPosition(); cs.vel = Vec3::sZero(); cs.vy = 0.0f;
            charOf[(size_t)i] = (int)chars.size(); chars.push_back(cs);
            continue;
        }
        ShapeRefC shape;
        if (ob.shapeN == 0) shape = new SphereShape(0.01f); // shapeless anchor bodies get a tiny sphere; their joints disable its collisions
        else if (ob.shapeN == 1) shape = MakeShape(s.shapes[ob.shape0]);
        else
        {
            StaticCompoundShapeSettings cs;
            for (int k = 0; k < ob.shapeN; k++) cs.AddShape(Vec3::sZero(), Quat::sIdentity(), MakeShape(s.shapes[ob.shape0 + k]));
            ShapeSettings::ShapeResult r = cs.Create();
            if (r.HasError()) { fprintf(stderr, "compound error: %s\n", r.GetError().c_str()); return 2; }
            shape = r.Get();
        }
        const Op3Shape& first = s.shapes[ob.shapeN > 0 ? ob.shape0 : 0];
        EMotionType mt = ob.type == 0 ? EMotionType::Static : ob.type == 1 ? EMotionType::Kinematic : EMotionType::Dynamic;
        BodyCreationSettings bcs(shape, RVec3((float)ob.pos[0], (float)ob.pos[1], (float)ob.pos[2]), Quat((float)ob.q[0], (float)ob.q[1], (float)ob.q[2], (float)ob.q[3]), mt, ob.type == 0 ? Layers::NON_MOVING : Layers::MOVING);
        bcs.mFriction = (float)s.friction[first.material]; bcs.mRestitution = (float)s.restitution[first.material];
        bcs.mLinearVelocity = V3(ob.v); bcs.mAngularVelocity = V3(ob.w);
        if (ob.bullet) bcs.mMotionQuality = EMotionQuality::LinearCast;
        if (jointed[i]) { bcs.mCollisionGroup.SetGroupFilter(filter); bcs.mCollisionGroup.SetGroupID(0); bcs.mCollisionGroup.SetSubGroupID((CollisionGroup::SubGroupID)i); }
        ids[i] = bi.CreateAndAddBody(bcs, (ob.type == 0 || ob.asleep) ? EActivation::DontActivate : EActivation::Activate);
    }
    for (int i = 0; i < s.nJoint; i++)
    {
        const Op3Joint& oj = s.joints[i];
        if (charOf[(size_t)oj.a] >= 0 || charOf[(size_t)oj.b] >= 0) continue;
        if (s.bodies[oj.a].type != 2 && s.bodies[oj.b].type != 2) continue;
        PointConstraintSettings ps; ps.mSpace = EConstraintSpace::LocalToBodyCOM; // bodies in the scenes have their centre of mass at the origin
        ps.mPoint1 = RVec3((float)oj.pa[0], (float)oj.pa[1], (float)oj.pa[2]); ps.mPoint2 = RVec3((float)oj.pb[0], (float)oj.pb[1], (float)oj.pb[2]);
        physics.AddConstraint(bi.CreateConstraint(&ps, ids[oj.a], ids[oj.b]));
    }
    std::vector<CarState> cars;
    for (int i = 0; i < s.nBody; i++)
        if (s.bodies[i].isVeh) { CarState c = MakeCar(physics, bi, ids[(size_t)i]); c.body = i; cars.push_back(c); }
    physics.OptimizeBroadPhase();

    // Wheel centres of every car, four wheels each, three doubles per wheel.
    std::vector<double> wheelRows(cars.size() * 12);
    auto wheels = [&]() { size_t k = 0; for (CarState& c : cars) for (uint w = 0; w < 4; w++) { RMat44 m = c.vc->GetWheelWorldTransform(w, Vec3::sAxisX(), Vec3::sAxisY()); RVec3 p = m.GetTranslation(); wheelRows[k++] = p.GetX(); wheelRows[k++] = p.GetY(); wheelRows[k++] = p.GetZ(); } };

    double* d = (double*)malloc((size_t)s.nBody * 14 * sizeof(double));
    double* stepNs = (double*)malloc((size_t)(s.steps + 1) * sizeof(double));
    FILE* dump = args.timing ? NULL : op3_dump_open(args.dump, s.nBody);
    if (dump) { snapshot(bi, ids, d, charOf, chars); wheels(); op3_dump_frame_x(dump, 0, d, s.nBody, wheelRows.data(), (int)wheelRows.size()); }

    const int collisionSteps = s.substeps > 0 ? s.substeps : 1; // the documented default: one collision step per update
    for (int step = 0; step < s.steps; step++)
    {
        for (int w = 0; w < s.nWake; w++)
            if (s.wakes[w].step == step) for (int i = 0; i < s.nBody; i++) if (s.bodies[i].type == 2 && charOf[(size_t)i] < 0) bi.ActivateBody(ids[i]);
        // Scripted drive as in the other drivers: throttle until the braking step, then full brake; a cosine-like triangle-wave slalom between the steering step and the braking step (docs/scenes.md).
        for (CarState& c : cars)
        {
            const Op3Body& ob = s.bodies[c.body];
            float throttle = 0.0f, steer = 0.0f, brake = 0.0f;
            if (step >= (int)ob.vBrakeFrom) brake = 1.0f;
            else
            {
                throttle = (float)ob.vThrottle;
                if (step >= (int)ob.vSteerFrom)
                {
                    double p = (step - ob.vSteerFrom) / ob.vPeriod + 0.25, f = p - std::floor(p);
                    steer = (float)(ob.vSteer * (f < 0.25 ? 4 * f : f < 0.75 ? 2 - 4 * f : 4 * f - 4));
                }
            }
            c.ctrl->SetDriverInput(throttle, steer, brake, 0.0f);
            if (throttle != 0.0f || brake != 0.0f) bi.ActivateBody(ids[(size_t)c.body]);
        }
        double t0 = op3_now_ns();
        // Scripted input as in the other drivers: walk along +x at the script's speed, reversing every period steps, jump every so many steps, stand still from the stop step; gravity is integrated
        // into a vertical speed held at a small downward value while supported. The character's own update then slides, steps and sticks to the floor.
        const float dtf = (float)s.dt;
        {
            size_t k = 0;
            for (int i = 0; i < s.nBody; i++)
            {
                if (charOf[(size_t)i] < 0) continue;
                const Op3Body& ob = s.bodies[i]; CharState& c = chars[k++];
                bool active = step < (int)ob.cStop;
                float walk = active ? (float)ob.cSpeed * ((((step / (int)ob.cPeriod) & 1) == 0) ? 1.0f : -1.0f) : 0.0f;
                bool grounded = c.ch->GetGroundState() == CharacterBase::EGroundState::OnGround;
                if (grounded && c.vy <= 0.0f) c.vy = (active && ob.cJump > 0 && step > 0 && (step + (int)(k - 1)) % (int)ob.cJump == 0) ? 5.0f : -1.0f;
                else c.vy -= (float)(-s.g[1]) * dtf;
                // Standing on a moving or turning body, the character is carried by its velocity (documented GetGroundVelocity).
                Vec3 ground = grounded ? c.ch->GetGroundVelocity() : Vec3::sZero();
                c.ch->SetLinearVelocity(ground + Vec3(walk, c.vy, 0.0f));
                CharacterVirtual::ExtendedUpdateSettings eus;
                eus.mStickToFloorStepDown = Vec3(0, -0.3f, 0);
                eus.mWalkStairsStepUp = Vec3(0, 0.3f, 0);
                c.prev = c.ch->GetPosition();
                c.ch->ExtendedUpdate(dtf, physics.GetGravity(), eus,
                    physics.GetDefaultBroadPhaseLayerFilter(Layers::MOVING), physics.GetDefaultLayerFilter(Layers::MOVING), BodyFilter(), ShapeFilter(), temp);
                RVec3 np = c.ch->GetPosition();
                c.vel = Vec3((float)((np.GetX() - c.prev.GetX()) / s.dt), (float)((np.GetY() - c.prev.GetY()) / s.dt), (float)((np.GetZ() - c.prev.GetZ()) / s.dt));
            }
        }
        physics.Update((float)s.dt, collisionSteps, &temp, &job);
        stepNs[step] = op3_now_ns() - t0;
        if (dump && ((step + 1) % s.dumpEvery == 0 || step + 1 == s.steps)) { snapshot(bi, ids, d, charOf, chars); wheels(); op3_dump_frame_x(dump, step + 1, d, s.nBody, wheelRows.data(), (int)wheelRows.size()); }
    }
    if (dump) fclose(dump);

    Op3QueryTimes qt; memset(&qt, 0, sizeof qt);
    if (s.nQuery > 0)
    {
        const NarrowPhaseQuery& npq = physics.GetNarrowPhaseQuery();
        for (int rep = 0; rep < s.queryReps; rep++)
        {
            bool measure = s.queryReps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                long hits = 0; int count = 0; double t0 = op3_now_ns();
                for (int i = 0; i < s.nQuery; i++)
                {
                    const Op3Query* q = &s.queries[i]; if (q->kind != kind) continue;
                    count++;
                    if (kind == 0)
                    {
                        RRayCast ray(RVec3((float)q->p[0], (float)q->p[1], (float)q->p[2]), Vec3((float)(q->p[3] * q->p[6]), (float)(q->p[4] * q->p[6]), (float)(q->p[5] * q->p[6])));
                        RayCastResult hit; if (npq.CastRay(ray, hit)) hits++;
                    }
                    else
                    {
                        Ref<SphereShape> sphere = new SphereShape((float)q->p[3]);
                        RMat44 at = RMat44::sTranslation(RVec3((float)q->p[0], (float)q->p[1], (float)q->p[2]));
                        if (kind == 1)
                        {
                            AllHitCollisionCollector<CollideShapeCollector> col; CollideShapeSettings cs;
                            npq.CollideShape(sphere, Vec3::sReplicate(1.0f), at, cs, RVec3::sZero(), col);
                            hits += (long)col.mHits.size();
                        }
                        else
                        {
                            RShapeCast sc = RShapeCast::sFromWorldTransform(sphere, Vec3::sReplicate(1.0f), at, Vec3((float)q->p[4], (float)q->p[5], (float)q->p[6]));
                            ShapeCastSettings scs; ClosestHitCollisionCollector<CastShapeCollector> col;
                            npq.CastShape(sc, scs, RVec3::sZero(), col); if (col.HadHit()) hits++;
                        }
                    }
                }
                if (measure) { qt.ns[kind] += op3_now_ns() - t0; qt.count[kind] += count; qt.hits[kind] = hits; }
            }
        }
    }
    char cfg[256];
    snprintf(cfg, sizeof cfg, "Distribution build, cross-platform determinism off, single precision, %d worker threads + caller, collision steps %d, default solver iterations, heightfields native", args.threads - 1, collisionSteps);
    op3_write_result(args.result, "jolt", "5.6.0", cfg, &s, args.threads, stepNs, s.nQuery > 0 ? &qt : NULL);

    UnregisterTypes();
    delete Factory::sInstance; Factory::sInstance = nullptr;
    return 0;
}
