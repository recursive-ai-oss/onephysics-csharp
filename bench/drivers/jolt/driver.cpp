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
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

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
        if (nz == nx)
        {
            HeightFieldShapeSettings s(h.data(), Vec3((float)p[3], (float)p[4], (float)p[5]), Vec3((float)cell, 1.0f, (float)cell), (uint32)nx);
            r = s.Create();
        }
        else
        {
            // Jolt's height field is square: a longer field is cut into square tiles that share their edge rows (or columns), joined in a static compound.
            int count = nx < nz ? nx : nz, span = count - 1, longer = nx < nz ? nz : nx;
            if ((longer - 1) % span != 0) { fprintf(stderr, "height field %d x %d cannot be tiled into squares\n", nx, nz); exit(2); }
            StaticCompoundShapeSettings comp;
            std::vector<float> tile((size_t)count * count);
            for (int t = 0; t < (longer - 1) / span; t++)
            {
                int x0 = nx > nz ? t * span : 0, z0 = nx > nz ? 0 : t * span; // sample offset of the tile
                for (int z = 0; z < count; z++) for (int x = 0; x < count; x++) tile[(size_t)z * count + x] = h[(size_t)(z0 + z) * nx + x0 + x];
                HeightFieldShapeSettings s(tile.data(), Vec3((float)(p[3] + x0 * cell), (float)p[4], (float)(p[5] + z0 * cell)), Vec3((float)cell, 1.0f, (float)cell), (uint32)count);
                ShapeSettings::ShapeResult tr = s.Create();
                if (tr.HasError()) { fprintf(stderr, "shape error: %s\n", tr.GetError().c_str()); exit(2); }
                comp.AddShape(Vec3::sZero(), Quat::sIdentity(), tr.Get());
            }
            r = comp.Create();
        }
    }
    else { fprintf(stderr, "unknown shape %s\n", sh.kind); exit(2); }
    if (r.HasError()) { fprintf(stderr, "shape error: %s\n", r.GetError().c_str()); exit(2); }
    ShapeRefC shape = r.Get();
    if (offset.LengthSq() > 0.0f) shape = RotatedTranslatedShapeSettings(offset, Quat::sIdentity(), shape).Create().Get();
    return shape;
}

// Scripted characters are Jolt CharacterVirtual objects (documented class); their dump rows hold the position and the displacement per second of the last step.
struct CharState { Ref<CharacterVirtual> ch; RVec3 prev; Vec3 vel; float vy; float feet; /* distance from the scene body centre down to the character origin (its feet) */ };

static void snapshot(BodyInterface& bi, const std::vector<BodyID>& ids, double* d, const std::vector<int>& charOf, const std::vector<CharState>& chars)
{
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (charOf[i] >= 0)
        {
            const CharState& c = chars[(size_t)charOf[i]]; RVec3 p = c.ch->GetPosition(); double* o = d + i * 14;
            o[0] = p.GetX(); o[1] = p.GetY() + c.feet; o[2] = p.GetZ(); o[3] = o[4] = o[5] = 0; o[6] = 1;
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

// Scripted vehicles are Jolt VehicleConstraint objects (documented classes) with the controller that matches the scene's preset. Everything a preset does not name keeps the library's default (tyre curves, torque curve,
// clutch). Settings taken from the class reference pages of VehicleConstraintSettings, WheelSettings, WheelSettingsWV, WheelSettingsTV, WheeledVehicleControllerSettings, TrackedVehicleControllerSettings,
// VehicleTrackSettings, MotorcycleControllerSettings and OffsetCenterOfMassShape.
enum VehKind { VK_CAR = 0, VK_TANK = 1, VK_BIKE = 2 };
struct VehState { Ref<VehicleConstraint> vc; VehicleController* ctrl; VehKind kind; int body; };

// How far the centre of mass is moved down, per preset. The class reference of OffsetCenterOfMassShape names "shift the center of mass of a vehicle down to improve its handling" as its use. Cars and tanks keep the
// centre of the box (the scene's centre of mass); the motorcycle's goes down to the bottom of its box, which brought its lean tracking within the scene's threshold (leanError 0.27 rad without, 0.02 with).
static float ComDrop(VehKind k) { return k == VK_BIKE ? 0.2f : 0.0f; }

static void StableForcePoint(WheelSettings* w)
{
    // The class reference of WheelSettings: forces applied at one fixed point of the body are more stable than forces applied at the contact point.
    w->mEnableSuspensionForcePoint = true;
    w->mSuspensionForcePoint = w->mPosition + w->mSuspensionDirection * (0.5f * (w->mSuspensionMinLength + w->mSuspensionMaxLength)); // the wheel centre in its neutral pose
}

static void SetWheelAxes(WheelSettings* w, bool bike)
{
    w->mSuspensionDirection = Vec3(0, -1, 0); w->mSteeringAxis = Vec3(0, 1, 0); w->mWheelUp = Vec3(0, 1, 0); w->mWheelForward = Vec3(0, 0, 1);
    (void)bike; // a bike's steering axis and wheel up are -mSuspensionDirection, which is +y here as well
}

static VehState MakeVehicle(PhysicsSystem& physics, BodyID id, VehKind kind)
{
    VehicleConstraintSettings vcs;
    vcs.mUp = Vec3::sAxisY(); vcs.mForward = Vec3::sAxisZ();
    VehState c; c.kind = kind; c.body = 0;
    if (kind == VK_CAR)
    {
        // docs/spec/scenes.md "car": four wheels of 0.34 m radius 0.8 m either side and 1.3 m ahead and behind the centre, suspension 0.05 to 0.5 m at 1.5 Hz and damping 0.6, front wheels steering to 0.5 rad, rear handbrake,
        // 400 N m engine between 1000 and 6500 rpm, six-speed automatic, one rear differential of ratio 3.4, anti-roll bars of 20 kN/m.
        const float xs[4] = { -0.8f, 0.8f, -0.8f, 0.8f }, zs[4] = { 1.3f, 1.3f, -1.3f, -1.3f };
        for (int k = 0; k < 4; k++)
        {
            WheelSettingsWV* w = new WheelSettingsWV;
            w->mPosition = Vec3(xs[k], 0.0f, zs[k]);
            SetWheelAxes(w, false);
            w->mSuspensionMinLength = 0.05f; w->mSuspensionMaxLength = 0.5f;
            w->mSuspensionSpring.mFrequency = 1.5f; w->mSuspensionSpring.mDamping = 0.6f;
            w->mRadius = 0.34f; w->mWidth = 0.25f;
            w->mMaxSteerAngle = k < 2 ? 0.5f : 0.0f;
            w->mMaxBrakeTorque = 3000.0f; w->mMaxHandBrakeTorque = k < 2 ? 0.0f : 4000.0f;
            w->mInertia = 1.0f; w->mAngularDamping = 0.2f;
            StableForcePoint(w);
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
    }
    else if (kind == VK_TANK)
    {
        // docs/spec/scenes.md "tank": five road wheels of 0.45 m radius and 0.5 m width a side, 1.2 m apart, 1.3 m either side of the centre and 0.2 m below it, suspension 0.1 to 0.45 m at 2 Hz and damping 0.7.
        // The left track (the x < 0 wheels) takes wheels 0 to 4, the right track 5 to 9; the first wheel of each track is the driven one. Engine, transmission and track settings keep the library's defaults.
        TrackedVehicleControllerSettings* cs = new TrackedVehicleControllerSettings;
        for (int side = 0; side < 2; side++)
            for (int i = 0; i < 5; i++)
            {
                WheelSettingsTV* w = new WheelSettingsTV;
                w->mPosition = Vec3(side == 0 ? -1.3f : 1.3f, -0.2f, 2.4f - 1.2f * (float)i);
                SetWheelAxes(w, false);
                w->mSuspensionMinLength = 0.1f; w->mSuspensionMaxLength = 0.45f;
                w->mSuspensionSpring.mFrequency = 2.0f; w->mSuspensionSpring.mDamping = 0.7f;
                w->mRadius = 0.45f; w->mWidth = 0.5f;
                StableForcePoint(w);
                cs->mTracks[side == 0 ? (int)ETrackSide::Left : (int)ETrackSide::Right].mWheels.push_back((uint)vcs.mWheels.size());
                vcs.mWheels.push_back(w);
            }
        cs->mTracks[(int)ETrackSide::Left].mDrivenWheel = 0; cs->mTracks[(int)ETrackSide::Right].mDrivenWheel = 5;
        vcs.mController = cs;
    }
    else
    {
        // docs/spec/scenes.md "bike": 0.32 m wheels 1.4 m apart, 0.1 m above the centre, suspension 0.1 to 0.4 m at 2.5 Hz and damping 0.7, front wheel steering to 0.5 rad, 90 N m engine between 1200 and 10000 rpm, five-speed
        // automatic box shifting up at 8000 and down at 3500 rpm, rear wheel drive through one differential, brakes 500 and 400 N m, lean controller on with a 40 degree limit.
        for (int k = 0; k < 2; k++)
        {
            WheelSettingsWV* w = new WheelSettingsWV;
            w->mPosition = Vec3(0.0f, 0.1f, k == 0 ? 0.7f : -0.7f);
            SetWheelAxes(w, true);
            w->mSuspensionMinLength = 0.1f; w->mSuspensionMaxLength = 0.4f;
            w->mSuspensionSpring.mFrequency = 2.5f; w->mSuspensionSpring.mDamping = 0.7f;
            w->mRadius = 0.32f; w->mWidth = 0.16f;
            w->mMaxSteerAngle = k == 0 ? 0.5f : 0.0f;
            w->mMaxBrakeTorque = k == 0 ? 500.0f : 400.0f; w->mMaxHandBrakeTorque = 0.0f;
            w->mInertia = 0.8f; w->mAngularDamping = 0.05f;
            StableForcePoint(w);
            vcs.mWheels.push_back(w);
        }
        MotorcycleControllerSettings* cs = new MotorcycleControllerSettings;
        cs->mEngine.mMaxTorque = 90.0f; cs->mEngine.mMinRPM = 1200.0f; cs->mEngine.mMaxRPM = 10000.0f; cs->mEngine.mInertia = 0.1f;
        cs->mTransmission.mMode = ETransmissionMode::Auto;
        cs->mTransmission.mGearRatios = { 2.8f, 1.9f, 1.4f, 1.1f, 0.9f };
        cs->mTransmission.mReverseGearRatios = { -2.8f };
        cs->mTransmission.mShiftUpRPM = 8000.0f; cs->mTransmission.mShiftDownRPM = 3500.0f; cs->mTransmission.mSwitchTime = 0.1f;
        cs->mDifferentials.resize(1);
        cs->mDifferentials[0].mLeftWheel = -1; cs->mDifferentials[0].mRightWheel = 1; cs->mDifferentials[0].mDifferentialRatio = 3.0f; cs->mDifferentials[0].mEngineTorqueRatio = 1.0f;
        cs->mMaxLeanAngle = DegreesToRadians(40.0f);
        vcs.mController = cs;
    }
    c.vc = new VehicleConstraint(*physics.GetBodyLockInterfaceNoLock().TryGetBody(id), vcs);
    // Wheels are tested with a sphere cast of the wheel's radius (VehicleCollisionTesterCastSphere): on the scenes' terrain the default cylinder cast gave a suspension jitter of 0.029 against 0.008 for the sphere.
    c.vc->SetVehicleCollisionTester(new VehicleCollisionTesterCastSphere(Layers::MOVING, kind == VK_CAR ? 0.34f : kind == VK_TANK ? 0.45f : 0.32f));
    physics.AddConstraint(c.vc);
    physics.AddStepListener(c.vc);
    c.ctrl = c.vc->GetController();
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
            // The class reference of CharacterBaseSettings asks for a shape whose bottom is at the origin, so the capsule is lifted by half its height and the character stands at the feet of the scene body.
            cvs.mShape = RotatedTranslatedShapeSettings(Vec3(0, halfCyl + radius, 0), Quat::sIdentity(), new CapsuleShape(halfCyl, radius)).Create().Get();
            cvs.mUp = Vec3::sAxisY();
            cvs.mSupportingVolume = Plane(Vec3::sAxisY(), -radius);
            cvs.mMaxSlopeAngle = DegreesToRadians(45.0f);
            cvs.mMass = 70.0f;
            cvs.mMaxStrength = ob.cPush > 0 ? 500.0f : 0.0f;
            CharState cs; cs.ch = new CharacterVirtual(&cvs, RVec3((float)ob.pos[0], (float)ob.pos[1] - (halfCyl + radius), (float)ob.pos[2]), Quat::sIdentity(), 0, &physics);
            cs.feet = halfCyl + radius;
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
        if (ob.isVeh)
        {
            float drop = ComDrop(!strcmp(ob.vPreset, "tank") ? VK_TANK : !strcmp(ob.vPreset, "bike") ? VK_BIKE : VK_CAR);
            if (drop != 0.0f) shape = OffsetCenterOfMassShapeSettings(Vec3(0, -drop, 0), shape).Create().Get();
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
    struct Drive { SwingTwistConstraint* con; int joint; };
    std::vector<Drive> drives;
    for (int i = 0; i < s.nJoint; i++)
    {
        const Op3Joint& oj = s.joints[i];
        if (charOf[(size_t)oj.a] >= 0 || charOf[(size_t)oj.b] >= 0) continue;
        if (s.bodies[oj.a].type != 2 && s.bodies[oj.b].type != 2) continue;
        if (!strcmp(oj.kind, "swingtwist"))
        {
            // The scene's joint frame has its twist axis along u (here +y) and the shortest-arc rotation from x to u, so its y axis is world -x and its z axis world z (docs/spec/scenes.md); the Jolt constraint space
            // is (twist, plane, normal) = the same three axes. The class reference of SwingTwistConstraintSettings (with its picture) measures the normal half cone angle as the tilt of the twist axis towards the
            // normal axis, a rotation about the plane axis: the scene's swing-y limit; the plane half cone angle is the swing-z limit.
            SwingTwistConstraintSettings st; st.mSpace = EConstraintSpace::LocalToBodyCOM;
            st.mPosition1 = RVec3((float)oj.pa[0], (float)oj.pa[1], (float)oj.pa[2]); st.mPosition2 = RVec3((float)oj.pb[0], (float)oj.pb[1], (float)oj.pb[2]);
            st.mTwistAxis1 = st.mTwistAxis2 = Vec3::sAxisY(); st.mPlaneAxis1 = st.mPlaneAxis2 = Vec3(-1, 0, 0);
            st.mTwistMinAngle = (float)oj.lower; st.mTwistMaxAngle = (float)oj.upper;
            st.mNormalHalfConeAngle = (float)oj.swingY; st.mPlaneHalfConeAngle = (float)oj.swingZ;
            if (oj.hasDrive)
            {
                // Absolute gains: the documented spring mode with stiffness (N m/rad) and damping (N m s/rad), torque = stiffness * angle error + damping * rate error, used as they are. The equivalent frequency and
                // damping ratio for an effective inertia I would be sqrt(k / I) / 2 pi and c / (2 sqrt(k I)); the mode makes the conversion unnecessary.
                for (MotorSettings* m : { &st.mSwingMotorSettings, &st.mTwistMotorSettings })
                {
                    m->mSpringSettings = SpringSettings(ESpringMode::StiffnessAndDamping, (float)oj.driveK, (float)oj.driveC);
                    m->SetTorqueLimit((float)oj.driveMax);
                }
            }
            Ref<Constraint> con = bi.CreateConstraint(&st, ids[oj.a], ids[oj.b]);
            physics.AddConstraint(con);
            if (oj.hasDrive)
            {
                SwingTwistConstraint* stc = static_cast<SwingTwistConstraint*>(con.GetPtr());
                stc->SetSwingMotorState(EMotorState::Position); stc->SetTwistMotorState(EMotorState::Position);
                drives.push_back({ stc, i });
            }
            continue;
        }
        PointConstraintSettings ps; ps.mSpace = EConstraintSpace::LocalToBodyCOM; // bodies in the scenes have their centre of mass at the origin
        ps.mPoint1 = RVec3((float)oj.pa[0], (float)oj.pa[1], (float)oj.pa[2]); ps.mPoint2 = RVec3((float)oj.pb[0], (float)oj.pb[1], (float)oj.pb[2]);
        physics.AddConstraint(bi.CreateConstraint(&ps, ids[oj.a], ids[oj.b]));
    }
    std::vector<VehState> cars;
    for (int i = 0; i < s.nBody; i++)
        if (s.bodies[i].isVeh)
        {
            const char* pr = s.bodies[i].vPreset;
            VehState c = MakeVehicle(physics, ids[(size_t)i], !strcmp(pr, "tank") ? VK_TANK : !strcmp(pr, "bike") ? VK_BIKE : VK_CAR);
            c.body = i; cars.push_back(c);
        }
    physics.OptimizeBroadPhase();

    // Wheel centres of every vehicle, three doubles per wheel, in the vehicle's wheel order.
    size_t wheelCount = 0; for (VehState& c : cars) wheelCount += c.vc->GetWheels().size();
    std::vector<double> wheelRows(wheelCount * 3);
    auto wheels = [&]() { size_t k = 0; for (VehState& c : cars) for (uint w = 0; w < (uint)c.vc->GetWheels().size(); w++) { RMat44 m = c.vc->GetWheelWorldTransform(w, Vec3::sAxisX(), Vec3::sAxisY()); RVec3 p = m.GetTranslation(); wheelRows[k++] = p.GetX(); wheelRows[k++] = p.GetY(); wheelRows[k++] = p.GetZ(); } };

    double* d = (double*)malloc((size_t)s.nBody * 14 * sizeof(double));
    double* stepNs = (double*)malloc((size_t)(s.steps + 1) * sizeof(double));
    FILE* dump = args.timing ? NULL : op3_dump_open(args.dump, s.nBody);
    if (dump) { snapshot(bi, ids, d, charOf, chars); wheels(); op3_dump_frame_x(dump, 0, d, s.nBody, wheelRows.data(), (int)wheelRows.size()); }

    // The documented default is one collision step per update. With vehicles in the scene the driver uses four: at one step 68 of the 100 cars of vehicles-3d rolled over on gentle terrain (two steps: 1; four: none), and
    // 20 of 20 motorcycles fell. Collision steps are the library's documented knob for simulating at a higher frequency.
    bool hasVehicles = false; for (int i = 0; i < s.nBody; i++) hasVehicles |= s.bodies[i].isVeh != 0;
    const int collisionSteps = s.substeps > 0 ? s.substeps : hasVehicles ? 4 : 1;
    for (int step = 0; step < s.steps; step++)
    {
        for (int w = 0; w < s.nWake; w++)
            if (s.wakes[w].step == step) for (int i = 0; i < s.nBody; i++) if (s.bodies[i].type == 2 && charOf[(size_t)i] < 0) bi.ActivateBody(ids[i]);
        // Scripted drive as in the other drivers: throttle until the braking step, then full brake; a cosine-like triangle-wave slalom between the steering step and the braking step (docs/spec/scenes.md).
        for (VehState& c : cars)
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
            // The scene's positive steering turns towards +x. With +y up and +z forward, +x is to the vehicle's left, and Jolt's steering input is positive to the right.
            if (c.kind == VK_TANK)
            {
                // Tracks as in the scene: left (x < 0) track command throttle + steer, right throttle - steer, both divided by the larger magnitude when it passes 1. The forward input carries the common magnitude and
                // the track ratios the split, as the class reference of TrackedVehicleController describes them (forward input; extra multipliers of each track's rotation rate between -1 and 1).
                float l = throttle + steer, r = throttle - steer, m = std::max(std::fabs(l), std::fabs(r));
                if (m > 1.0f) { l /= m; r /= m; m = 1.0f; }
                float lr = m > 0.0f ? l / m : 1.0f, rr = m > 0.0f ? r / m : 1.0f; // with no throttle the ratios stay at full: zero ratios made the wheel transforms non-finite while braking
                static_cast<TrackedVehicleController*>(c.ctrl)->SetDriverInput(m, lr, rr, brake);
            }
            else if (c.kind == VK_BIKE) static_cast<MotorcycleController*>(c.ctrl)->SetDriverInput(throttle, -steer, brake, 0.0f);
            else static_cast<WheeledVehicleController*>(c.ctrl)->SetDriverInput(throttle, -steer, brake, 0.0f);
            if (throttle != 0.0f || brake != 0.0f) bi.ActivateBody(ids[(size_t)c.body]);
        }
        // Powered ragdolls: the scene's pose drive, a target rotation about one axis of the joint frame with angle amp times a triangle wave 0 to 1 to 0 over period steps (docs/spec/scenes.md), in constraint space.
        for (const Drive& d : drives)
        {
            const Op3Joint& oj = s.joints[d.joint];
            double ph = (double)(step % oj.drivePeriod) / oj.drivePeriod, tri = ph < 0.5 ? 2 * ph : 2 - 2 * ph;
            // A body that went to sleep does not follow a changed motor target; the pose drive keeps its bodies awake.
            bi.ActivateBody(ids[(size_t)oj.a]); bi.ActivateBody(ids[(size_t)oj.b]);
            // The motor target is in Jolt's constraint space, whose axes were found by experiment (drive a known rotation, read the relative orientation back): x = twist axis, y = the normal axis (plane axis x twist axis,
            // opposite to the cross product the class reference's picture suggests), z = the plane axis. Scene joint-frame axes (twist x, y = plane axis, z) map to (x, -z, y).
            Vec3 ax((float)oj.driveAx, -(float)oj.driveAz, (float)oj.driveAy);
            d.con->SetTargetOrientationCS(Quat::sRotation(ax.Normalized(), (float)(oj.driveAmp * tri)));
        }
        // Water: Jolt's buoyancy takes a factor (1 = neutral), not a density, so each dynamic body gets fluid density / its own density, with the documented typical drags (0.5 linear, 0.01 angular).
        if (s.hasFluid)
        {
            for (int i = 0; i < s.nBody; i++)
            {
                const Op3Body& ob = s.bodies[i];
                if (ob.type != 2 || ob.shapeN == 0 || !bi.IsActive(ids[(size_t)i])) continue;
                double bodyDensity = s.shapes[ob.shape0].density;
                bi.ApplyBuoyancyImpulse(ids[(size_t)i], RVec3(0, s.fluidY, 0), Vec3(0, 1, 0), (float)(s.fluidDensity / bodyDensity), 0.5f, 0.01f,
                    Vec3((float)s.fluidFlow[0], (float)s.fluidFlow[1], (float)s.fluidFlow[2]), physics.GetGravity(), (float)s.dt);
            }
        }
        double t0 = op3_now_ns();
        // Scripted input as in the other drivers: walk along +x at the script's speed, reversing every period steps, jump every so many steps, stand still from the stop step; gravity is integrated
        // into a vertical speed; while supported and not moving away from the ground it is delta time * gravity, as the class reference of ExtendedUpdate gives it. The character's own update then slides, steps and sticks to the floor.
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
                if (grounded && c.vy <= 0.0f) c.vy = (active && ob.cJump > 0 && step > 0 && (step + (int)(k - 1)) % (int)ob.cJump == 0) ? 5.0f : (float)s.g[1] * dtf; // the documented recipe: on the ground and not moving away from it, the vertical speed is delta time * gravity
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
