#pragma once
#include "BulletSoftBody\btSoftBody.h"
#include "BulletSoftBody\btSoftRigidDynamicsWorld.h"
//#include "LinearMath\btVector3.h"
#include "btBulletDynamicsCommon.h"
#include <list>
#include <functional>
void RenderVertice(btSoftRigidDynamicsWorld* world);
typedef btManifoldPoint Manipoint;
typedef btDefaultCollisionConfiguration CollisionConf;
typedef btCollisionDispatcher Dispat;
typedef btDbvtBroadphase Broad;
typedef btAxisSweep3 Sweep3;
class SoftBodyPhysic
{
    btSoftRigidDynamicsWorld *dynamicsWorld;
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btDbvtBroadphase* broadphase;btSequentialImpulseConstraintSolver* solver;
    typedef std::list<btCollisionShape*> ShapeList;
    ShapeList* collisionShapes;
    btAxisSweep3* phase;
    typedef std::function<void(btSoftBody*,btSoftBody*)> CollideCallBack;
    typedef btScalar Scalar;
    CollideCallBack* callback;
public:
    SoftBodyPhysic(const btVector3&);
    void        Step();
    Scalar*     MakeFloor(const btVector3&,float);
    btSoftBody* getOneSoft();
    void        RenderVertice() const;
    void        ColisionCallBack(CollideCallBack*);
    Scalar*     MakeBox(const btVector3& pos,bool,float scale);
    std::list<const btVector3*> getOrientations();
    std::list<const btVector3*> getVeros();
    btCollisionObject& MakeSensor(btCollisionObject&);
    int         getObjectCount();
    void        MakeShape(const btVector3&,bool);
    ~SoftBodyPhysic(void);
};

