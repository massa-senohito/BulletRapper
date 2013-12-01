#pragma once
#include "btBulletDynamicsCommon.h"
#include "TypeOfShape.h"
#include <list>

//static bool OnContacted(btManifoldPoint& cp,
//    const btCollisionObject* colObj0,
//    int partId0,
//    int index0,
//    const btCollisionObject* colObj1,
//    int partId1,
//    int index1);


void myTickCallback(btDynamicsWorld *world, btScalar timeStep);
class PhysicsEngine
{
    btDiscreteDynamicsWorld *dynamicsWorld;
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btDbvtBroadphase* broadphase;btSequentialImpulseConstraintSolver* solver;
    typedef std::list<btCollisionShape*> ShapeList;
    ShapeList* collisionShapes;
    //btAlignedObjectArray<btRigidBody*> bodies;
public:
    PhysicsEngine(const btVector3&);

    void SetSensor(const btVector3& pos,const btVector3& size);

    btRigidBody* CreateBody(btScalar ,TypeOfShape );
    void AddBox(const btVector3&);
    void CreatePlane(const btVector3&,const float);
    void Step();
    ~PhysicsEngine(void);
};

