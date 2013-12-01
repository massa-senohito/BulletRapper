#pragma once

#include "PhysicsEngine.h"

void glDraw();
class ContactTest
{
    btRigidBody* body;
    PhysicsEngine* eng;
public:
    ContactTest(void);
    void RigidTest();
    void SoftTest();
    ~ContactTest(void);
};

