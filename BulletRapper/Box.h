#pragma once
#include "typeofshape.h"
#include "BulletCollision\CollisionShapes\btBoxShape.h"
#include "LinearMath\btVector3.h"
template<typename T>
class Box :
    public TypeOfShape
{
    T width;
    T height;
    T depth;
public:
    Box(T,T,T);
    ~Box();
    void CreateShape(float,const btVector3*);
};

