#include "Box.h"


template<typename T>
Box<T>::Box(T width,T height,T depth)
{
this->width=width;this->height=height;this->depth=depth;
}
template<typename T>
void Box<T>::CreateShape(float mass,const btVector3* localinertia){
    auto shape=new btBoxShape(btVector3(this->width,this->height,this->depth));
    //shape->
    
}

template<typename T>
Box<T>::~Box(void)
{

}
