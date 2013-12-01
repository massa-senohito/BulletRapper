#include "ExportDLL.h"
#include "SoftBodyPhysic.h"
SoftBodyPhysic* phy=nullptr;
void Init(float y){
    phy=new SoftBodyPhysic(btVector3(0,y,0));
}
void MakeSpere(float x,float y,float z,bool isStatic){
    if(phy==nullptr){Init(0);}
    phy->MakeShape(btVector3(x,y,z),isStatic);

}
int ObjectCount(){
    return phy->getObjectCount();
}
void MakeBox(Vector3 pos,float scale,bool isStatic,float* mat){
    auto tmp= (phy->MakeBox(btVector3(pos.x,pos.y,pos.z),isStatic,scale));
    memcpy_s(mat,sizeof(float)*16, tmp,sizeof(float)*16);
    delete tmp;
}
void MakeFloor(Vector3 pos,float scale,float* mat){
    auto tmp= (phy->MakeFloor(btVector3(pos.x,pos.y,pos.z),scale));
    memcpy_s(mat,sizeof(float)*16, tmp,sizeof(float)*16);
    delete tmp;
}
void Step(){
    phy->Step();
}
void Render(){
    phy->RenderVertice();
}
Vector3 makeV3(float x,float y,float z){
    Vector3 v;
    v.x=x;v.y=y;v.z=z;
    return v;
}
Vector3 makeV3(const btVector3& v){
    return makeV3(v.getX(),v.getY(),v.getZ());
}
Vector3 Orientation(){
    
    auto soft=phy->getOneSoft();
    if(soft==nullptr)return makeV3(99,99,99);
    return makeV3(    soft->getWorldTransform().getOrigin());
}
Vector3* Vertice(){
  
    auto soft=phy->getOneSoft();
    if(soft==nullptr)return nullptr;

    btSoftBody::tNodeArray&   nodes(soft->m_nodes);
    int size=nodes.size();
    auto vs=static_cast<Vector3*>( malloc(sizeof(Vector3)*size));
    for(int i=0;i<size;i++){

        auto n=nodes.at(i);
        vs[i]=makeV3(n.m_x);
    }
    return vs;
}
void Quit(){
    delete phy;
}