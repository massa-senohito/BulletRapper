#include "SoftBodyPhysic.h"

#include <algorithm>
#include "freeglut.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "DebugDrawer.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
//todo PhysicsEngine���p������Ƃ��ǂ��Ȃ邩�������Ă݂�
struct
{
    void DrawPoint(const btVector3& pos) {glBegin(GL_POINTS); glVertex3f( pos.x(),pos.y(),pos.z());glEnd();}
    void DrawLine(const btVector3& a,const btVector3& b) {
        glColor3d(1,0,0);
        glBegin(GL_LINE);
        glVertex3f(a.x(),a.y(),a.z()); 
        glVertex3f(b.x(),b.y(),b.z()); 
        glEnd();
    }
    void DrawTriangle(const btVector3& a,const btVector3& b,const btVector3& c) {
        glColor3d(0,1,0);
        glBegin(GL_TRIANGLE_STRIP);
        glVertex3f(a.x(),a.y(),a.z()); 
        glVertex3f(b.x(),b.y(),b.z()); 
        glVertex3f(c.x(),c.y(),c.z()); 
        glEnd();
    }
} *mygfx;
#include "RigidBodyPhysic.h"
DebugDrawer drawer;
SoftBodyPhysic::SoftBodyPhysic(const btVector3& grav):collisionConfiguration(new CollisionConf)
{
    dispatcher = new	Dispat(collisionConfiguration);
    //�}���`�X���b�h�Ȃ�ʂ̃R���X�g���N�^���g���Ă�
    //overlappingPairCache  
    broadphase=new Broad();
    
    btVector3 worldBoundsMin(-1000,-1000,-1000);
    btVector3 worldBoundsMax(1000,1000,1000);
    phase=new Sweep3(worldBoundsMin,worldBoundsMax,1024);
    
    solver = new btSequentialImpulseConstraintSolver;
    collisionShapes=new ShapeList;
    dynamicsWorld = 
        new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(grav);
    dynamicsWorld->setDebugDrawer(&drawer);
}
//4�r������ē���炵��
//����������ƃN���X�ɕ�����ׂ����Ȃ��A�G���Ȍ^�ˑ��������Ă���

void SoftBodyPhysic::Step(){
    dynamicsWorld->stepSimulation(1.f/60.f);
}
int SoftBodyPhysic::getObjectCount()
{
    return dynamicsWorld->getCollisionObjectArray().size();
}
std::list<const btVector3*> SoftBodyPhysic::getOrientations(){
    std::list<const btVector3*> xs=std::list<const btVector3*>();

    auto arr=dynamicsWorld->getCollisionObjectArray();
    
    for(int i=0;i<arr.size();i++){
        auto ob=arr.at(i);
        xs.push_back(&( ob->getWorldTransform().getOrigin()));
    }
    
    return xs;
}
std::list<const btVector3*> SoftBodyPhysic::getVeros(){
  
    std::list<const btVector3*> xs=std::list<const btVector3*>();
    auto arr=dynamicsWorld->getSoftBodyArray();
    for(int i=0;i<arr.size();i++){
        auto ob=arr.at(i);
        //xs.push_back
        //    (&( ob->m_nodes));�I�u�W�F�N�g�̃m�[�h������
        //ob->refine(
	//ob->cutLink
    }
    
    return xs;
}
btScalar* SoftBodyPhysic::MakeFloor(const btVector3& pos,float scale){
    //�@�������̐ݒ�
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
    //btSoftBody* body=btSoftBodyHelpers::Create
    //    ((dynamicsWorld->getWorldInfo()),pos,btVector3(1,1,1)*3,128);
    //groundShape->calculateLocalInertia(
        btVector3 localInertia(0,0,0);
        btScalar mass(0);
        btTransform transform(btQuaternion(0,0,0,1),btVector3(0,-1,0));

    auto mat=new btScalar[16];
    transform.getOpenGLMatrix(mat);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    auto body= new btRigidBody(rbInfo);
    
    //body->setFriction(1);

    dynamicsWorld->addRigidBody(body);
    return mat;
}

btScalar* SoftBodyPhysic::MakeBox(const btVector3& pos,bool isStatic,float scale)
{
    btBoxShape* box=new btBoxShape(btVector3(scale,scale,scale));

        btVector3 localInertia(0,0,0);
        btScalar mass(1);
        if(isStatic)mass=0;
        btTransform transform(btQuaternion(0,0,0,1),pos);
        
        if(!isStatic)box->calculateLocalInertia(mass,localInertia);
    auto mat=new btScalar[16];
    transform.getOpenGLMatrix(mat);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,box,localInertia);
    auto body= new btRigidBody(rbInfo);
    
    dynamicsWorld->addRigidBody(body);
    return mat;
}
struct	ImplicitSphere : btSoftBody::ImplicitFn
{
	btVector3	center;
	btScalar	sqradius;
	ImplicitSphere() {}
	ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
	btScalar	Eval(const btVector3& x)
	{
		return((x-center).length2()-sqradius);
	}
};
class TesImplicit: public btSoftBody::ImplicitFn{
public:
    TesImplicit(){
                                        //���x
	//�����炭���ꂼ��̒��_�ɂ���accuracy�Ɣ�ׂď������Ƃ��낪�ؒf�����
    }
    btScalar Eval(const btVector3& x);
};
btScalar TesImplicit::Eval(const btVector3& x){
  return 2;
}
void SoftBodyPhysic::MakeShape(const btVector3& pos,bool isStatic){
    //�傫���ƒ��_��
    btSoftBody* soft=btSoftBodyHelpers::CreateEllipsoid
        ((dynamicsWorld->getWorldInfo()),pos,btVector3(1,1,1)*3,128);
    
    auto mate=soft->m_materials[0];
    mate->m_kLST =0.1;
    soft->m_cfg.kDF=1;
    soft->m_cfg.kDP=0.001;
    soft->m_cfg.kPR=2500;
    //soft->m_cfg.kL
    
//    this->collisionShapes->push_back(sp);
    auto conf=btSoftBodyRigidBodyCollisionConfiguration();
    if(!isStatic) soft->setTotalMass(30,true);
    soft->setMass(48,0);
    //soft->cutLink(48,49,0.5);
    auto imp=new TesImplicit();
    soft->refine(imp,3,true);
    delete imp;
    dynamicsWorld->addSoftBody(soft);
}
void SoftBodyPhysic::RenderVertice()const{
    //auto dynamicsWorld->getCollisionObjectArray();
    dynamicsWorld->debugDrawWorld();
    /*
    btSoftBodyArray& softbodies( dynamicsWorld->getSoftBodyArray() );
    for(int i=0;i<softbodies.size();++i)
    {
    btSoftBody*            softbody(softbodies[i]);

    // ���ꂼ��̃{�f�B�͒��_�̔z��������Ă���
    btSoftBody::tNodeArray&   nodes(softbody->m_nodes);

    // �G�b�W�@�i�����N�⋗���W���C���g�j
    btSoftBody::tLinkArray&   links(softbody->m_links);

    // And finally, faces (triangles)                                
    btSoftBody::tFaceArray&   faces(softbody->m_faces);

    // Then, you can draw vertices...       
    // Node::m_x => �ʒu             
    // Node::m_n => �@�� (if meaningful)    
    for(int j=0;j<nodes.size();++j)
    {
    mygfx->DrawPoint(nodes[j].m_x);
    }

    // Or �G�b�W (���[�v�̂���)                
    // Link::m_n[2] => �m�[�h�ւ̃|�C���^    
    for(int j=0;j<links.size();++j)
    {
    btSoftBody::Node*   node_0=links[j].m_n[0];
    btSoftBody::Node*   node_1=links[j].m_n[1];
    mygfx->DrawLine(node_0->m_x,node_1->m_x);

    // Or if you need indices...       
    const int indices[]={   int(node_0-&nodes[0]),
    int(node_1-&nodes[0])};
    }

    // And even faces                   
    // Face::m_n[3] -> �m�[�h�ւ̃|�C���^  
    for(int j=0;j<faces.size();++j)
    {
    btSoftBody::Node*   node_0=faces[j].m_n[0];
    btSoftBody::Node*   node_1=faces[j].m_n[1];
    btSoftBody::Node*   node_2=faces[j].m_n[2];
    mygfx->DrawTriangle(node_0->m_x,node_1->m_x,node_2->m_x);

    // Or if you need indices...       
    const int indices[]={   int(node_0-&nodes[0]),
    int(node_1-&nodes[0]),
    int(node_2-&nodes[0])};
    }
    }
    */
}

void Translate(btTransform& trans,btVector3& toward){
    //trans.setOrigin()
    //btRigidBody sof;
    //sof.getCenterOfMassTransform()
    //sof.translate
    //�V�F�[�_�[���畨���G���W�����X�V���遄�V�F�[�_�[�Ɉړ����W�b�N�Ȃǋ@�\���W�܂肷����
    //�����G���W������Q�[���ɕύX��ʒm���ăV�F�[�_�[�ɔ��f�@��
    //�V�F�[�_�[���œ����蔻�聄�����蔻��̈ړ����ʓ|�A�A�b�v�f�[�g���A���G�ȃf�[�^���p�[�X
}

btCollisionObject& SoftBodyPhysic::MakeSensor(btCollisionObject& ob){
    ob.setCollisionFlags(ob.getCollisionFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
    return ob;
}

/*
  return [f](Minipoint){f(poinr)}

*/
template<class T1, class T2>
class ContactFun : public std::function<bool()>
{
public:

	// �ׂ��悷��֐��I�u�W�F�N�g
	T1 operator()(T1 base, T2 exp) const { return std::pow( base, exp ); }
};
bool dummyCallback(Manipoint& cp,
    const btCollisionObjectWrapper* colObj0,
    int partId0,
    int index0,
    const btCollisionObjectWrapper* colObj1,
    int partId1,
    int index1){

       return true;
}
void SoftBodyPhysic::ColisionCallBack(SoftBodyPhysic::CollideCallBack* f){
  //�I�u�W�F�N�g�ɃR�[���o�b�N�t�������(
  //  dynamicsWorld->contactTest(
    gContactAddedCallback=dummyCallback;
}
btSoftBody* SoftBodyPhysic::getOneSoft(){
    auto ar=dynamicsWorld->getSoftBodyArray();
    int size=ar.size();
    if(size==0)return nullptr;
    return ar.at(0);

}

SoftBodyPhysic::~SoftBodyPhysic(void)
{
    int i;
    for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    std::for_each(collisionShapes->begin(),collisionShapes->end(),[](btCollisionShape* shape) 
    {
    	delete shape;
    });
    collisionShapes->clear();
    delete collisionShapes;
    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete phase;
    delete collisionConfiguration;
}
