#include "PhysicsEngine.h"
#include <algorithm>
#include <functional>
#include "TypeOfShape.h"
/*
�V�~�����[�V�����ƕ�ԏ���
�f�t�H���g�ł́ABullet�̕����V�~�����[�V�����͓����t���[�����[�g60Hz(0.01666)�ŌŒ肳��Ă��܂��B
�Q�[����A�v���P�[�V�����ł́A�قȂ�t���[�����[�g�A�܂��͕ϓ��t���[�����[�g���g���Ă���ꍇ������܂��B
�A�v���P�[�V�����̃t���[�����[�g�ƃV�~�����[�V�����̃t���[�����[�g����v���Ȃ��ꍇ�́A
������ԃ��\�b�h��stepSimuration�ɑg�ݍ��܂�܂��B
�A�v���P�[�V�����̍������Ԃ������̌Œ�^�C���X�^���v�����������ꍇ�ABullet�͕����V�~�����[�V�������s�킸�ɁA
���[���h�ϊ��s����Ԃ��A�����btMotionState�֑���܂��B�A�v���P�[�V�����^�C���X�^���v�̂ق����傫���ꍇ�́A
�estepSimulation�ŕ����V�~�����[�V�����X�e�b�v��1�񑽂��s���܂��B���̃V�~�����[�V�����X�e�b�v�̍ő�l�́A
2�Ԗڂ̈����ɂă��[�U�[���w�肷�邱�Ƃ��ł��܂��B
 
���̂��������ꂽ�Ƃ��AbtMotionState::getWorldTransform���\�b�h���g����MotionState���珉�����[���h�ϊ��s����擾���܂��B
stepSimuration�ɂ��V�~�����[�V���������s�����ƁAbtMotionState::setWorldTransform�ɂ��Ώۂ̍��̂ɂ��āA
�V�������[���h�ϊ��s�񂪐ݒ肳��܂��B
 
���̂͐��̎��ʂ������A���̓����̓V�~�����[�V�����ɂ�茈�肳��܂��B
�ÓI���̂�Kinematic���͎̂��ʃ[���ł��B�ÓI�I�u�W�F�N�g�̓��[�U�[�̎�œ������ׂ��ł͂���܂���


*/

PhysicsEngine::PhysicsEngine(const btVector3 &grav)
{
    //auto world=new btDynamicsWorld();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new	btCollisionDispatcher(collisionConfiguration);
    //�}���`�X���b�h�Ȃ�ʂ̃R���X�g���N�^���g���Ă�
    //overlappingPairCache  
    broadphase=new btDbvtBroadphase();
    /*
    btVector3 worldBoundsMin(-1000,-1000,-1000);
    btVector3 worldBoundsMax(1000,1000,1000);
    broadphase=new btAxisSweep3(worldBoundsMin,worldBoundsMax,100);
    */
    solver = new btSequentialImpulseConstraintSolver;
    collisionShapes=new ShapeList;
    //bodies=btAlignedObjectArray<btRigidBody*>();
    dynamicsWorld = 
        new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(grav);
    dynamicsWorld->setInternalTickCallback(myTickCallback);
}
//http://bulletjpn.web.fc2.com/05_BulletCollisionDetection.html
//todo setHeight(float height){btCollisionShapde::setScaling(vector3)} ���̂̍���������ݒ肵����
template <typename T>
void foreach(std::list<T*>* list,std::function<void(T*)> f){
    std::for_each(list->begin();,list->end(),f);
}

void PhysicsEngine::Step(){
    /*
�f�t�H���g�̓����Œ�^�C���X�e�b�v���g���Ă�������
Bullet�͓����Œ�^�C���X�e�b�v�ł��� 60Hz(1/60�b)�ɍœK������Ă��܂��B
�V�~�����[�V���������S�����肳���邽�߂ɁABullet�͎����I�Ƀ^�C���X�e�b�v���Œ�V�~�����[�V�����T�u�X�e�b�v�ɕ������܂��B
���̃T�u�X�e�b�v���̍ő�l�́AstepSimulation��2�Ԗڂ̈����Ŏw�肵�܂��B
�^�C���X�e�b�v�������T�u�X�e�b�v�����������ꍇ�ABullet�͓�����Ԃ��܂��B
 
�T�u�X�e�b�v�̍ő吔(stepSimulation�̑�2����)���[���Ƃ��邱�ƂŁA���̋@�\�𖳌������邱�Ƃ��ł��܂��B
�����^�C���X�e�b�v�ƃT�u�X�e�b�v�𖳌��ɂ���ƁA���ۂ̃^�C���X�e�b�v�ŃV�~�����[�V�������s���܂��B����͐�������܂���B
    */
    //std::for_each( collisionShapes->begin()
    dynamicsWorld->stepSimulation(1.f/60.f,10);
    //std::for_each(collisionShapes->begin(),collisionShapes->end(),[](btCollisionShape* shape){});
    
}
//position,scale
void PhysicsEngine::CreatePlane(const btVector3& pos,const float scale){
    //�@�������̐ݒ�
    btStaticPlaneShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),scale);
    
        btVector3 localInertia(1,1,1);
        btScalar mass(1);
        collisionShapes->push_back(static_cast<btCollisionShape*>(groundShape));
    
    btTransform transform;
    {   transform.setIdentity();
        transform.setOrigin(pos);
    }
    btDefaultMotionState* myMotionState = new btDefaultMotionState();
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    auto body= new btRigidBody(rbInfo);
    //bodies.push_back(body);
    dynamicsWorld->addCollisionObject(body);
}

void PhysicsEngine::SetSensor(const btVector3& pos,const btVector3& size){
    auto box=new btBoxShape(size);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(pos);
    btDefaultMotionState* state=new btDefaultMotionState(trans);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0,state,box);
    auto body=new btRigidBody(rbInfo);
    body->setCollisionFlags(body->getCollisionFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
    dynamicsWorld->addRigidBody(body);
}

btTransform originTransform(){
      btTransform trans;
      trans.setIdentity();
      trans.setOrigin(btVector3(0,-50,0));
      return trans;
}
//todo btCollisionShape���^�����Ă�̂ŉB�������i���������_���Ƃ��Ȃ����j
//BoxShape�͉��s���Ȃǂ̃p�����[�^������  type TypeOfShape=|BoxShape of width*height
template<typename T>
void forEachObject(btAlignedObjectArray<T> objects,std::function<void(T)> f){
    int size=objects.size();
    for(int i=0;i<size;i++){
      f(objects.at(i));
    }
}

//todo typeofShape����������A���_���Ȃǂ��񋟂���
btRigidBody* PhysicsEngine::CreateBody(btScalar mass,TypeOfShape shape){

    auto trans=originTransform();

    btVector3 localInertia(0,1,0);
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btBoxShape *box=new btBoxShape(btVector3(10,10,10));
    //shape.CreateShape();
    if(mass!=0.f)box->calculateLocalInertia(mass,localInertia);
    collisionShapes->push_back(box);
    //�ŏ��̎p���Əd�S��ݒ�
    btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,box,localInertia);
    auto body= new btRigidBody(rbInfo);
    dynamicsWorld->addRigidBody(body);
    return body;
}
void myTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    //�Ƃ肠��������������
    //ManifoldPoint�ŕԂ��Ă���A���g�͂ނ�݂₽��ƃ|�C���^���g���Ă��Ȃ�
    //�ڐG�̕����}�e���A�����擾����̂�partID���g���Ă���
    auto objects = world->getCollisionObjectArray();
    forEachObject<btCollisionObject*>
        (objects,[](btCollisionObject*& i) {
            //pos=i->getWorldTransform().getOrigin();
	    auto pos=i;
    });
    return ;
};

extern ContactAddedCallback	gContactAddedCallback;

PhysicsEngine::~PhysicsEngine(void)
{
    gContactAddedCallback=nullptr;
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
    delete collisionConfiguration;
}
