#include "PhysicsEngine.h"
#include <algorithm>
#include <functional>
#include "TypeOfShape.h"
/*
シミュレーションと補間処理
デフォルトでは、Bulletの物理シミュレーションは内部フレームレート60Hz(0.01666)で固定されています。
ゲームやアプリケーションでは、異なるフレームレート、または変動フレームレートを使っている場合があります。
アプリケーションのフレームレートとシミュレーションのフレームレートが一致しない場合は、
自動補間メソッドがstepSimurationに組み込まれます。
アプリケーションの差分時間が内部の固定タイムスタンプよりも小さい場合、Bulletは物理シミュレーションを行わずに、
ワールド変換行列を補間し、それをbtMotionStateへ送ります。アプリケーションタイムスタンプのほうが大きい場合は、
各stepSimulationで物理シミュレーションステップを1回多く行います。このシミュレーションステップの最大値は、
2番目の引数にてユーザーが指定することができます。
 
剛体が生成されたとき、btMotionState::getWorldTransformメソッドを使ってMotionStateから初期ワールド変換行列を取得します。
stepSimurationによりシミュレーションが実行されると、btMotionState::setWorldTransformにより対象の剛体について、
新しいワールド変換行列が設定されます。
 
剛体は正の質量を持ち、その動きはシミュレーションにより決定されます。
静的剛体とKinematic剛体は質量ゼロです。静的オブジェクトはユーザーの手で動かすべきではありません


*/

PhysicsEngine::PhysicsEngine(const btVector3 &grav)
{
    //auto world=new btDynamicsWorld();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new	btCollisionDispatcher(collisionConfiguration);
    //マルチスレッドなら別のコンストラクタを使ってね
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
//todo setHeight(float height){btCollisionShapde::setScaling(vector3)} 物体の高さだけを設定したい
template <typename T>
void foreach(std::list<T*>* list,std::function<void(T*)> f){
    std::for_each(list->begin();,list->end(),f);
}

void PhysicsEngine::Step(){
    /*
デフォルトの内部固定タイムステップを使ってください
Bulletは内部固定タイムステップである 60Hz(1/60秒)に最適化されています。
シミュレーションを安全かつ安定させるために、Bulletは自動的にタイムステップを固定シミュレーションサブステップに分割します。
このサブステップ数の最大値は、stepSimulationの2番目の引数で指定します。
タイムステップが内部サブステップよりも小さい場合、Bulletは動作を補間します。
 
サブステップの最大数(stepSimulationの第2引数)をゼロとすることで、この機能を無効化することができます。
内部タイムステップとサブステップを無効にすると、実際のタイムステップでシミュレーションが行われます。これは推奨されません。
    */
    //std::for_each( collisionShapes->begin()
    dynamicsWorld->stepSimulation(1.f/60.f,10);
    //std::for_each(collisionShapes->begin(),collisionShapes->end(),[](btCollisionShape* shape){});
    
}
//position,scale
void PhysicsEngine::CreatePlane(const btVector3& pos,const float scale){
    //法線方向の設定
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
//todo btCollisionShapeが型見えてるので隠したい（しかも頂点情報とかないし）
//BoxShapeは奥行きなどのパラメータがある  type TypeOfShape=|BoxShape of width*height
template<typename T>
void forEachObject(btAlignedObjectArray<T> objects,std::function<void(T)> f){
    int size=objects.size();
    for(int i=0;i<size;i++){
      f(objects.at(i));
    }
}

//todo typeofShapeを実装する、頂点情報なども提供する
btRigidBody* PhysicsEngine::CreateBody(btScalar mass,TypeOfShape shape){

    auto trans=originTransform();

    btVector3 localInertia(0,1,0);
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btBoxShape *box=new btBoxShape(btVector3(10,10,10));
    //shape.CreateShape();
    if(mass!=0.f)box->calculateLocalInertia(mass,localInertia);
    collisionShapes->push_back(box);
    //最初の姿勢と重心を設定
    btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,box,localInertia);
    auto body= new btRigidBody(rbInfo);
    dynamicsWorld->addRigidBody(body);
    return body;
}
void myTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    //とりあえず動くか試す
    //ManifoldPointで返している、中身はむやみやたらとポインタを使っていない
    //接触の物理マテリアルを取得するのにpartIDを使っている
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
