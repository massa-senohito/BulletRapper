#include "ContactTest.h"
#include "TypeOfShape.h"
#include <memory>
#include <iostream>
#include "SoftBodyPhysic.h"
#include "freeglut.h"
#include "SoftBodyPhysic.h"
namespace test{
int margc;char** margv;

SoftBodyPhysic *phy;

}
int main(int argc,char* argv[]){
    test::margc=argc;test::margv=argv;
    auto game=new ContactTest();
    //std::unique_ptr<
    //delete game;
}

ContactTest::ContactTest(void)
{
    //using IntLis=std::list<int*>;
    //typedef std::list<int*> IntLis
    //
    //auto lis=new IntLis();
    SoftTest();
}/**/
void ContactTest::RigidTest(){
    eng=new PhysicsEngine(btVector3(0,-10,0));
    eng->CreatePlane(btVector3(0,0,0),10);
    
    body=eng->CreateBody(1,TypeOfShape());
    auto i=0;
    while(i<30){
      eng->Step();
      auto trans=body->getWorldTransform();
      std::cout << trans.getOrigin().getX() << " "<< trans.getOrigin().getY() << " "<<trans.getOrigin().getZ()<<std::endl;
      i++;
    }
}
void ContactTest::SoftTest(){
 
    test::phy=new SoftBodyPhysic(btVector3(0,0,0));
    test::phy->MakeShape(btVector3(3,4,5),false);
    //mat=test::phy->MakeFloor(btVector3(0,0,0),10);
    glDraw();
}
void SoftDraw(){
    
    glClear(GL_COLOR_BUFFER_BIT);   //  カラーバッファをクリア
    test::phy->RenderVertice();
    glutSwapBuffers();

}

void Onkey(unsigned char k,int,int){
    if(k=' ') glutExit(); delete test::phy;
}
void idle(){
    glutPostRedisplay();
}

void glDraw(){
    double width=500;double height=500;
    glutInitWindowSize(500, 500); glutInitWindowPosition ( 140, 140 );
    glutInit(&test::margc,test::margv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |GLUT_MULTISAMPLE);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_CONTINUE_EXECUTION);

    glClear(GL_COLOR_BUFFER_BIT);   //  カラーバッファをクリア
    glMatrixMode( GL_PROJECTION );  //  射影行列
    glLoadIdentity();               //  変換行列を初期化
    gluPerspective(30.0,            //  視野角
        width/height,     //  アスペクト比
        1.0, 100.0);    //  視野範囲（近距離・遠距離）
    gluLookAt(  0, 0, 0,      //  視点位置
        2, 6, 2,      //  目標位置
        0.0, 1.0, 0.0);     //  上方向

    glMatrixMode( GL_MODELVIEW );   //  モデルビュー行列
    glLoadIdentity();
    glutCreateWindow("softbody");
    glutDisplayFunc(SoftDraw);
    glutKeyboardFunc( Onkey);
//  glutReshapeFunc(resize);
  glutIdleFunc(idle);
//  glutInitWarningFunc(Warning);
//  glutInitErrorFunc(Error);
    //glutExit
    glutMainLoop();
}

ContactTest::~ContactTest(void)
{
    delete eng;
    //bodyへのポインタも消してくれている
}