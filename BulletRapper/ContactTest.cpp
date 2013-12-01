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
    
    glClear(GL_COLOR_BUFFER_BIT);   //  �J���[�o�b�t�@���N���A
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

    glClear(GL_COLOR_BUFFER_BIT);   //  �J���[�o�b�t�@���N���A
    glMatrixMode( GL_PROJECTION );  //  �ˉe�s��
    glLoadIdentity();               //  �ϊ��s���������
    gluPerspective(30.0,            //  ����p
        width/height,     //  �A�X�y�N�g��
        1.0, 100.0);    //  ����͈́i�ߋ����E�������j
    gluLookAt(  0, 0, 0,      //  ���_�ʒu
        2, 6, 2,      //  �ڕW�ʒu
        0.0, 1.0, 0.0);     //  �����

    glMatrixMode( GL_MODELVIEW );   //  ���f���r���[�s��
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
    //body�ւ̃|�C���^�������Ă���Ă���
}