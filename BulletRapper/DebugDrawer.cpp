#include "DebugDrawer.h"


DebugDrawer::DebugDrawer(void)
{
}


DebugDrawer::~DebugDrawer(void)
{
}
void DebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
    glColor3f(color.x(),color.y(),color.z()); 
       //std::cout << "fromX:"<<from.getX() << std::endl;
       //std::cout << "fromY:"<<from.getY() << std::endl;
       //std::cout << "fromZ:"<<from.getZ() << std::endl;
       //std::cout <<to.getX() << std::endl;
       //std::cout <<to.getY() << std::endl;
       //std::cout <<to.getZ() << std::endl;
    glBegin(GL_LINES);
    glVertex3f(from.x(),from.y(),from.z()); 
    glVertex3f(to.x(),to.y(),to.z()); 
    glEnd();
}
void DebugDrawer::drawContactPoint
    (const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color){
        glPushMatrix();
        glColor3f(color.x(),color.y(),color.z()); 
        glTranslatef(PointOnB.x(),PointOnB.y(),PointOnB.z()); 
        glutWireCube(0.1);
        glBegin(GL_LINES);
        
        glVertex3f(normalOnB.x(),normalOnB.y(),normalOnB.z()); 
	glEnd();
}
void DebugDrawer::reportErrorWarning(const char* warningString){
    std::cout << warningString << std::endl;

}
void DebugDrawer::draw3dText(const btVector3& location,const char* textString){
    std::cout << textString<< std::endl;
}
void DebugDrawer::setDebugMode(int debugMode){
    ;
}
int DebugDrawer::getDebugMode() const{
    return DBG_MAX_DEBUG_DRAW_MODE;
}

