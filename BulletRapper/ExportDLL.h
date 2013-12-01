#pragma once
extern "C"{
    void MakeSpere(float,float,float,bool);
struct Vector3{
  float x,y,z;
};

void Init(float);
void Render();
void Step();
void MakeSpere(float x,float y,float z,bool);
void MakeFloor(Vector3,float,float*);
//void MakeSpereFromV3(Vector3);
Vector3 Orientation();
Vector3* Vertice();
void Quit();
void MakeBox(Vector3,float,bool,float*);
int ObjectCount();
/*
  makeCurveScript()

  unityで攻撃モーションのスクリプト
*/
}
#include <list>
