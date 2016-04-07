//
// Created by xuhao on 2016/4/6.
//
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace Eigen;

int main(int argc,char ** argv)
{
    Vector3f a,b;
    a<< 1,2,3;
    b<<4,5,6;
    std::cout<< a / b;
}
