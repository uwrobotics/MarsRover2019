#include "Roboarm.h"
#include <iostream>
#include <cmath>
using namespace std;

Roboarm::Roboarm(float shoulder=0, float elbow=0, float wrist=0, float l_s, float l_e, float l_w) :
    shoulder_len(l_s), elbow_len(l_e), wrist_len(l_w){

    shoulder_pos =shoulder;
    elbow_pos = elbow;
    wrist_pos = wrist;

    shoulder_vel = 0;
    elbow_vel = 0;
    wrist_vel = 0;


}

void Roboarm::calc_velocities(float ee_x, float ee_y, float ee_phi){
    c1 = cos(shoulder_pos);
    s1 = sin(shoulder_pos);
    c12 = cos(shoulder_pos+elbow_pos);
    s12 = sin(shoulder_pos+elbow_pos);
    c123 = cos(shoulder_pos+elbow_pos+wrist_pos);
    s123 = sin(shoulder_pos+elbow_pos+wrist_pos);

    l1=shoulder_len;
    l2 =elbow_len;
    l3 = wrist_len();

    shoulder_vel = l2*c12*ee_x + l2*s12*ee_y + l2*l3*(c12*s123-s12*c123)*ee_phi;
    elbow_vel = (-l1*c1 -l2*c12)*ee_x - (l1*s1 + l2*s12)*ee_y - (l1*l3*(c1*s123-s1*c123)+l2*l3*(c12*s123-s12*c123))*ee_phi;
    wrist_vel = l1*c1*ee_x +l1*s1*ee_y + (l1*l2*(c1*s12-s1*c12)+l1*l3*(c1*s123-s1*c123))*ee_phi;



}
