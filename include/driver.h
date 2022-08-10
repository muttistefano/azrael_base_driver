#ifndef DR_HH
#define DR_HH

#include <iostream>
#include <csignal>
#include <signal.h>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <softPwm.h>
#include <thread>
#include <phidget22.h>
#include <mutex>
#include <malloc.h>
#include <limits.h>
#include <sys/mman.h>  
#include <cmath>
#include <pid.h>

#include <iostream>
#include <fstream>

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include "Iir.h"
#include "random"
#include "time.h"

std::ofstream myfile;
  
#define DEBUG_PID

std::mutex mtx_enc1, mtx_enc2, mtx_enc3, mtx_enc4 ; 
bool stopping_ = false;

constexpr double radius    = 0.1016;
constexpr double lxy       = 0.71;

double vx,vy,vw;

class azrael_mobile_driver
{

    // safety data
    int safe_ = 1;

    double cmdvel_x = 0.0;
    double cmdvel_y = 0.0;
    double cmdvel_z = 0.0;
    

    double vel_enc1, vel_enc2, vel_enc3, vel_enc4  = 0.0;
    double vel_enc1_f, vel_enc2_f, vel_enc3_f, vel_enc4_f  = 0.0;

    Iir::Butterworth::LowPass<2> f_vel_1;
    Iir::Butterworth::LowPass<2> f_vel_2;
    Iir::Butterworth::LowPass<2> f_vel_3;
    Iir::Butterworth::LowPass<2> f_vel_4;

    PID pid_w1 = PID(10,30.0,0.,0.001,&vel_enc1,&mtx_enc1);
    PID pid_w2 = PID(10,30.0,0.,0.001,&vel_enc2,&mtx_enc2);
    PID pid_w3 = PID(10,30.0,0.,0.001,&vel_enc3,&mtx_enc3);
    PID pid_w4 = PID(10,30.0,0.,0.001,&vel_enc4,&mtx_enc4);

    double velx_odom = 0.0;
    double vely_odom = 0.0;
    double velw_odom = 0.0;
    double posx_odom = 0.0;
    double posy_odom = 0.0;
    double posw_odom = 0.0;


    int state1 = HIGH;
    int state2 = HIGH;
    int state3 = HIGH;
    int state4 = HIGH;
    
    PhidgetEncoderHandle encoder0;
    PhidgetEncoderHandle encoder1;
    PhidgetEncoderHandle encoder2;
    PhidgetEncoderHandle encoder3;

    //Socket
    int sockfd; 
    double buffer_in[3] = {0.0,0.0,0.0}; 
    double buffer_out[4] = {0.0,0.0,0.0,0.0}; 
    struct sockaddr_in     servaddr; 
    


public:
  
    // void pins_setup();
    azrael_mobile_driver();
    ~azrael_mobile_driver();

    void setWheelsSpeed();
    void control_thread();
    void odometry();
    void socket_feed();

    //   void safety_task();


};

azrael_mobile_driver* mobile_robot_driver;


#endif
