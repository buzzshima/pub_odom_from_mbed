
#include "mbed.h"
#include "QEI.h"


QEI r_wheel(p30, p29, NC , 100 ,QEI::X4_ENCODING);
QEI l_wheel(p28, p27, NC , 100 ,QEI::X4_ENCODING);
Ticker flipper;
Serial pc(USBTX, USBRX); // tx, rx

/*constant zone*/

#define ROTATE_PER_REVOLUTIONS 1000 //分解能
#define WHEEL_RADIUS 0.1//100//mm
#define REDUCTION_RATIO 1//減速比
#define MPI 3.14
#define TREAD 0.507//507//557 //mm　
#define DT 0.05 //要調整
/*constant zone */

/*Global variable*/
int now_pulse_l = 0, now_pulse_r = 0;
int old_pulse_l = 0, old_pulse_r = 0;
float omega_l, omega_r,omega_now,omega_old;
float vel_l, vel_r, vel_now, vel_old, vx, vy;
float x, y,  theta_now, theta_old, degree;
/*Global variable*/
/*Prototype declaration*/
void initodometory();
void odom() ;
 /*Prototype declaration*/

int main(){

    initodometory ();//初期化時は割り込み停止？
    flipper.attach(&odom, DT); //フリップ
    pc.baud(9600); 
    wait(2);
    while(1) {
        pc.printf("%f\t%f\t%f\t%f\t%f\t%f\n",x,y,theta_now,vx,vy,omega_now);
        wait(0.1);
    }
}

void initodometory(){
    l_wheel.reset();
    r_wheel.reset();

    now_pulse_l =0;now_pulse_r=0;
    old_pulse_l=0;old_pulse_r=0;
    
    omega_l=0;
    omega_r=0;
    omega_now=0;
    omega_old=0;
    
    vel_l=0; vel_r=0;
    vel_now=0; vel_old=0; vx = 0 ; vy = 0;
    x=0;  y=0;  degree=0; theta_now=0; theta_old=0;
}

void odom(){
    
    now_pulse_l = l_wheel.getPulses(); now_pulse_r = -1* r_wheel.getPulses();

    omega_l = (2.0 * MPI * (float)(now_pulse_l - old_pulse_l) ) / (4.0 * ROTATE_PER_REVOLUTIONS * REDUCTION_RATIO * DT);//4は1パルスで4カウントだだから相殺(X4_ENCODING)　<-4逓倍になってる
    omega_r = (2.0 * MPI * (float)(now_pulse_r - old_pulse_r) ) / (4.0 * ROTATE_PER_REVOLUTIONS * REDUCTION_RATIO * DT);

    vel_l = omega_l * WHEEL_RADIUS; 
    vel_r = omega_r * WHEEL_RADIUS;
    old_pulse_l = now_pulse_l;
    old_pulse_r = now_pulse_r;
    
    vel_now = (vel_l + vel_r) / 2;
    vx = vel_now * cos(theta_now);
    vy = vel_now * sin(theta_now);
    omega_now = (vel_r - vel_l) / TREAD;
    
    theta_now = ((omega_now + omega_old) * DT) / 2.0 + theta_now; 
    //x +=vel_now * cos(theta_now)*DT;
    //y += vel_now * sin(theta_now) * DT;
    x = ((((vel_now * cos(theta_now)) + (vel_old * cos(theta_old))) * DT) / 2.0 + x);
    y = ((((vel_now * sin(theta_now)) + (vel_old * sin(theta_old))) * DT) / 2.0 + y);
    degree = theta_now * (360.0 / (2.0 * MPI));

    // printf("x%f,y%f",x,y);//gtkかteratermで表示して保存　main文カナ
    
    omega_old = omega_now;
    vel_old = vel_now;
    theta_old = theta_now;


}