#include<driver.h>
#include<pin_config.h>

void my_handler(int signum){
    stopping_ = true;
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    softPwmWrite(PWM_pin_1,0);
    softPwmWrite(PWM_pin_2,0);
    softPwmWrite(PWM_pin_3,0);
    softPwmWrite(PWM_pin_4,0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));	
    softPwmWrite(PWM_pin_1,0);
    softPwmWrite(PWM_pin_2,0);
    softPwmWrite(PWM_pin_3,0);
    softPwmWrite(PWM_pin_4,0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));	
    softPwmStop   (PWM_pin_1) ;
    softPwmStop   (PWM_pin_2) ;
    softPwmStop   (PWM_pin_3) ;
    softPwmStop   (PWM_pin_4) ;
    delete(mobile_robot_driver);
}

static void CCONV onEncoder0_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD1";
        return;
    } 
    mtx_enc1.lock();
	*((double *) ctx) = ((double)positionChange * -2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc1.unlock();
}

static void CCONV onEncoder1_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD2";
        return;
    } 
    mtx_enc2.lock();
	*((double *) ctx) = ((double)positionChange * 2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc2.unlock();
}

static void CCONV onEncoder2_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD2";
        return;
    } 
    mtx_enc3.lock();
	*((double *) ctx) = ((double)positionChange * 2 * M_PI) / (timeChange  * 0.001 * 520 * 4) ;
    mtx_enc3.unlock();
}

static void CCONV onEncoder3_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD2";
        return;
    } 
    mtx_enc4.lock();
	*((double *) ctx) = ((double)positionChange * -2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc4.unlock();
}

azrael_mobile_driver::~azrael_mobile_driver()
{
    std::cout << " azrael_mobile_driver Destructor\n";
    #ifdef DEBUG_PID
    myfile.close();
    #endif
    close(this->sockfd); 

    Phidget_close((PhidgetHandle)this->encoder0);
	PhidgetEncoder_delete(&this->encoder0);
    Phidget_close((PhidgetHandle)this->encoder1);
	PhidgetEncoder_delete(&this->encoder1);
    Phidget_close((PhidgetHandle)this->encoder2);
	PhidgetEncoder_delete(&this->encoder2);
    Phidget_close((PhidgetHandle)this->encoder3);
	PhidgetEncoder_delete(&this->encoder3);
}

azrael_mobile_driver::azrael_mobile_driver()
{
    std::cout << "Constructor \n";
    pinMode(PWM_pin_1,OUTPUT);
    pinMode(PWM_pin_2,OUTPUT);
    pinMode(PWM_pin_3,OUTPUT);
    pinMode(PWM_pin_4,OUTPUT);
    softPwmCreate (PWM_pin_1, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_2, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_3, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_4, 0, MAX_PWM_RANGE) ;
    pinMode (PWM_dir_1, OUTPUT) ;
    pinMode (PWM_dir_2, OUTPUT) ;
    pinMode (PWM_dir_3, OUTPUT) ;
    pinMode (PWM_dir_4, OUTPUT) ;

    PhidgetEncoder_create(&this->encoder0);
    PhidgetEncoder_create(&this->encoder1);
    PhidgetEncoder_create(&this->encoder2);
    PhidgetEncoder_create(&this->encoder3);

    Phidget_setChannel((PhidgetHandle)this->encoder0, 0);
    Phidget_setChannel((PhidgetHandle)this->encoder1, 1);
    Phidget_setChannel((PhidgetHandle)this->encoder2, 2);
    Phidget_setChannel((PhidgetHandle)this->encoder3, 3);

    PhidgetEncoder_setOnPositionChangeHandler(this->encoder0, onEncoder0_PositionChange, (void *)&this->vel_enc1);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder1, onEncoder1_PositionChange, (void *)&this->vel_enc2);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder2, onEncoder2_PositionChange, (void *)&this->vel_enc3);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder3, onEncoder3_PositionChange, (void *)&this->vel_enc4);


    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder0, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder1, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder2, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder3, 0);

    PhidgetEncoder_setPositionChangeTrigger(this->encoder0, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder1, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder2, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder3, 0);

    int interval = 32;

    PhidgetEncoder_setDataInterval(this->encoder0, interval);
    PhidgetEncoder_setDataInterval(this->encoder1, interval);
    PhidgetEncoder_setDataInterval(this->encoder2, interval);
    PhidgetEncoder_setDataInterval(this->encoder3, interval);

    //Socket

    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    memset(&servaddr, 0, sizeof(servaddr)); 
        
    // Filling server information 
    this->servaddr.sin_family = AF_INET; 
    this->servaddr.sin_port = htons(44100); 
    this->servaddr.sin_addr.s_addr = inet_addr("192.169.1.2");


    std::cout << "Constructor End\n";

}


void azrael_mobile_driver::control_thread()
{
    double time_sin = 0.0;
    while(!stopping_)
    {   

        #ifndef DEBUG_PID        
        double v1in = ( buffer_in[0] - buffer_in[1] - (lxy * buffer_in[2])) * (1.0/radius);
        double v2in = (-buffer_in[0] - buffer_in[1] + (lxy * buffer_in[2])) * (1.0/radius);
        double v3in = ( buffer_in[0] - buffer_in[1] + (lxy * buffer_in[2])) * (1.0/radius);
        double v4in = (-buffer_in[0] - buffer_in[1] - (lxy * buffer_in[2])) * (1.0/radius);
        #endif

        #ifdef DEBUG_PID
        double vy = 0.2 * sin(0.1 * M_PI * time_sin);
        time_sin = time_sin + 0.032;
        double vx = 0;//sin*(0.2 * M_PI + t);
        double vw = 0;//sin*(0.2 * M_PI + t);

        double v1in = ( vx - vy - (lxy * vw)) * (1.0/radius);
        double v2in = (-vx - vy + (lxy * vw)) * (1.0/radius);
        double v3in = ( vx - vy + (lxy * vw)) * (1.0/radius);
        double v4in = (-vx - vy - (lxy * vw)) * (1.0/radius);
        #endif

        this->pid_w1.setSetpoint(abs(v1in));
        this->pid_w2.setSetpoint(abs(v2in));
        this->pid_w3.setSetpoint(abs(v3in));
        this->pid_w4.setSetpoint(abs(v4in));

        int v1dir = (v1in < 0) ? LOW  : HIGH;
        int v2dir = (v2in < 0) ? HIGH : LOW;
        int v3dir = (v3in < 0) ? HIGH : LOW;
        int v4dir = (v4in < 0) ? LOW  : HIGH;
        
        //TODO tristate assign
        digitalWrite(PWM_dir_1,v1dir);
        digitalWrite(PWM_dir_2,v2dir);
        digitalWrite(PWM_dir_3,v3dir);
        digitalWrite(PWM_dir_4,v4dir);

        int pwm1 = (int)pid_w1.update_state();
        int pwm2 = (int)pid_w2.update_state();
        int pwm3 = (int)pid_w3.update_state();
        int pwm4 = (int)pid_w4.update_state();

        #ifdef DEBUG_PID
        myfile << pwm1 << "," << pwm2 << "," << pwm3 << "," << pwm4 << "," 
               << this->vel_enc1 << "," << this->vel_enc2 << "," << this->vel_enc3 << "," << this->vel_enc4 << "," 
               << v1in << "," << v2in << "," << v3in << "," << v4in << "\n";
        #endif

        softPwmWrite (PWM_pin_1,  pwm1) ;	
        softPwmWrite (PWM_pin_2,  pwm2) ;
        softPwmWrite (PWM_pin_3,  pwm3) ;
        softPwmWrite (PWM_pin_4,  pwm4) ;
        std::this_thread::sleep_for(std::chrono::milliseconds(32));
    }
}

void azrael_mobile_driver::odometry()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto last_time    = std::chrono::high_resolution_clock::now();

    while(!stopping_)
    {
        last_time = current_time;
        current_time = std::chrono::high_resolution_clock::now();

        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(last_time-current_time).count() / 1e-9;
        mtx_enc1.lock();
        mtx_enc2.lock();
        mtx_enc3.lock();
        mtx_enc4.lock();
        this->velx_odom = (      this->vel_enc1 + this->vel_enc2 + this->vel_enc3 + this->vel_enc4 ) * (radius * 0.5);
        this->vely_odom = ( -1 * this->vel_enc1 + this->vel_enc2 + this->vel_enc3 - this->vel_enc4 ) * (radius * 0.5);
        this->velw_odom = ( -1 * this->vel_enc1 + this->vel_enc2 - this->vel_enc3 + this->vel_enc4 ) * (radius / ( 4 * lxy));
        mtx_enc1.unlock();
        mtx_enc2.unlock();
        mtx_enc3.unlock();
        mtx_enc4.unlock();
        this->posx_odom += (this->velx_odom * cos(this->posw_odom) - this->vely_odom * sin(this->posw_odom)) * dt;
        this->posy_odom += (this->velx_odom * sin(this->posw_odom) + this->vely_odom * cos(this->posw_odom)) * dt;
        this->posw_odom += this->velw_odom * dt;


        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // std::cout << "enc :" << this->vel_enc1 << " " << this->vel_enc2 << " " << this->vel_enc3 << " " << this->vel_enc4 << "\n";

    }
}

void azrael_mobile_driver::socket_feed()
{
    int n; 
    unsigned int len;
    while(!stopping_)
    {
        // std::cout << "SENDING \n";
        sendto(this->sockfd, (const void *)buffer_out, sizeof(double) * 4, MSG_DONTWAIT, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
        // std::cout << "RECEIVING \n";
        n = recvfrom(this->sockfd, (void *)buffer_in, sizeof(double) * 3, MSG_DONTWAIT, (struct sockaddr *) &servaddr, &len); 
        // std::cout << "rec: " << buffer_in[0] << " " << buffer_in[1] << " " << buffer_in[2] << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char **argv)
{      

    #ifdef DEBUG_PID
    std::cout << "DEBUG PID MODE ----  DEBUG PID MODE \n DEBUG PID MODE ----  DEBUG PID MODE \n";
    myfile.open ("log.txt");
    #endif

    // vx = atof(argv[1]);
    // vy = atof(argv[2]);
    // vw = atof(argv[3]);

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
            exit(-2);
    }


    std::cout << "Wiring pi setup .\n";
    if (wiringPiSetup () == -1) exit (1) ;

    struct sigaction sigIntHandler;

    printf("Sig setting\n");
    signal(SIGINT, my_handler);

    std::cout << "Class init \n";
    mobile_robot_driver = new azrael_mobile_driver();

    std::cout << "Thread started .\n";
    std::thread control_t(&azrael_mobile_driver::control_thread, mobile_robot_driver);
    // std::thread    odom_t(&azrael_mobile_driver::odometry, mobile_robot_driver);
    std::thread    sock_t(&azrael_mobile_driver::socket_feed, mobile_robot_driver);

    sched_param sch;
    int policy;
    pthread_getschedparam(control_t.native_handle(), &policy, &sch);
    sch.sched_priority = 80;

    pthread_setschedparam(control_t.native_handle(), SCHED_RR, &sch);
    // pthread_setschedparam(odom_t.native_handle(), SCHED_RR, &sch);
    pthread_setschedparam(sock_t.native_handle(), SCHED_RR, &sch);
    


    std::cout << "Thread joined.\n";

    control_t.join();
    // odom_t.join();
    sock_t.join();

    return 0;
}
