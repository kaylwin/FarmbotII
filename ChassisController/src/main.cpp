#include <Arduino.h>
#include <HardwareSerial.h>
#include <string>
#include <Encoder.h>

// Front motor controller input
#define FL_FPWM 21
#define FL_RPWM 20
#define FR_FPWM 4
#define FR_RPWM 3

// Rear Motor controller input
#define RL_FPWM 23
#define RL_RPWM 22
#define RR_FPWM 6
#define RR_RPWM 5

// 4 pins left forwards or
#define R_FW 8
#define R_REV 7
#define L_FW 1
#define L_REV 0

// Front encoders
#define FL_ENC_A 15
#define FL_ENC_B 11
#define FR_ENC_A 14
#define FR_ENC_B 10

// Rear encoders
#define RL_ENC_A 16
#define RL_ENC_B 12
#define RR_ENC_A 13
#define RR_ENC_B 9

bool joy_alive = false;
uint32_t cycles_since_update = 0;
bool all_halted = false;



class WheelController{
private:
    Encoder enc;
    int fpwm_;
    int rpwm_;
    bool first_run = true;
    uint32_t last_micros;
    const double ticks_rev =2994.6;


public:
    double setpoint = 0;
    double p = 5;
    double i = 0;
    double d = 0;
    double MAX_SUM = 50;

    double sum = 0;
    double last_e = 0;

    double max_jump_cycle = 25;

    // Necessary to prevent rapid oscillations in speed in 20ms
    double last_command = 0;
    boolean ishalted = true;
    double rpm = 0;
    boolean isForward = true;

    WheelController(int fpwm, int rpwm, int enc_a, int enc_b): enc(enc_a, enc_b){
        fpwm_ = fpwm;
        rpwm_ = rpwm;
        pinMode(fpwm_, OUTPUT);
        pinMode(rpwm_, OUTPUT);

        // Initialize to zero
        analogWrite(fpwm_, 0);
        analogWrite(rpwm_, 0);
    }

    void setSpeed(double speed){
        setpoint = speed;
        ishalted = false;
    }

    void run(){

        // Don't run if halted
        if (ishalted){
            analogWrite(fpwm_, 0);
            analogWrite(rpwm_, 0);
            sum = 0;
            return;
        }

        if (first_run){
            last_micros = micros();
            first_run = false;
            return;
        }


        // Convert time to seconds
        double dt = (micros() - last_micros) / 1e6;
        last_micros = micros();
        double ticks = enc.readAndReset();
        double revs = ticks / ticks_rev;

        // Calculate velocity in rpm
        rpm += 1.0/4 * ((revs / dt) * 60 - rpm);  // lpf on encoder msmnt

        double e =  setpoint - rpm;
        double diff = (e - last_e);

        double command = p * e + i * sum + d * diff;

        sum += e;

        // Saturation limits on integral term
        if (sum > MAX_SUM){
            sum = MAX_SUM;
        }
        else if (sum < - MAX_SUM){
            sum = -MAX_SUM;
        }
        last_e = e;

        // Put a low pass filter
        command = last_command + 1.0/8.0 * (command- last_command);

        // Prevent the command value from winding up above 255
        if (command < 0 && command <= -255){
            command = -255;
        }
        else if (command > 0 && command >= 255){
            command = 255;
        }

        last_command = command;

        if (command > 0){
            analogWrite(rpwm_, 0);
            command = (command > 255) ? 255 : command;  // Additional safety check
            analogWrite(fpwm_, static_cast<int>(command));
        }
        else{
            analogWrite(fpwm_, 0);
            command = (command < -255) ? -255 : command;  // Additional safety check
            analogWrite(rpwm_, static_cast<int>(-1*command));
        }
    }


    /**
     * Terminate running to handle controller error
     */
    void halt(){
        last_command = 0;
        setpoint = 0;
        ishalted = true;
        sum = 0;
        analogWrite(rpwm_, 0);
        analogWrite(fpwm_, 0);
    }

};

// Initialize wheel controllers
WheelController fl(FL_RPWM, FL_FPWM, FL_ENC_B, FL_ENC_A);  // inverted F & Rev
WheelController fr(FR_FPWM, FR_RPWM, FR_ENC_A, FR_ENC_B);
WheelController rr(RR_FPWM, RR_RPWM, RR_ENC_A, RR_ENC_B);
WheelController rl(RL_RPWM, RL_FPWM, RL_ENC_B, RL_ENC_A);

void setupPins(){

}

void handleSetCommand(String command){
    int id;
    int value;
    int res = sscanf(command.c_str(),"%d %d", &id, &value);
    if (res != 2){
        Serial.println("Error: " + command);
    }

    all_halted = false;
    Serial.printf("ACK ID: %d VAL: %d\n", id, value);
    switch(id){
        case 1:
            rl.setSpeed(value);
            break;
        case 2:
            fl.setSpeed(value);
            break;
        case 3:
            fr.setSpeed(value);
            break;
        case 4:
            rr.setSpeed(value);
            break;
        default:
            Serial.println("ERROR: ID not recognized");
            break;
    }
}

void handleStopCommand(String command){
    // Stop all motors
    fl.setSpeed(0);
    fr.setSpeed(0);
    rr.setSpeed(0);
    rl.setSpeed(0);

}

void handleHalt(){
   fl.halt();
   fr.halt();
   rl.halt();
   rr.halt();
   all_halted = true;
   Serial.printf("HALTING\nHALTING\nHALTING\nHALTING\n");
}

void setTunableParameter(WheelController& wc, char param, double value){

    switch(param){
        case 'p':
        case 'P':
            wc.p = value;
            break;

        case 'i':
        case 'I':
            wc.i = value;
            break;

        case 'D':
        case 'd':
            wc.d = value;
            break;

        case 'M':
        case 'm':
            wc.MAX_SUM = value;
            break;
        default:
            Serial.println("Error: invalid command");
            break;
    }

}

void handleTuneCommand(String command){
    command.trim();
    int id;
    char tuning_value;
    double value;
    int res = sscanf(command.c_str(),"%d %c %lf", &id, &tuning_value, &value);

    if (res != 3){
        Serial.println("Error: " + command);
    }
    switch(id){
        case 1:
            setTunableParameter(rl, tuning_value, value);
            break;
        case 2:
            setTunableParameter(fl, tuning_value, value);
            break;
        case 3:
            setTunableParameter(fr, tuning_value, value);
            break;
        case 4:
            setTunableParameter(rr, tuning_value, value);
            break;
        default:
            Serial.println("ERROR: Invalid command");
            break;

    }

}

void handleJoy(String command){
    joy_alive = true;
    cycles_since_update = 0;

    double velocity, heading;
    auto res = sscanf(command.c_str(), "V:%lf H:%lf", &velocity, &heading);
    if (res != 2){
        Serial.printf("ERROR: invalid command sent\n");
        return;
    }

    if (all_halted){
        Serial.printf("All halted\n");
        return;
    }

    // Break apart left side over right side
    double vl = velocity * 80;
    double vr = velocity * 80; // 80 is max RPM for our motors
    if (heading > 0){
        vr *= (1.0 - heading);
    }
    else if (heading < 0){
        vl *= (1.0 + heading);

    }
    else{
        // Do nothing to assigned values
    }

    fl.setSpeed(vl);
    rl.setSpeed(vl);

    fr.setSpeed(vr);
    rr.setSpeed(vr);
    Serial.printf("Vl: %lf VR: %lf \n", vl, vr);
}

void handleReset(){
    // Clear the halt on the joy stick
    all_halted = false;
}

void processCommand(String& command){
    // Trim the command to remove any leading/trailing whitespace
    command.trim();

    // Check if the command starts with "SET" or "STOP"
    if (command.startsWith("SET ")) {
        handleSetCommand(command.substring(4));  // Pass the argument part to handleSetCommand
    }
    else if (command.startsWith("TUN")){
        handleTuneCommand(command.substring(3));
    }
    else if (command.startsWith("HALT") || command.startsWith("halt")){
        handleHalt();
    }
    else if (command.startsWith("RESET")){
        handleReset();
    }
    else if (command.startsWith("JOY ")){
        handleJoy(command.substring(4));
    }
    else if (command.startsWith("STOP")) {
        handleStopCommand(command.substring(4));  // Call the stop command handler
    } else {
        Serial.println("ERR: " + command);
    }

}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(R_FW, OUTPUT);
    pinMode(R_REV, OUTPUT);
    pinMode(L_FW, OUTPUT);
    pinMode(L_REV, OUTPUT);

    // Set to forward for initialization
    digitalWrite(R_FW, HIGH);
    digitalWrite(R_REV, HIGH);
    digitalWrite(L_FW, HIGH);
    digitalWrite(L_REV, HIGH);

    // Make sure pins are in the correct pin mode
    setupPins();
}

void loop() {
    double start_time = micros();

    static String receivedData = "";  // Buffer to hold incoming data
    while (Serial.available() > 0) {
        char incomingByte = Serial.read();  // Read each incoming byte
        if (incomingByte == '\n') {
            // If newline is received, command is complete
            processCommand(receivedData);
            receivedData = "";  // Clear the buffer after processing
        } else {
            // Append each byte into the buffer
            receivedData += incomingByte;
        }
    }

    static uint32_t index = 0;

    //Serial.printf("%7d | FL: %.3lf | FR %.3lf | RL: %.3lf | RR: %.3lf\n", ++index, fl.rpm, fr.rpm, rl.rpm, rr.rpm);
   //Serial.printf("RR: %7d RPM: %.3lf COMM: %.3lf P: %.3lf i: %.3lf d: %.3lf HTL: %d SET: %lf\n", ++index, rr.rpm, rr.last_command, rr.p, rr.i, rr.d, (int) rr.ishalted, rr.setpoint);


    // Update the wheel controllers
    fl.run();
    fr.run();
    rr.run();
    rl.run();

    cycles_since_update++;
    if (joy_alive && cycles_since_update > 20){
        fl.halt();
        fr.halt();
        rr.halt();
        rl.halt();
        all_halted = false;
    }


    // Output current status
    double desired_delay = 20e3 - (micros() - start_time);
    if (desired_delay <= 0){
        Serial.println("ERROR: Overrun");
    }
    else{
        delayMicroseconds(static_cast<uint32_t>(desired_delay));
    }
}

