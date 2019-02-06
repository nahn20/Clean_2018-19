package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="armPID", group="Pushboat")
public class armPID extends LinearOpMode {
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    private DcMotor hook_motor   = null;
    private AnalogInput potentiometer = null;
    BNO055IMU imu = null;
    private ElapsedTime runtime = new ElapsedTime();
    double lastTime = 0;
    double integral = 0;
    double error = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    @Override
    public void runOpMode() {
        hook_motor   = hardwareMap.dcMotor.get("hook_motor");

        hook_motor.setDirection(DcMotor.Direction.FORWARD);

        hook_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        potentiometer = hardwareMap.analogInput.get("potentiometer");

        /* IMU STUFF */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        while(!opModeIsActive()){
        }
        lastTime = runtime.seconds();
        while(opModeIsActive()){
            tune();
            basicControl();
            updateKeys();
            telemetry.update();
        }
    }
    
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void basicControl(){
        telemetry.addData("Potentiometer position", potentiometer.getVoltage());
        double deltaT = runtime.seconds() - lastTime;
        double desiredPosition = .682;
        if(gamepad1.a == true) {
        desiredPosition = 1;
    }
        double errorPrior = error;
        error = -(desiredPosition - potentiometer.getVoltage());
        integral += error*deltaT;
        double derivative = (error-errorPrior)/deltaT;
        hook_motor.setPower(kp*error + ki*integral + kd*derivative);
        telemetry.addData("Error", error);
        telemetry.addData("Integral", integral);
        telemetry.addData("Derivative", derivative);
        telemetry.addData("DeltaT", deltaT);
        telemetry.addData("Time", runtime.milliseconds());
        lastTime = runtime.seconds();
    }
    public void updateKeys(){
        if(gamepad1.a && cdCheck(useMap1.a, 1000)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
        }
        if(gamepad1.right_stick_x > 0 && cdCheck(useMap1.right_stick_x_pos, 700)){
            toggleMap1.right_stick_x_pos = toggle(toggleMap1.right_stick_x_pos);
            useMap1.right_stick_x_pos = runtime.milliseconds();
        }
    }
    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
    public void tune() {
        double tuneamnt =.01;
        telemetry.addData("kp", kp);
        telemetry.addData("ki", ki);
        telemetry.addData("kd", kd);
        // if (gamepad.y) { BROKEN
        //     desiredPosition = potentiometer.getVoltage();
        // }
        if(gamepad1.dpad_up) {
        kp += tuneamnt;
            }
       if(gamepad1.dpad_down) {
        kp -= tuneamnt;
            }
        if(gamepad1.dpad_right) {
        ki += tuneamnt;
            }   
        if(gamepad1.dpad_left) {
        ki -= tuneamnt;
            }
        if(gamepad1.b == true) {
        kd += tuneamnt/100;
            }
        if(gamepad1.x == true) {
        kd -= tuneamnt/100;
            }
        if(kp<0){
            kp=0;
            }
        if(ki<0){
            ki=0;
            }
        if(kd<0){
            kd=0;
            }
        
    }
}