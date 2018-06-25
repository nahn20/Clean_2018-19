package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servoTester", group="Test")
public class servoTester extends LinearOpMode {
    private Servo servo = null;
    @Override
    public void runOpMode() {
        servo = hardwareMap.servo.get("armServo"); //Change "armServo" to servo's name in hardware map

        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
            servoAdjust();
            servoToLimits();
            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
        }
    }
    public void servoAdjust(){
        int denominator = 1000;
        if(gamepad1.right_bumper){
            denominator = 100;
        }
        servo.setPosition(servo.getPosition() - gamepad1.right_stick_x/denominator);
    }
    public void servoToLimits(){
        if(gamepad1.b){
            servo.setPosition(1);
        }
        else if(gamepad1.a){
            servo.setPosition(0);
        }
    }
}