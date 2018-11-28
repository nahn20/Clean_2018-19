package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="testeOp", group="Test")
public class testeOp extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();

    private ElapsedTime runtime = new ElapsedTime();
    
    /////////////////
    /*IMU Variables*/
    //[[[[[[[[[[[[[[[
    double newZero = 0;
    int fullRotationCount = 0;
    double previousAngle = 0;
    //]]]]]]]]]]]]]]]

    int savedEncoderPositions[] = {0, 0, 0, 0}; //fl, fr, bl, br

    @Override
    public void runOpMode() {

        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        shell.front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shell.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeIsActive()){
            if(gamepad1.a){
                savedEncoderPositions[0] = shell.front_left_motor.getCurrentPosition();
                savedEncoderPositions[1] = shell.front_right_motor.getCurrentPosition();
            }
            if(toggleMap1.left_bumper){
                runToPositions(savedEncoderPositions[0], savedEncoderPositions[1], savedEncoderPositions[2], savedEncoderPositions[3], 0.3);
            }
            else if(toggleMap1.right_bumper){
                runToPositions(0, 0, 0, 0, 0.3);
            }
            else{
                shell.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shell.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive();
            }

            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    ///////////
    // TESTS //
    ///////////
    public void runToPositions(int flp, int frp, int blp, int brp, double power){
        shell.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shell.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shell.front_left_motor.setTargetPosition(flp);
        shell.front_right_motor.setTargetPosition(frp);
        
        shell.front_left_motor.setPower(power);
        shell.front_right_motor.setPower(power);
        shell.back_left_motor.setPower(shell.front_right_motor.getPower());
        shell.back_right_motor.setPower(shell.front_left_motor.getPower());
        telemetry.addData("frm power", shell.front_right_motor.getPower());
    }

    public void drive(){
        double Protate = 0.6*gamepad1.right_stick_x;
        double stick_x = gamepad1.left_stick_x; //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = -gamepad1.left_stick_y; //Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2)
        double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
        double magnitudeMultiplier = 0;

        gyroAngle = 0;

        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        double modifiedTheta = theta + Math.PI/4 - gyroAngle; 

        double thetaInFirstQuad = Math.abs(Math.atan(stick_y/stick_x)); //square to circle conversion
        if(thetaInFirstQuad > Math.PI/4){
            magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
        }
        else if(thetaInFirstQuad <= Math.PI/4){
            magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
        }

        double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2))*magnitudeMultiplier*(1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        double Px = magnitude * Math.cos(modifiedTheta); 
        double Py = magnitude * Math.sin(modifiedTheta);
                        
        if(gamepad1.dpad_up){
            Px = -1;
            Py = 1;
            Protate = 0;
        }
        if(gamepad1.dpad_down){
            Px = 1;
            Py = -1;
            Protate = 0;
        }
        shell.front_left_motor.setPower(Py + Protate);
        shell.back_left_motor.setPower(Px - Protate);
        shell.back_right_motor.setPower(Py - Protate);
        shell.front_right_motor.setPower(Px + Protate);
    }
    public double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = shell.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading = (Math.PI/180)*heading;
        if(heading < 0){
            heading = (2*Math.PI) + heading;
        }
        heading = heading - newZero;
        
        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void updateKeys(){ 
        if(gamepad1.a && cdCheck(useMap1.a, 1000)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)){
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
            toggleMap1.right_bumper = false;
        }
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
            toggleMap1.left_bumper = false;
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
}


