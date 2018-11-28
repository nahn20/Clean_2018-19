package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="drive", group="Pushboat")
public class drive extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    /////////////////
    /*IMU Variables*/
    //[[[[[[[[[[[[[[[
    double newZero = 0;
    int fullRotationCount = 0;
    double previousAngle = 0;
    //]]]]]]]]]]]]]]]
    
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        toggleMap1.y = true;
        while(opModeIsActive()){
            angleOverflow(); //Keep at the beginning of teleop loop
            drive();
            deadBois();
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    ////////////////
    // DRIVE CODE //
    //[[[[[[[[[[[[[[
    public void deadBois(){
        if(toggleMap1.y){
            shell.deadBois.setPosition(0.0);
        }
        else{
            shell.deadBois.setPosition(1.0);
        }
    }
    public void drive(){
        double Protate = 0.6*gamepad1.right_stick_x;
        if(toggleMap1.right_bumper){
            telemetry.addData("CHAD MODE", "ON");
            Protate = gamepad1.right_stick_x;
        }
        double stick_x = gamepad1.left_stick_x; //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = -gamepad1.left_stick_y; //Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2)
        double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
        double magnitudeMultiplier = 0;
        
        if(gamepad1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            gyroAngle = 0;
        }
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

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Theta", theta);
        telemetry.addData("Modified Theta", modifiedTheta);
        telemetry.addData("Magnitude",  magnitude);
        telemetry.addData("Front Left", Py + Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py - Protate);
        telemetry.addData("Front Right", Px + Protate);
        //CCCCCCC      HHH      HHH           AA           DDDDDDD \\
        //C            HHH      HHH          AAAA          DDD   DD\\
        //C            HHHHHHHHHHHH         AA  AA         DDD   DD\\
        //C            HHHHHHHHHHHH        AAAAAAAA        DDD   DD\\
        //C            HHH      HHH       AA      AA       DDD   DD\\
        //CCCCCCC      HHH      HHH      AA        AA      DDDDDDD \\
        if(gamepad1.dpad_up || (toggleMap1.right_bumper && theta > Math.PI/4 && theta <= 3*Math.PI/4)){
            Px = -1;
            Py = 1;
        }
        else if(gamepad1.dpad_left || (toggleMap1.right_bumper && (theta > 3*Math.PI/4 || theta <= -3*Math.PI/4))){
            Px = -1;
            Py = -1;
        }
        else if(gamepad1.dpad_down || (toggleMap1.right_bumper && theta < -Math.PI/4 && theta >= -3*Math.PI/4)){
            Px = 1;
            Py = -1;
        }
        else if(gamepad1.dpad_right || (toggleMap1.right_bumper && theta > -Math.PI/4 && theta <= Math.PI/4 && !(stick_y == 0 && stick_x == 0))){
            Px = 1;
            Py = 1;
        }
        else if(toggleMap1.right_bumper){
            Px = 0;
            Py = 0;
        }
        shell.front_left_motor.setPower(Py + Protate);
        shell.back_left_motor.setPower(Px - Protate);
        shell.back_right_motor.setPower(Py - Protate);
        shell.front_right_motor.setPower(Px + Protate);
    }
    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0 
        double heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI/2 && previousAngle > 3*Math.PI/2){
            fullRotationCount++;
        }
        if(heading > 3*Math.PI/2 && previousAngle < Math.PI/2){
            fullRotationCount--;
        }
        previousAngle = heading;
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
    
    //]]]]]]]]]]]]]]
    
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
    
    public void updateKeys(){ 
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
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
    
    //]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
}


