package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="mainTeleOp", group="Pushboat")
public class mainTeleOp extends LinearOpMode {
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
    
    //Tick Rate
    int tickCount = 0;
    double startTime = 0;
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
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            angleOverflow(); //Keep at the beginning of teleop loop
            tickCount();
            //Player 1
            drive();
            if(gamepad2.b && cdCheck(useMap2.b, 500)){
                telemetry.update();//THIS GOES AT THE END
            }
        }
    }
    public void tickCount(){
        tickCount++;
        if(gamepad1.a){
            tickCount = 0;
            startTime = runtime.milliseconds();
        }
        customTelemetryDouble("Tick Count", tickCount);
        customTelemetryDouble("Average Tick Rate", tickCount/(runtime.milliseconds()-startTime));
    }
    public void drive(){
        double stick_x = gamepad1.left_stick_x;
        double stick_y = -gamepad1.left_stick_y;
        double Px = 0;
        double Py = 0;
        double Protate = 0;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        if(toggleMap1.right_bumper || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down){
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
            Protate = gamepad1.right_stick_x;
            shell.front_left_motor.setPower(Py + Protate);
            shell.back_left_motor.setPower(Px - Protate);
            shell.back_right_motor.setPower(Py - Protate);
            shell.front_right_motor.setPower(Px + Protate);
        }
        else{
            Protate = 0.6*gamepad1.right_stick_x;
            if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
                Protate = gamepad1.right_stick_x;
            }
            double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double magnitudeMultiplier = 0;
            
            if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
                gyroAngle = 0;
            }
            double modifiedTheta = theta + Math.PI/4 - gyroAngle; 

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y/stick_x)); //square to circle conversion
            if(thetaInFirstQuad > Math.PI/4){
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            }
            else if(thetaInFirstQuad <= Math.PI/4){
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }

            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2))*magnitudeMultiplier*(1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
            Px = magnitude * Math.cos(modifiedTheta); 
            Py = magnitude * Math.sin(modifiedTheta);

            shell.front_left_motor.setPower(Py + Protate);
            shell.back_left_motor.setPower(Px - Protate);
            shell.back_right_motor.setPower(Py - Protate);
            shell.front_right_motor.setPower(Px + Protate);
        }
    }

    //Other Function
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
    
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
    
    public void updateKeys(){ 
        if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)){ //Use burton's stupid angle drive
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
        }
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){ //Chad mode
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
        }
        if(gamepad2.dpad_up && cdCheck(useMap2.dpad_up, 500)){ //Not used for anything
            toggleMap2.dpad_up = true;
            useMap2.dpad_up = runtime.milliseconds();
            toggleMap2.dpad_left = false;
            toggleMap2.dpad_down = false;
            toggleMap2.a = false;
        }
        if(gamepad2.dpad_left && cdCheck(useMap2.dpad_left, 500)){ //Not used for anything
            toggleMap2.dpad_left = true;
            useMap2.dpad_left = runtime.milliseconds();
            toggleMap2.dpad_up = false;
            toggleMap2.dpad_down = false;
        }
        if(gamepad2.dpad_down && cdCheck(useMap2.dpad_down, 500)){ //Not used for anything
            toggleMap2.dpad_down = true;
            useMap2.dpad_down = runtime.milliseconds();
            toggleMap2.dpad_up = false;
            toggleMap2.dpad_left = false;
        }
        if(gamepad2.dpad_right){
            toggleMap2.dpad_up = false;
            toggleMap2.dpad_left = false;
            toggleMap2.dpad_down = false;
        }
        if(gamepad2.b){ //Used for telemetry stuff
            useMap2.b = runtime.milliseconds();
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
    public void customTelemetryDouble(string 1, double 2){
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            telemetry.addData(1, 2);
        }
    }
    public void customTelemetryString(string 1, string 2){
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            telemetry.addData(1, 2);
        }
    }
}


