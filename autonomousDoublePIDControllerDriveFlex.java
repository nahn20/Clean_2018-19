package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="autonomousDoublePIDControllerDriveFlex", group="Pushboat")
public class autonomousDoublePIDControllerDriveFlex extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();

    encoderClass encoder1 = new encoderClass();
    encoderClass encoder2 = new encoderClass();

    double x = 0; //From drop, driving out is +x
    double y = 0; //From drop, driving left is +y, driving right is -y
    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;
    double savedX = 3;
    double savedY = 3;
    double savedAngle = Math.PI/2;
    ////////////////
    // PID Stuffs \\

    //PID constants are [p, i, d] in order. 
    double kRotate[] = {0, 0, 0}; //PID constants for rotation
    double kDrive[] = {.21, 6.82, 3.72}; //PID constants for linear drive power

    double errorDrive = 0;
    double errorRotate = 0;
    double integralDrive = 0;
    double derivativeDrive = 0;
    double integralRotate = 0;
    double derivativeRotate = 0;

    double lastTime = 0;


    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        //toggleMap2.b = true;
        toggleMap1.x = true;
        //toggleMap1.left_bumper = true;
        encoder1.init(shell.encoder1.getVoltage());
        encoder2.init(shell.encoder2.getVoltage());
        lastTime = runtime.milliseconds();
        while(opModeIsActive()){
            //Priority 1
            updateKeys();
            encoder1.update(shell.encoder1.getVoltage());
            encoder2.update(shell.encoder2.getVoltage());
            angleOverflow();
            updateCoordinates();
            //Priority 2
            deadBois();
            constantModifier();
            if(!toggleMap1.x){
                drive();
            }
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Full Rotation Count", fullRotationCount);
            telemetry.addData("Angle", 180*getHeading()/Math.PI);

            if(toggleMap1.right_bumper){
                telemetry.addData("Going to", 0 + " " + 0 + " " + 0);
                doublePID(0, 0, 0);
            }
            else if(toggleMap1.left_bumper){
                telemetry.addData("Going to", savedX + " " + savedY + " " + savedAngle);
                doublePID(savedX, savedY, savedAngle);
            }
            else{
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.b){
                savedX = x;
                savedY = y;
                savedAngle = getHeading();
            }
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void constantModifier(){
        //Usage:
        //B toggled on modifies kRotate. B toggled off modified kPower
        //Y toggled on modifies kp 
        //X toggled on modifies ki
        //A toggled on modifies kd
        //Holding right bumper increases increment amount.
        double incrementAmount = 0.01;
        if(gamepad2.left_trigger > 0){
            incrementAmount = 0.1;
        }
        if(gamepad2.dpad_down){
            incrementAmount = -incrementAmount; //Subtracts instead if using dpad_down
        }
        if(toggleMap2.b){
            telemetry.addData("Editing", "Rotation Constants");
            telemetry.addData("kP", kRotate[0]);
            telemetry.addData("kI", kRotate[1]);
            telemetry.addData("kD", kRotate[2]);
        }
        else if(!toggleMap2.b){
            telemetry.addData("Editing", "Drive Constants");
            telemetry.addData("kP", kDrive[0]);
            telemetry.addData("kI", kDrive[1]);
            telemetry.addData("kD", kDrive[2]);
        }
        if(toggleMap2.y){
            telemetry.addData("Editing", "kP");
        }
        if(toggleMap2.x){
            telemetry.addData("Editing", "kI");
        }
        if(toggleMap2.a){
            telemetry.addData("Editing", "kD");
        }
        if((gamepad2.dpad_up && cdCheck(useMap2.dpad_up, 20)) || (gamepad2.dpad_down && cdCheck(useMap2.dpad_down, 20))){ //More compact this way
            if(toggleMap2.b){
                if(toggleMap2.y){
                    kRotate[0] += incrementAmount;
                }
                if(toggleMap2.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kRotate[1] += incrementAmount;
                }
                if(toggleMap2.a){
                    kRotate[2] += incrementAmount;
                }
            }
            else if(!toggleMap2.b){
                if(toggleMap2.y){
                    kDrive[0] += incrementAmount;
                }
                if(toggleMap2.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kDrive[1] += incrementAmount;
                }
                if(toggleMap2.a){
                    kDrive[2] += incrementAmount;
                }
            }
        }
        for(int i = 0; i < 3; i++){
            if(kRotate[i] < 0){
                kRotate[i] = 0;
            }
            if(kDrive[i] < 0){
                kDrive[i] = 0;
            }
        }
    }
    //I'm leaving a lot of notes labeled TODOInAutonomous because they're things I need to do once this is converted to an autonomous
    public void doublePID(double desiredX, double desiredY, double desiredAngle){
        updateCoordinates();
        double errorX = x-desiredX;
        double errorY = desiredY-y;
        double tempErrorDrive = errorDrive;
        double tempErrorRotate = errorRotate;
        errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)); //aww shit we're using polar :(
        errorRotate = angle - desiredAngle;
        //lastTime = runtime.milliseconds(); //TODOInAutonomous: Uncomment this. Makes it so deltaT isn't f'd by long time in between drives
        if(true){ //TODOInAutonomous: Switch this to while(){}
            updateCoordinates();
            double theta = Math.atan2(errorY, errorX);
            if(theta < 0){
                theta += Math.PI;
            }
            theta = -(theta-Math.PI);
            double deltaT = (runtime.milliseconds() - lastTime); //Delta time. Subtracts last time of tick from current time
            telemetry.addData("deltaT", deltaT);
            errorX = x-desiredX; //Code repetition is unnecessary in teleop, but needed in auto
            errorY = desiredY-y;
            tempErrorDrive = errorDrive;
            tempErrorRotate = errorRotate;
            errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)); //aww shit we're using polar :(
            if(errorY < 0){
                errorDrive *= -1;
            }
            errorRotate = angle - desiredAngle;
            //integralDrive; Just reminding myself that these are things
            //derivativeDrive;
            double pDrive = 0;
            //integralRotate;
            //derivativeRotate;
            double pRotate = 0;
            
            integralDrive += errorDrive*deltaT/10000;
            if(Math.abs(errorDrive) > 1){
                integralDrive = 0;
            }
            derivativeDrive = (tempErrorDrive-errorDrive)/deltaT;
            pDrive = kDrive[0]*errorDrive + kDrive[1]*integralDrive + kDrive[2]*derivativeDrive;
            telemetry.addData("Proportional", kDrive[0]*errorDrive);
            telemetry.addData("Integral", kDrive[1]*integralDrive);
            telemetry.addData("Derivative", kDrive[2]*derivativeDrive);

            integralRotate += errorRotate*deltaT/10000;
            if(Math.abs(errorRotate) > Math.PI/8){
                integralRotate = 0;
            }
            derivativeRotate = (tempErrorRotate-errorRotate)/deltaT;
            pRotate = kRotate[0]*errorRotate + kRotate[1]*integralRotate + kRotate[2]*derivativeRotate;
            telemetry.addData("DriveAtTheta", 180*theta/Math.PI);
            telemetry.addData("DriveLinearPower", pDrive);
            telemetry.addData("DriveRotatePower", pRotate);
            telemetry.addData("Error Rotate", errorRotate);
            if(toggleMap1.x){
                autoDrive(theta, pDrive, pRotate);
            }
            lastTime = runtime.milliseconds();
        }
        lastTime = runtime.milliseconds(); //TODOInAutonomous: Delete this
    }
    public void autoDrive(double theta, double magnitude, double Protate){
        double modifiedTheta = theta + Math.PI/4 - angle; 

        magnitude *= (1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        double Px = magnitude * Math.cos(modifiedTheta); 
        double Py = magnitude * Math.sin(modifiedTheta);

        shell.front_left_motor.setPower(Py + Protate);
        shell.back_left_motor.setPower(Px - Protate);
        shell.back_right_motor.setPower(Py - Protate);
        shell.front_right_motor.setPower(Px + Protate);
    }
    public void updateCoordinates(){
        angle = getHeading();
        double deltaYAngle = encoder2.deltaAngle;
        double deltaXAngle = -encoder1.deltaAngle;
        double movementAngle = Math.atan2(deltaYAngle, deltaXAngle);
        x -= Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.cos(movementAngle-angle);
        y += Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.sin(movementAngle-angle);
        // y += Math.cos(angle)*(-encoder1.deltaAngle + encoder2.deltaAngle)/2;
        // x -= Math.cos(angle)*encoder3.deltaAngle;
        // angle = 2*Math.PI*(y - encoder1.angle)/11.285;
        // telemetry.addData("1", encoder1.angle);
        // telemetry.addData("2", encoder2.angle);
        // telemetry.addData("3", encoder3.angle);
        // telemetry.addData("parallel dif", encoder1.angle/encoder2.angle);
        // telemetry.addData("Conversion Factor", getHeading()/(y - encoder1.angle));
    }
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

        // telemetry.addData("Stick_X", stick_x);
        // telemetry.addData("Stick_Y", stick_y);
        // telemetry.addData("Theta", theta);
        // telemetry.addData("Modified Theta", modifiedTheta);
        // telemetry.addData("Magnitude",  magnitude);
        // telemetry.addData("Front Left", Py + Protate);
        // telemetry.addData("Back Left", Px - Protate);
        // telemetry.addData("Back Right", Py - Protate);
        // telemetry.addData("Front Right", Px + Protate);
                          
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
        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }

    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void updateKeys(){ //a, x, and y are conflicting keys
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
        }
        if(gamepad2.a && cdCheck(useMap2.a, 500)){
            toggleMap2.a = toggle(toggleMap2.a);
            useMap2.a = runtime.milliseconds();
            toggleMap2.y = false;
            toggleMap2.x = false;
        }
        if(gamepad2.x && cdCheck(useMap2.x, 500)){
            toggleMap2.x = toggle(toggleMap2.x);
            useMap2.x = runtime.milliseconds();
            toggleMap2.y = false;
            toggleMap2.a = false;
        }
        if(gamepad2.y && cdCheck(useMap2.y, 500)){
            toggleMap2.y = toggle(toggleMap2.y);
            useMap2.y = runtime.milliseconds();
            toggleMap2.x = false;
            toggleMap2.a = false;
        }
        if((gamepad1.left_bumper || gamepad2.left_bumper) && cdCheck(useMap1.left_bumper, 500)){ //Bumpers on both controllers do the same thing
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
            toggleMap1.right_bumper = false;
        }
        if((gamepad1.right_bumper || gamepad2.right_bumper) && cdCheck(useMap1.right_bumper, 500)){
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


