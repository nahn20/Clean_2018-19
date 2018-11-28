package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="craterSingleAuto", group="Sheldor Autonomae")
public class craterSingleAuto extends LinearOpMode{
    
    private ElapsedTime runtime = new ElapsedTime();
    shellFish shell = new shellFish();

    encoderClass encoder1 = new encoderClass();
    encoderClass encoder2 = new encoderClass();
    String cheese = "center"; //Better to use int, but here I get to write string cheese

    double x = 0; //From drop, driving out is +x
    double y = 0; //From drop, driving left is +y, driving right is -y
    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;

    //PID STUFF\\
    double kRotate[] = {0, 0, 0}; //PID constants for rotation
    double kDrive[] = {.21, 6.82, 3.72}; //PID constants for linear drive power

    double errorDrive = 0;
    double errorRotate = 0;
    double integralDrive = 0;
    double derivativeDrive = 0;
    double integralRotate = 0;
    double derivativeRotate = 0;

    double lastTime = 0;
    @Override
    public void runOpMode(){
        telemetry.addData(">", "Wait");
        telemetry.update();
        shell.init(hardwareMap);
        //AutoTransitioner.transitionOnStop(this, "RedFar_TunaOp");

        telemetry.addData(">", "Loading IMU.");
        telemetry.update();
        shell.imu();
        
        telemetry.addData(">", "Ready ayaya");
        telemetry.update();

        while(!opModeIsActive()){
        }
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        // S T A R T /\/\ S T A R T /\/\ S T A R T /\/\ S T A R T \\
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        
        double startTime = runtime.milliseconds();
        shell.deadBois.setPosition(1.0);
        encoder1.init(shell.encoder1.getVoltage());
        encoder2.init(shell.encoder2.getVoltage());
        if(cheese == "center"){
            doublePID(-14.4, -0.38, 0, 0.5, 0);
            doublePID(-7.2, -1.09, 0, 0.5, 0);
        }
        doublePID(-5.07, -23.25, 0, 0.5, 0);
        doublePID(7.8, -34.5, 0, 0.5, 0);
        //doublePID(0, 0, 0, 0.3, 0);
    
    }
    public void doublePID(double desiredX, double desiredY, double desiredAngle, double driveMargin, double rotateMargin){
        updateCoordinates();
        boolean running = true;
        double driveTime = runtime.milliseconds();
        double errorX = x-desiredX;
        double errorY = desiredY-y;
        double tempErrorDrive = errorDrive;
        double tempErrorRotate = errorRotate;
        errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)); //aww shit we're using polar :(
        errorRotate = angle - desiredAngle;
        lastTime = runtime.milliseconds();
        while(runtime.milliseconds() - driveTime < 500){
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
            errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
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
            if(Math.abs(errorDrive) > 2){
                integralDrive = 0;
            }
            derivativeDrive = (tempErrorDrive-errorDrive)/deltaT;
            pDrive = kDrive[0]*errorDrive + kDrive[1]*integralDrive + kDrive[2]*derivativeDrive;

            integralRotate += errorRotate*deltaT/10000;
            if(Math.abs(errorRotate) > Math.PI/8){
                integralRotate = 0;
            }
            derivativeRotate = (tempErrorRotate-errorRotate)/deltaT;
            pRotate = kRotate[0]*errorRotate + kRotate[1]*integralRotate + kRotate[2]*derivativeRotate;
            telemetry.addData("Proportional", kDrive[0]*errorDrive);
            telemetry.addData("Integral", kDrive[1]*integralDrive);
            telemetry.addData("Derivative", kDrive[2]*derivativeDrive);
            telemetry.addData("DriveAtTheta", 180*theta/Math.PI);
            telemetry.addData("DriveLinearPower", pDrive);
            telemetry.addData("DriveRotatePower", pRotate);
            telemetry.addData("Error Drive", errorDrive);
            telemetry.addData("Error Rotate", errorRotate);
            telemetry.update();
            autoDrive(theta, pDrive, pRotate);
            if(Math.abs(errorDrive) > driveMargin){
                driveTime = runtime.milliseconds();
            }
            lastTime = runtime.milliseconds();
        }
        shell.front_left_motor.setPower(0);
        shell.back_left_motor.setPower(0);
        shell.back_right_motor.setPower(0);
        shell.front_right_motor.setPower(0);
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
        encoder1.update(shell.encoder1.getVoltage());
        encoder2.update(shell.encoder2.getVoltage());
        angleOverflow();
        angle = getHeading();
        double deltaYAngle = encoder2.deltaAngle;
        double deltaXAngle = -encoder1.deltaAngle;
        double movementAngle = Math.atan2(deltaYAngle, deltaXAngle);
        x -= Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.cos(movementAngle-angle);
        y += Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.sin(movementAngle-angle);
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
}