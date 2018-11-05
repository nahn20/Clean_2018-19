package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="autonomousDoublePIDControllerDriveFlex", group="Pushboat")
public class autonomousDoublePIDControllerDriveFlex extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();

    double x = 0; //From drop, driving out is +x
    double y = 0; //From drop, driving left is +y, driving right is -y
    double angle = 0; //Forward is 0. Angle goes ccw like normal. In radians
    ////////////////
    // PID Stuffs \\

    //PID constants are [p, i, d] in order. 
    float kRotate = [0, 0, 0]; //PID constants for rotation
    float kDrive = [0, 0, 0]; //PID constants for linear drive power

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
        lastTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();

            constantModifier();
            doublePID();

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
        if(gamepad1.right_bumper){
            incrementAmount = 0.1;
        }
        if(gamepad1.dpad_down){
            incrementAmount = -incrementAmount; //Subtracts instead if using dpad_down
        }
        if((gamepad1.dpad_up && cdCheck(useMap1.dpad_up, 20)) || (gamepad1.dpad_down && cdCheck(useMap1.dpad_down, 20))){ //More compact this way
            if(toggleMap1.b){
                if(toggleMap1.y){
                    kRotate[0] += incrementAmount;
                }
                if(toggleMap1.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kRotate[1] += incrementAmount;
                }
                if(toggleMap1.a){
                    kRotate[2] += incrementAmount;
                }
            }
            else if(!toggleMap1.b){
                if(toggleMap1.y){
                    kDrive[0] += incrementAmount;
                }
                if(toggleMap1.x){ //Don't worry these three will never be on at the same time. Read updateKeys();
                    kDrive[1] += incrementAmount;
                }
                if(toggleMap1.a){
                    kDrive[2] += incrementAmount;
                }
            }
        }
    }
    //I'm leaving a lot of notes labeled TODOInAutonomous because they're things I need to do once this is converted to an autonomous
    public void doublePID(double desiredX, double desiredY, double desiredAngle){
        updateCoordinates();
        double errorX = desiredX - x;
        double errorY = desiredY - y;
        double tempErrorDrive = errorDrive;
        double tempErrorRotate = errorRotate;
        errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)); //aww shit we're using polar :(
        errorRotate = desiredAngle - angle;
        //lastTime = runtime.milliseconds(); //TODOInAutonomous: Uncomment this. Makes it so deltaT isn't f'd by long time in between drives
        if(true){ //TODOInAutonomous: Switch this to while(){}
            updateCoordinates();
            double theta = Math.atan(errorY/errorX);
            double deltaT = (runtime.milliseconds() - lastTime); //Delta time. Subtracts last time of tick from current time
            errorX = desiredX - x; //Code repetition is unnecessary in teleop, but needed in auto
            errorY = desiredY - y;
            tempErrorDrive = errorDrive;
            tempErrorRotate = errorRotate;
            errorDrive = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)); //aww shit we're using polar :(
            errorRotate = desiredAngle - angle;
            //integralDrive; Just reminding myself that these are things
            //derivativeDrive;
            double pDrive = 0;
            //integralRotate;
            //derivativeRotate;
            double pRotate = 0;
            
            integralDrive += errorDrive*deltaT;
            derivativeDrive = (errorDrive-tempErrorDrive)/deltaT;
            pDrive = kDrive[0]*errorDrive + kDrive[1]*integralDrive + kDrive[2]*derivativeDrive;

            integralRotate += errorRotate*deltaT;
            derivativeRotate = (errorRotate-tempErrorRotate)/deltaT;
            pRotate = kRotate[0]*errorRotate + kRotate[1]*integralRotate + kRotate[2]*derivativeRotate;

            //drive(theta, pDrive, pRotate);
            lastTime = runtime.milliseconds();
        }
        lastTime = runtime.milliseconds(); //TODOInAutonomous: Delete this
    }
    public void updateCoordinates(){
        //ToAThing
    }

    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void updateKeys(){ //a, x, and y are conflicting keys
        if(gamepad1.b && cdCheck(useMap1.b, 1000)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad1.a && cdCheck(useMap1.a, 1000)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
            toggleMap1.y = false;
            toggleMap1.x = false;
        }
        if(gamepad1.x && cdCheck(useMap1.x, 1000)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
            toggleMap1.y = false;
            toggleMap1.a = false;
        }
        if(gamepad1.y && cdCheck(useMap1.y, 1000)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
            toggleMap1.x = false;
            toggleMap1.a = false;
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


