package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    float newZero = 0;
    int fullRotationCount = 0;
    float previousAngle = 0;
    //]]]]]]]]]]]]]]]
    
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        while(opModeIsActive()){
            angleOverflow(); //Keep at the beginning of teleop loop
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    ////////////////
    // DRIVE CODE //
    //[[[[[[[[[[[[[[
    
    public void drive(){
        double Protate = gamepad1.right_stick_x/4;
        double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
        double theta = 0;
        double Px = 0;
        double Py = 0;
        float gyroAngle = getHeading(); //In radiants, proper rotation, yay
        
        if(gamepad1.right_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            gyroAngle = 0;
        }
        
        theta = Math.atan2(stick_y, stick_x) + Math.PI/4 - gyroAngle;
        double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2));
        Px = magnitude * Math.cos(theta);
        Py = magnitude * Math.sin(theta);
                          
        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  magnitude);
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);
                          
        front_left_wheel.setPower(Py - Protate);
        back_left_wheel.setPower(Px - Protate);
        back_right_wheel.setPower(Py + Protate);
        front_right_wheel.setPower(Px + Protate);
    }
    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0 
        float heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI && previousAngle > Math.PI){
            fullRotationCount++;
        }
        if(heading > Math.PI && previousAngle < Math.PI){
            fullRotationCount--;
        }
        previousAngle = heading;
    }
    public float getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
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
    
    //]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
}


