package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    int chadArmPos = 0;
    int armMax = 1994;

    int chadSlidePos = 0;
    int slideMax = -3789;

    int hookPos = 0;
    int hookMax = 0;

    int cleanRaiseStage = 0;
    int cleanLowerStage = 0;

    //int chungoidPos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode(){
        shell.init(hardwareMap);
        shell.imu();

        shell.front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shell.back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shell.back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shell.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shell.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shell.back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shell.back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shell.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while(!opModeIsActive()){
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                shell.chadExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shell.chadArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shell.chadArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shell.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData(">", "RESETTETTED");
            }
            telemetry.update();
        }
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            angleOverflow(); //Keep at the beginning of teleop loop
            //Player 1
            drive();
            chungoids();
            //Player 2
            cleanRaise();
            cleanLower();
            intake();
            if(!toggleMap2.dpad_down){
                chadArm();
                chadExtend();
            }
            hook();
            //telemetry.update();
            if(toggleMap2.dpad_up || toggleMap2.dpad_left || toggleMap2.dpad_down){
                telemetry.addData("Chungoid 1", shell.chungoid1.getPosition());
                telemetry.update();//THIS GOES AT THE END
            }
        }
    }
    //Player 1
    public void drive(){
        double stick_x = gamepad1.left_stick_x;
        double stick_y = -gamepad1.left_stick_y;
        double Px = 0;
        double Py = 0;
        double Protate = 0;
        int stupidAdwithMultiplier = 1;
        if(toggleMap1.left_bumper){
            stupidAdwithMultiplier = -1;
        }
        if(toggleMap1.x){
            stick_x *= 0.5;
            stick_y *= 0.5;
        }
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
            Protate = -gamepad1.right_stick_x;
            shell.front_left_motor.setPower(stupidAdwithMultiplier*Py - Protate);
            shell.back_left_motor.setPower(stupidAdwithMultiplier*Px + Protate);
            shell.back_right_motor.setPower(stupidAdwithMultiplier*Py + Protate);
            shell.front_right_motor.setPower(stupidAdwithMultiplier*Px - Protate);
        }
        else{
            Protate = 0.6*gamepad1.right_stick_x;
            if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
                Protate = gamepad1.right_stick_x;
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double gyroAngle = 0;
            double magnitudeMultiplier = 0;
            
            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
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

            shell.front_left_motor.setPower(stupidAdwithMultiplier*Py + Protate);
            shell.back_left_motor.setPower(stupidAdwithMultiplier*Px - Protate);
            shell.back_right_motor.setPower(stupidAdwithMultiplier*Py - Protate);
            shell.front_right_motor.setPower(stupidAdwithMultiplier*Px + Protate);
        }
    }
    /*public void chungoid(){ //Motor
        if(gamepad1.y){ //High
            chungoidPos = 0;
            clearFancyFunctions();
        }
        else if(gamepad1.b){
            chungoidPos = 0;
            clearFancyFunctions();
        }
        else if(gamepad1.a){ //Low
            chungoidPos = chungoidMax;
            clearFancyFunctions();
        }
        else if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
            chungoidPos += gamepad1.right_trigger*10;
            chungoidPos -= gamepad1.left_trigger*10;
            // if(chungoidPos < 0){
            //     chungoidPos = 0;
            // }
            // if(chungoidPos > chungoidMax){
            //     chungoidPos = chungoidMax;
            // }
            clearFancyFunctions();
        }
        if(!toggleMap2.dpad_down){
            shell.chungoid.setTargetPosition(chungoidPos);
            shell.chungoid.setPower(0.6);
        }
        if(toggleMap2.dpad_down){
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                shell.chungoid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chungoid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            shell.chungoid.setPower(0);
            telemetry.addData("Chungoid", shell.chungoid.getCurrentPosition());
        }
        telemetry.addData("chngoid pos", chungoidPos);
        telemetry.addData("Chungoid", shell.chungoid.getCurrentPosition());
        telemetry.update();
    }*/
    public void chungoids(){
        if(gamepad1.y){
            setChungoids(0);
        }
        else if(gamepad1.a){
            setChungoids(.84);
        }
        else if(gamepad1.b){
            setChungoids(.58);
        }
        else if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
            double pos = shell.chungoid1.getPosition() + gamepad1.right_trigger/30 - gamepad1.left_trigger/30;
            // if(pos > .92){
            //     pos = .92;
            // }
            setChungoids(pos);
        }
    }
    public void setChungoids(double pos){
        shell.chungoid1.setPosition(pos);
        shell.chungoid2.setPosition(1-pos);
    }
    //Player 2
    public void intake(){
        if(gamepad2.left_trigger > 0){ //Outtake
            shell.intake1.setPower(gamepad2.left_trigger);
            shell.intake2.setPower(-gamepad2.left_trigger);
        }
        else if(toggleMap2.left_bumper){ //Intake
            shell.intake1.setPower(-1);
            shell.intake2.setPower(1);
            telemetry.addData("Boop", "Boop");
        }
        else{
            shell.intake1.setPower(0);
            shell.intake2.setPower(0);
        }
    }
    public void chadArm(){
        if((gamepad2.right_stick_y > 0 || gamepad2.right_stick_x < 0) && gamepad2.right_bumper && Math.abs(shell.chadExtend.getCurrentPosition()-slideMax) > 300){ //Low
            chadArmPos = 0;
            clearFancyFunctions();
        }
        else if((gamepad2.right_stick_y < 0 || gamepad2.right_stick_x > 0) && gamepad2.right_bumper){ //High
            chadArmPos = slideMax;
            clearFancyFunctions();
        }
        if(!gamepad2.right_bumper){
            chadArmPos += (-gamepad2.right_stick_y*50) + (gamepad2.right_stick_x*200);
            if(chadArmPos < 0){
                chadArmPos = 0;
            }
            if(chadArmPos > armMax){
                chadArmPos = armMax;
            }
            if(gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0){
                clearFancyFunctions();
            }
        }
        if(!toggleMap2.dpad_up){
            shell.chadArm1.setTargetPosition(chadArmPos);
            shell.chadArm2.setTargetPosition(chadArmPos);
            shell.chadArm1.setPower(1);
            shell.chadArm2.setPower(1);
        }
        if(toggleMap2.dpad_up){
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                shell.chadArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shell.chadArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            shell.chadArm1.setPower(0);
            shell.chadArm2.setPower(0);
            telemetry.addData("Chad Arm", shell.chadArm1.getCurrentPosition());
        }
    }
    public void chadExtend(){
        if((gamepad2.left_stick_y > 0 || gamepad2.left_stick_x < 0) && gamepad2.right_bumper){ //Low
            chadSlidePos = 0;
            clearFancyFunctions();
        }
        else if((gamepad2.left_stick_y < 0 || gamepad2.left_stick_x > 0) && gamepad2.right_bumper){ //High
            chadSlidePos = slideMax;
            clearFancyFunctions();
        }
        if(!gamepad2.right_bumper){
            chadSlidePos -= (-gamepad2.left_stick_y*50) + (gamepad2.left_stick_x*200);
            if(chadSlidePos > 0){
                chadSlidePos = 0;
            }
            if(chadSlidePos < slideMax){
                chadSlidePos = slideMax;
            }
            if(gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0){
                clearFancyFunctions();
            }
        }
        if(!toggleMap2.dpad_left){
            shell.chadExtend.setTargetPosition(chadSlidePos);
            shell.chadExtend.setPower(1);
        }
        if(toggleMap2.dpad_left){
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                shell.chadExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.chadExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            shell.chadExtend.setPower(0);
            telemetry.addData("Chad Extend", shell.chadExtend.getCurrentPosition());
        }
    }
    public void hook(){
        if(!toggleMap2.dpad_down){
            if(!toggleMap2.x){
                hookPos = 0;
            }
            if(toggleMap2.x){
                hookPos = 3260;
            }
        }
        if(toggleMap2.dpad_down){
            hookPos -= Math.round(100*gamepad2.right_stick_y);
            // if(hookPos > 3081){
            //     hookPos = 3081;
            // }
            // if(pos < 0){
            //     pos = 0;
            // }
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                shell.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shell.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("Hook", shell.hook.getCurrentPosition());
        }
        shell.hook.setTargetPosition(hookPos);
        shell.hook.setPower(1);
    }
    public void cleanRaise(){
        if(toggleMap2.y){
            switch(cleanRaiseStage){
                case 0:
                    setChungoids(.6);
                    chadSlidePos = -600;
                    if(Math.abs(shell.chadExtend.getCurrentPosition()-(-600)) < 50){
                        setChungoids(.86);
                        chadArmPos = 1568;
                        cleanRaiseStage++;
                    }
                    break;
                case 1:
                    if(Math.abs(shell.chadArm1.getCurrentPosition()-1568) < 50){
                        chadSlidePos = -1879;
                        cleanRaiseStage++;
                    }
                    break;
                case 2:
                    if(Math.abs(shell.chadExtend.getCurrentPosition()-(-1879)) < 50){
                        cleanRaiseStage = 0;
                        toggleMap2.y = false;
                    }
                    break;
            }
        }
    }
    public void cleanLower(){
        if(toggleMap2.a){
            switch(cleanLowerStage){
                case 0:
                    chadArmPos = 1700;
                    if(Math.abs(shell.chadArm1.getCurrentPosition()-1700) < 50){
                        setChungoids(.4);
                        chadSlidePos = -600;
                        cleanLowerStage++;
                    }
                    break;
                case 1:
                    if(Math.abs(shell.chadExtend.getCurrentPosition()-(-1000)) < 50){
                        chadArmPos = 0;
                        cleanLowerStage++;
                    }
                    break;
                case 2:
                    if(Math.abs(shell.chadArm1.getCurrentPosition()-0) < 50){
                        chadSlidePos = -2600;
                        cleanLowerStage++;
                    }
                    break;
                case 3:
                    if(Math.abs(shell.chadExtend.getCurrentPosition()-(-2600)) < 50){
                        cleanLowerStage = 0;
                        setChungoids(.58);
                        toggleMap2.a = false;
                    }
                    break;
            }
        }
    }
    public void clearFancyFunctions(){
        cleanRaiseStage = 0;
        cleanLowerStage = 0;
        toggleMap2.y = false;
        toggleMap2.a = false;
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
        if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)){ //Weird Gyro mode
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
        }
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){ //Chad mode
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
        }
        if(gamepad2.dpad_up && cdCheck(useMap2.dpad_up, 500)){ //Chad Arm
            toggleMap2.dpad_up = true;
            useMap2.dpad_up = runtime.milliseconds();
            toggleMap2.dpad_left = false;
            toggleMap2.dpad_down = false;
            toggleMap2.a = false;
        }
        if(gamepad2.dpad_left && cdCheck(useMap2.dpad_left, 500)){ //Chad extend
            toggleMap2.dpad_left = true;
            useMap2.dpad_left = runtime.milliseconds();
            toggleMap2.dpad_up = false;
            toggleMap2.dpad_down = false;
        }
        if(gamepad2.dpad_down && cdCheck(useMap2.dpad_down, 500)){ //Hook
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
        if(gamepad2.y && cdCheck(useMap2.y, 500)){ //Clean flip up
            toggleMap2.y = toggle(toggleMap2.y);
            useMap2.y = runtime.milliseconds();
            toggleMap2.a = false;
            cleanRaiseStage = 0;
        }
        if(gamepad2.a && cdCheck(useMap2.a, 500)){ //Clean flip up
            toggleMap2.a = toggle(toggleMap2.a);
            useMap2.a = runtime.milliseconds(); 
            toggleMap2.y = false;
            cleanLowerStage = 0;
        }
        if(gamepad2.x && cdCheck(useMap2.x, 500)){ //Hook
            toggleMap2.x = toggle(toggleMap2.x);
            useMap2.x = runtime.milliseconds();
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){ //Slow drive
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
        }
        if(gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 500)){ //Intake
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper);
            useMap2.left_bumper = runtime.milliseconds();
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


