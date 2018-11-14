package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testeOp", group="Test")
public class testeOp extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    

    int fullRotationCounter = 0;
    double voltageToAngle = 360/3.28;
    double[] voltageArray = new double[10000];

    double highestValue = 0;

    double[] startingVoltage = new double[3];
    double[] previousVoltage = new double[3];
    int[] rotationCounter = new int[3];

    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {

        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        startingVoltage[1] = shell.encoder1.getVoltage();
        previousVoltage[1] = shell.encoder1.getVoltage();
        while(opModeIsActive()){
            telemetry.addData("Encoder 1", shell.encoder1.getVoltage()); //11/7/18: working increasing in positives, broken in negatives
            telemetry.addData("Starting Voltage 1", startingVoltage[1]);
            telemetry.addData("Full Rotation Count 1", rotationCounter[1]);
            double angle = returnAngle(shell.encoder1.getVoltage(), 1);
            telemetry.addData("Angle", angle);
            if(shell.encoder1.getVoltage() > highestValue){
                highestValue = shell.encoder1.getVoltage();
            }
            telemetry.addData("Highest", highestValue);
            voltageOverflow();
            voltageArray();
//            colorSensorDisplayHSV();
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    ///////////
    // TESTS //
    ///////////
    public void voltageOverflow(){
        double voltage = shell.encoder1.getVoltage();
         double centeredVoltage = 1.64 + (voltage - startingVoltage[1]);
          if (centeredVoltage > 3.28) {
            centeredVoltage -= 3.28;
        } else if (centeredVoltage < 0) {
            centeredVoltage += 3.28;
        }
        if(centeredVoltage > 3.28 - 0.6 && previousVoltage[1] < 0.6){
            rotationCounter[1]--;
        }
        else if(centeredVoltage < 0.6 && previousVoltage[1] > 3.28-0.6){
            rotationCounter[1]++;
        }
        previousVoltage[1] = centeredVoltage;
    }
    public double returnAngle(double currentVoltage, int encoderNumber){ //Encoder number either 0, 1, 2
        double centeredVoltage = 1.64 + (currentVoltage - startingVoltage[encoderNumber]);
        double dicksuck9000 = 69;
        if (centeredVoltage > 3.28) {
            centeredVoltage -= 3.28;
        } else if (centeredVoltage < 0) {
            centeredVoltage += 3.28;
        }
        if (rotationCounter[encoderNumber] == 0) {
            dicksuck9000 = (centeredVoltage - 1.64) / 1.64 * 180;
        } else if (rotationCounter[encoderNumber] > 0) {
            dicksuck9000 = (rotationCounter[encoderNumber] * 360) - 180 + (currentVoltage / 3.28 * 360);
        } else if (rotationCounter[encoderNumber] < 0) {
            dicksuck9000 = (rotationCounter[encoderNumber] * 360) + 180 - ((3.28 - currentVoltage) / 3.28 * 360);
        }
        return dicksuck9000;
    }
    public void voltageArray(){
        int i = 0;
        int o = 0;
        boolean q = false;
        while(q == false){
            if(i == voltageArray.length){
                break;
            }
            if(voltageArray[i] == 0){
                voltageArray[i] = shell.encoder1.getVoltage();
                q = true;
                o = i;
            }
            i++;
        }
        if(o >= 20){
            for(int u = o-20; u < o; u++){
                telemetry.addData(">", voltageArray[u]);
            }
        }
        // boolean h = false;
        // int u = 0;
        // while(h == false){
        //     telemetry.addData(">", voltageArray[u]);
        //     u++;
        //     if(voltageArray[u] == 0 && voltageArray[u+1] == 0 && voltageArray[u+2] == 0){
        //         h = true;
        //     }
        // }
    }
    /*
    public void colorSensorDisplayHSV(){
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;
        Color.RGBToHSV((int) (shell.colorSensor.red() * scale_factor),
                (int) (shell.colorSensor.green() * scale_factor),
                (int) (shell.colorSensor.blue() * scale_factor),
                hsvValues);
        float hsv = hsvValues[0];
        telemetry.addData("HSV Values", hsv);
        if(hsv > 30 && hsv < 60){
            telemetry.addData("Color", "Yellow");
        }
        if(hsv > 130 && hsv < 170){
            telemetry.addData("Color", "Yellow");
        }
    }
    */
    public void displayEncoder(){
        
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
}


