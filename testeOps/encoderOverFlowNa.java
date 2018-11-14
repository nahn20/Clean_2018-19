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
    

    double voltageToAngle = 2*Math.PI/3.28;
    double[] voltageArray = new double[10000];

    double highestValue = 0;

    double[] startingVoltage = new double[3];
    double[] previousVoltage = new double[3];
    int[] rotationCounter = new int[3];
    int[] direction = new int[3];

    double[] encoderAngle = new double[3];

    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {

        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        startingVoltage[0] = shell.encoder1.getVoltage();
        previousVoltage[0] = shell.encoder1.getVoltage();
        while(opModeIsActive()){

            updateEncoderAngle(shell.encoder1.getVoltage(), 0);
            voltageArray();
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    ///////////
    // TESTS //
    ///////////
    boolean tooFast = false;
    public void updateEncoderAngle(double voltage, int n){
        double range = 1;
        telemetry.addData("Previous Voltage " + n, previousVoltage[n]);
        telemetry.addData("Voltage " + n, voltage);
        if(previousVoltage[n] < range && voltage > 3.28 - range){
            rotationCounter[n]--;
            telemetry.addData("-----------", "------");
        }
        else if(previousVoltage[n] > 3.28 - range && voltage < range){
            rotationCounter[n]++;
            telemetry.addData("+++++++++++", "+++++++");
        }
        else if(Math.abs(voltage-previousVoltage[n]) > range){
            rotationCounter[n] += direction[n];
        }
        if(voltage < 3 && voltage > 0.3 && previousVoltage[n] < 3 && previousVoltage[n] > 0.3){
            if(voltage - previousVoltage[n] > 0.1){
                direction[n] = 1;
            }
            if(voltage - previousVoltage[n] < -0.1){
                direction[n] = -1;
            }
        }
        if(tooFast){
            telemetry.addData("monkaS", "too fast");
        }
        previousVoltage[n] = voltage;
        encoderAngle[n] = voltageToAngle*(rotationCounter[n]*3.28+voltage-startingVoltage[n]);
        telemetry.addData("Direction", direction[n]);
        telemetry.addData("Encoder Angle P1", voltageToAngle*rotationCounter[n]*3.28);
        telemetry.addData("Encoder Angle P2", voltageToAngle*(voltage));
        telemetry.addData("Rotation Count " + n, rotationCounter[n]);
        telemetry.addData("Encoder Angle " + n, encoderAngle[n]);
        telemetry.addData("Encoder Angle " + n, encoderAngle[n]/Math.PI + " pi radians");
        telemetry.addData("Rotations " + n, encoderAngle[n]/(2*Math.PI));
    }
    public void voltageArray(){
        double range = 1.5;
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
        // if(o >= 20){
        //     for(int u = o-20; u < o; u++){
        //         telemetry.addData(">", voltageArray[u]);
        //     }
        // }
        for(int u = 100; u < 200; u++){
            telemetry.addData(">", voltageArray[u]);    
        }
        // boolean h = false;
        // int u = 1;
        // while(h == false){
        //     if(Math.abs(voltageArray[u]-voltageArray[u+1]) > range && !(voltageArray[u] < range  && voltageArray[u+1] > 3.28-range) && !(voltageArray[u] > 3.28-range && voltageArray[u+1] < range)){
        //         telemetry.addData("u", voltageArray[u]);
        //         telemetry.addData("u+1", voltageArray[u+1]);
        //     }
        //     u++;
        //     if(voltageArray[u] == 0 && voltageArray[u+1] == 0 && voltageArray[u+2] == 0){
        //         h = true;
        //     }
        // }
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


