package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="tickRateTest", group="Pushboat")
public class tickRateTest extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    
    //Highest I've seen is 30
    double startTime = 0;
    int tickCount = 0;
    @Override
    public void runOpMode() {
        
        while(!opModeIsActive()){
        }
        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            tickCount();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void tickCount(){
        tickCount++;
        telemetry.addData("Tick Count", tickCount);
        telemetry.addData("Average Tick Rate", tickCount/(runtime.milliseconds()-startTime));
        if(gamepad1.a){
            telemetry.addData(">", "Restarting");
            tickCount = 0;
            startTime = runtime.milliseconds();
        }
    }
}