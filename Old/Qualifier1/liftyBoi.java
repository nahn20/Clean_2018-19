package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Lifty Boi", group="Pushboat")
public class liftyBoi extends LinearOpMode {
    shellFish shell = new shellFish();
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        shell.init(hardwareMap);
        shell.imu();
        
        while(!opModeIsActive()){
        }
        shell.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeIsActive()){
            updateKeys();
            lift();
            intake();
            telemetry.update();//THIS GOES AT THE END
        }
    }
    public void intake(){
        if(toggleMap2.a){
            shell.intake.setPower(1);
        }
        else if(toggleMap2.b){
            shell.intake.setPower(-1);
        }
        else{
            shell.intake.setPower(0);
        }
    }
    public void lift(){
        telemetry.addData("toggleMap a", toggleMap1.a);
        if(gamepad1.left_stick_y == 0 && (toggleMap1.a || toggleMap1.b)){
            shell.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int min = 0;
            int max = -2586;
            if(toggleMap1.a){
                shell.hook.setTargetPosition(min);
                if(Math.abs(shell.hook.getCurrentPosition()-min) < 10){
                    toggleMap1.a = false;
                }
            }
            if(toggleMap1.b){
                shell.hook.setTargetPosition(max);
                if(Math.abs(shell.hook.getCurrentPosition()-max) < 10){
                    toggleMap1.b = false;
                }
            }
            shell.hook.setPower(1);
        }
        else{
            shell.hook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shell.hook.setPower(gamepad1.left_stick_y);
        }
        telemetry.addData("Position", shell.hook.getCurrentPosition());
    }
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    ////////////////////////////////
    public void updateKeys(){ 
        if(gamepad2.a && cdCheck(useMap2.a, 500)){
            toggleMap2.a = toggle(toggleMap2.a);
            useMap2.a = runtime.milliseconds();
            toggleMap2.b = false;
        }
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
            toggleMap2.a = false;
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


