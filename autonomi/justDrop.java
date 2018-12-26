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

@Autonomous(name="justDrop", group="Sheldor Autonomae")
public class justDrop extends LinearOpMode{
    
    private ElapsedTime runtime = new ElapsedTime();
    shellFish shell = new shellFish();
    @Override
    public void runOpMode(){
        telemetry.addData(">", "Wait");
        telemetry.update();
        shell.init(hardwareMap);
        AutoTransitioner.transitionOnStop(this, "mainTeleOp");

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
        shell.hook.setPower(-0.40);
        shell.badonger.setPower(-0.6);
        while(runtime.milliseconds()-startTime < 3000 && opModeIsActive()){}
        shell.hook.setPower(0);
        shell.badonger.setPower(0);
        driveTime(500, 0.7, 0.7);
    }
    public void driveTime(double time, double Px, double Py){
        double start = runtime.milliseconds();
        while(runtime.milliseconds() - start < time && opModeIsActive()){
            shell.front_left_motor.setPower(Py);
            shell.back_left_motor.setPower(Px);
            shell.back_right_motor.setPower(Py);
            shell.front_right_motor.setPower(Px);
        }
        shell.front_left_motor.setPower(0);
        shell.back_left_motor.setPower(0);
        shell.back_right_motor.setPower(0);
        shell.front_right_motor.setPower(0);
    }
    
}