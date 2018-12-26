package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motorTester", group="Test")
public class motorTester extends LinearOpMode {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    @Override
    public void runOpMode() {
        motor0 = hardwareMap.dcMotor.get("motor0"); //Change "motor" to motor's name in hardware map
        motor1 = hardwareMap.dcMotor.get("motor1");

        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
            motorPower();
            telemetry.update();
        }
    }
    public void motorPower(){
        motor0.setPower(gamepad1.left_stick_y);
        motor1.setPower(gamepad1.right_stick_y);
    }
}