package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shellFish{
    //////////////////
    /* DECLARATIONS */
    //////////////////
    
    //public ColorSensor colorSensor = null;
    
    
    //DRIVE//
    public DcMotor front_left_motor   = null;
    public DcMotor front_right_motor  = null;
    public DcMotor back_left_motor    = null;
    public DcMotor back_right_motor   = null;

    public DcMotor chadArm = null;

    public DcMotor hook = null;

    public DcMotor slide = null;
    public Servo cardboardFlip = null;

    public DcMotor intake = null;

    public AnalogInput encoderX = null;
    public AnalogInput encoderY = null;
    
    //IMU//
    BNO055IMU imu;

    HardwareMap hwMap           =  null;

    /* Constructor */
    public shellFish(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////
        
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        
        //DRIVE//
        front_left_motor   = hwMap.dcMotor.get("fl");
        back_left_motor    = hwMap.dcMotor.get("bl");
        back_right_motor   = hwMap.dcMotor.get("br");
        front_right_motor  = hwMap.dcMotor.get("fr");
        
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        chadArm = hwMap.dcMotor.get("chadArm");

        hook = hwMap.dcMotor.get("hook");
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide = hwMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cardboardFlip = hwMap.servo.get("cardboardFlip");

        intake = hwMap.dcMotor.get("intake");

        encoderX = hwMap.analogInput.get("encoderX");
        encoderY = hwMap.analogInput.get("encoderY");

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
        //////////////////
        /* STUFFY STUFF */
        //////////////////

    }
    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}
