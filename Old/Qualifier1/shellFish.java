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

    public DcMotor hook = null;
    public CRServo badonger = null;

    public DcMotor lifterino = null;
    public Servo cheeserino = null;
    public CRServo wire = null;

    public DcMotor intake = null;
    public Servo intakeServo = null;
    
    public AnalogInput encoder1 = null;
    public AnalogInput encoder2 = null;

    public Servo deadBois = null;

    public ColorSensor groundSensor1 = null;
    public ColorSensor groundSensor2 = null;
    
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
        front_right_motor  = hwMap.dcMotor.get("fr");
        back_left_motor    = hwMap.dcMotor.get("bl");
        back_right_motor   = hwMap.dcMotor.get("br");
        
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hook = hwMap.dcMotor.get("hook");
        hook.setDirection(DcMotor.Direction.REVERSE);
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        badonger = hwMap.crservo.get("badonger");

        lifterino = hwMap.dcMotor.get("lifterino");
        lifterino.setDirection(DcMotor.Direction.FORWARD);
        lifterino.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cheeserino = hwMap.servo.get("cheeserino");
        wire = hwMap.crservo.get("wire");

        intake = hwMap.dcMotor.get("intake");
        intakeServo = hwMap.servo.get("intakeServo");

        encoder1 = hwMap.analogInput.get("encoder1");
        encoder2 = hwMap.analogInput.get("encoder2");

        deadBois = hwMap.servo.get("deadBois");

        groundSensor1 = hwMap.colorSensor.get("groundSensor1");
        groundSensor2 = hwMap.colorSensor.get("groundSensor2");
        
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
