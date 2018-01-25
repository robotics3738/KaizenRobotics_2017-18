package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumRobot {
    
    HardwareMap hardwareMap = null;
    
    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    public DcMotor LeftLift = null;

    public Servo grabServo = null;

    public ModernRoboticsI2cGyro gyro = null;

    private static double PID4 = (Math.PI / 4);
    private static final double COUNTS_PER_ROTATION = 1120;
    private static final double LIFT_SPEED = 0.65;
    
    private double currentHeight;

    public MecanumRobot() {
        //
    }
    
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        
        // Define and Initialize Motors
        FR = hardwareMap.dcMotor.get("Front Right");
        FL = hardwareMap.dcMotor.get("Front Left");
        BR = hardwareMap.dcMotor.get("Back Right");
        BL = hardwareMap.dcMotor.get("Back Left");

        LeftLift = hardwareMap.dcMotor.get("Left Lift");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        LeftLift.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        LeftLift.setPower(0);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grabServo = hardwareMap.get(Servo.class, "Grab Servo");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");
        gyro.calibrate();
        
        currentHeight = LiftLevel.BOTTOM.getRotations();
    }
    
    public void move(double move_speed, double angleToMove) {
        move(move_speed, angleToMove, 0);
    }
    
    public void move(double move_speed, double angleToMove, double turnModifier) {
        double limit = 1.0;
        
        // MASSIVE THANKS TO http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf
        // For the equaions used for the mecanum drive
        double FL_Power = move_speed*Math.sin(angleToMove + PID4) + turnModifier;
        double FR_Power = move_speed*Math.cos(angleToMove + PID4) - turnModifier;
        double BL_Power = move_speed*Math.cos(angleToMove + PID4) + turnModifier;
        double BR_Power = move_speed*Math.sin(angleToMove + PID4) - turnModifier;
        
        if (Math.abs(FL_Power) > limit) limit = Math.abs(FL_Power);
        if (Math.abs(FR_Power) > limit) limit = Math.abs(FR_Power);
        if (Math.abs(BL_Power) > limit) limit = Math.abs(BL_Power);
        if (Math.abs(BR_Power) > limit) limit = Math.abs(BR_Power);
        
        FL_Power /= limit;
        FR_Power /= limit;
        BL_Power /= limit;
        BR_Power /= limit;
        
        FL.setPower(FL_Power);
        FR.setPower(FR_Power);
        BL.setPower(BL_Power);
        BR.setPower(BR_Power);
    }
    
    public double calculateTurnModifier(double turn_x, double turn_y, double robotHeading) { //This Still Needs Work
        double turnModifier = 0;
        
        double targetHeading = Math.atan2(turn_x, turn_y);
        
        if (targetHeading < 0) {
            targetHeading += (2*Math.PI);
        }
        
        targetHeading = Math.toDegrees(targetHeading);
        
        Double toTurnDouble = new Double(targetHeading - robotHeading);
        int toTurn = toTurnDouble.intValue();
        
        while (toTurn > 179) {
            toTurn -= 180;
        }
        
        while (toTurn < -179) {
            toTurn += 180;
        }
        
        int toTurnABS = Math.abs(toTurn);
        
        return turnModifier;
    }
    
    public void turn(double turnSpeed) {
        FL.setPower(-turnSpeed);
        FR.setPower(turnSpeed);
        BL.setPower(-turnSpeed);
        BR.setPower(turnSpeed);
    }
    
    public void stopRobot() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    
    public double getAngleRelativeToRobot(double move_x, double move_y, double robotHeading) {
        return (Math.atan2(move_x, move_y) - robotHeading);
    }
    
    public void dropBlock() { //this will come from the other rnd
        grabServo.setPosition(1);
    }
    
    public void liftToLevel(LiftLevel targetPosition) {
        
        if ( Math.abs(targetPosition.getRotations() - currentHeight) > 0.001 ) {
        
            double rotationsToMake = targetPosition.getRotations() - currentHeight;
            
            int newLeftTarget = (int)(COUNTS_PER_ROTATION*rotationsToMake);
            
            LeftLift.setTargetPosition(newLeftTarget);
                
            LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            LeftLift.setPower(Math.abs(LIFT_SPEED)); //Do I need the ABS Thing?
            
            currentHeight = targetPosition.getRotations();
        }
    }
}
