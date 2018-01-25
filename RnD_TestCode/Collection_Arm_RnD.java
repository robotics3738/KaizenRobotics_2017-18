package org.firstinspires.ftc.teamcode.RnD_TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Collection & Lift Test", group="TeleOp")
public class Collection_Arm_RnD extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    
    static final double COUNTS_PER_ROTATION = 1120;
    static final double LIFT_SPEED = 0.65;
           
    enum  CurrentHeight {
        BOTTOM (0), ABOVE_ONE (1.875), ABOVE_TWO (3.2);
        
        private final double rotations;
        
        CurrentHeight(double rotations) {
            this.rotations = rotations;
        }
        
        double getRotations() { return rotations; }
    }
    
    Servo s;
    
    DcMotor leftLiftMotor = null;
    DcMotor rightLiftMotor = null;
    
    CurrentHeight currentHeight;
    
    @Override
    public void init() {
        s  = hardwareMap.get(Servo.class, "Grab Servo");
    
        leftLiftMotor = hardwareMap.dcMotor.get("Left Lift");
        rightLiftMotor = hardwareMap.dcMotor.get("Right Lift");
    
        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
    
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
        
        currentHeight = CurrentHeight.BOTTOM;
    }
    
    public void moveLift(double speed, CurrentHeight currentPosition, CurrentHeight targetPosition) {
        double rotationsToMake = targetPosition.getRotations() - currentPosition.getRotations();
        int newLeftTarget = leftLiftMotor.getCurrentPosition() + (int)(1120*rotationsToMake);//(leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightLiftMotor.getCurrentPosition() + (int)(1120*rotationsToMake);//(rightInches * COUNTS_PER_INCH);
            
        leftLiftMotor.setTargetPosition(newLeftTarget);
        rightLiftMotor.setTargetPosition(newRightTarget);
            
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftLiftMotor.setPower(Math.abs(speed));
        rightLiftMotor.setPower(Math.abs(speed));

        while(leftLiftMotor.isBusy() || rightLiftMotor.isBusy() ) {
            //Now We Wait
        }
        
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //OVERLOAD
    public void moveLift(double speed, double JumpsToMake) {
        int newLeftTarget = leftLiftMotor.getCurrentPosition() + (int)(100*JumpsToMake);//(leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightLiftMotor.getCurrentPosition() + (int)(100*JumpsToMake);//(rightInches * COUNTS_PER_INCH);
            
        leftLiftMotor.setTargetPosition(newLeftTarget);
        rightLiftMotor.setTargetPosition(newRightTarget);
        
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftLiftMotor.setPower(Math.abs(speed));
        rightLiftMotor.setPower(Math.abs(speed));
        
        while(leftLiftMotor.isBusy() || rightLiftMotor.isBusy() ) {
            /*
            if(!leftLiftMotor.isBusy()) {
                leftLiftMotor.setPower(0);
            }
            if(!rightLiftMotor.isBusy()) {
                rightLiftMotor.setPower(0);
            }
            */
        }
        /*
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
        */
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        //23 2nd
        //39 3rd
        if(gamepad1.a) {
            s.setPosition(360);
        }
        else if(gamepad1.b) {
            s.setPosition(0);
        }
        if(gamepad1.dpad_down) {
            moveLift(LIFT_SPEED, currentHeight, CurrentHeight.BOTTOM);
            currentHeight = CurrentHeight.BOTTOM;
        }
        else if(gamepad1.dpad_right) {
            moveLift(LIFT_SPEED, currentHeight, CurrentHeight.ABOVE_ONE);
            currentHeight = CurrentHeight.ABOVE_ONE;
        }
        else if(gamepad1.dpad_up) {
            moveLift(LIFT_SPEED, currentHeight, CurrentHeight.ABOVE_TWO);
            currentHeight = CurrentHeight.ABOVE_TWO;
        }
        ///*
        if(gamepad1.x) {
            moveLift(LIFT_SPEED, 1);
        }
        else if(gamepad1.y) {
            moveLift(LIFT_SPEED, -1);
        }
        else if(gamepad1.dpad_left) {
            moveLift(LIFT_SPEED, -5);
        }
        //*/
    }

    @Override
    public void stop() {
    }
}
