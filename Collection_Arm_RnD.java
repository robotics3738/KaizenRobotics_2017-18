/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

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
