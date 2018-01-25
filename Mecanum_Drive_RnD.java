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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mecanum Testing", group="Drive Testing")
public class Mecanum_Drive_RnD extends OpMode
{
    
    // MASSIVE CREDIT TO http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf
    
    
    // ADD WHILE(ABS(X) > 0.1 && ABS(Y) > 0.1)
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    public DcMotor LeftLift = null;
    public DcMotor RightLift = null;
    
    public Servo grabServo = null;

    ModernRoboticsI2cGyro gyro = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        FR = hardwareMap.dcMotor.get("Front Right");
        FL = hardwareMap.dcMotor.get("Front Left");
        BR = hardwareMap.dcMotor.get("Back Right");
        BL = hardwareMap.dcMotor.get("Back Left");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabServo = hardwareMap.get(Servo.class, "Grab Servo");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");
        gyro.calibrate();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
                
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double speed = gamepad1.right_trigger;
        
        double robotRotation = Math.toRadians(gyro.getHeading());
                 
        if ( speed > 0 ) {
            
            double aAngle = Math.atan2(x, y);
            
            if (aAngle < 0) {
                aAngle += (Math.PI*2);
            }
        
            //double roboAngle = aAngle - robotRotation;
            double roboAngle = aAngle;
            
            double FL_Power;
            double FR_Power;
            double BL_Power;
            double BR_Power;
            
            double PID4 = Math.PI / 4;
            
            FL_Power = speed*Math.sin(roboAngle + PID4);// + turnModifier;
            FR_Power = speed*Math.cos(roboAngle + PID4);// - turnModifier;
            BL_Power = speed*Math.cos(roboAngle + PID4);// + turnModifier;
            BR_Power = speed*Math.sin(roboAngle + PID4);// - turnModifier;
            
            /*
            double limit = 1.0;
            
            if (Math.abs(FL_Power) > limit) limit = Math.abs(FL_Power);
            if (Math.abs(FR_Power) > limit) limit = Math.abs(FR_Power);
            if (Math.abs(BL_Power) > limit) limit = Math.abs(BL_Power);
            if (Math.abs(BR_Power) > limit) limit = Math.abs(BR_Power);
            
            FL_Power /= limit;
            FR_Power /= limit;
            BL_Power /= limit;
            BR_Power /= limit;
            */
            
            FL.setPower(FL_Power);
            FR.setPower(FR_Power);
            BL.setPower(BL_Power);
            BR.setPower(BR_Power);
        
        telemetry.addData("FL", FL_Power);
        telemetry.addData("BL", BL_Power);
        
        telemetry.addData("Sin Value", Math.sin(aAngle + (Math.PI/4)));
        telemetry.addData("Cos Value", Math.cos(aAngle + (Math.PI/4)));
        
        telemetry.addData("Joystick Angle", Math.toDegrees(aAngle));
        }
        else if(gamepad1.y){
            FL.setPower(0.2);
            FR.setPower(0.2);
            BL.setPower(0.2);
            BR.setPower(0.2);
        }
        else if(gamepad1.b){
            FL.setPower(0.2);
            FR.setPower(-0.2);
            BL.setPower(-0.2);
            BR.setPower(0.2);
        }
        else {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
