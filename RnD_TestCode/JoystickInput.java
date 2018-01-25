package org.firstinspires.ftc.teamcode.OldCode;

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

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Joystick Input", group="Testing")
public class JoystickInput extends OpMode
{
    @Override
    public void init() {
        //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        String offCenter = "";
        
        if ( Math.abs(x) > .1 || Math.abs(y) > .1 ) {
            offCenter = "True";
        }
        else {
            offCenter = "False";
        }
        
        telemetry.addData("Joystick X", x);
        telemetry.addData("Joystick Y", y);
        telemetry.addData("Is Off Center", offCenter);
        
        double requestedAngle = Math.atan2(x, y);
        
        if (requestedAngle < 0) {
            requestedAngle += (Math.PI*2);
        }
        
        telemetry.addData("Sin Value", Math.sin(requestedAngle + (Math.PI/4)));
        telemetry.addData("Cos Value", Math.cos(requestedAngle + (Math.PI/4)));
        
        telemetry.addData("Joystick Angle", Math.toDegrees(requestedAngle));
    }

    @Override
    public void stop() {
    }
}
