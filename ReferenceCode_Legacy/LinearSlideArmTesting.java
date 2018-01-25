package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
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

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SlideArmTesting", group="Testing")
public class LinearSlideArmTesting extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    public DcMotor LiftMotor = null;
    Servo LeftServo;
    Servo RightServo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        LiftMotor  = hardwareMap.dcMotor.get("Lift Motor");
        LeftServo  = hardwareMap.get(Servo.class, "Left Servo");
        RightServo = hardwareMap.get(Servo.class, "Right Servo");
        
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        LiftMotor.setPower(0);
        
        LeftServo.setPosition(.5);
        RightServo.setPosition(.5);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (gamepad1.a) {
            LeftServo.setPosition(0);
            RightServo.setPosition(1);
        }
        else if (gamepad1.b) {
            LeftServo.setPosition(1);
            RightServo.setPosition(0);
        }
        else {
            LeftServo.setPosition(.5);
            RightServo.setPosition(.5);
        }
        
        if (gamepad1.x) {
            LiftMotor.setPower(1);
        }
        else if(gamepad1.y) {
            LiftMotor.setPower(-1);
        }
        else {
            LiftMotor.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
