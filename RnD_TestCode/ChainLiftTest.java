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

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ChainLiftTest", group="Testing")
public class ChainLiftTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    public DcMotor RightMotor = null;
    public DcMotor LeftMotor = null;

    Servo GrabServo;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        RightMotor = hardwareMap.dcMotor.get("Right Lift");
        LeftMotor = hardwareMap.dcMotor.get("Left Lift");
        
        GrabServo  = hardwareMap.get(Servo.class, "Grab Servo");

        RightMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        RightMotor.setPower(0);
        LeftMotor.setPower(0);

        //RightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //LeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            RightMotor.setTargetPosition(4000);
            LeftMotor.setTargetPosition(4000);
            
            RightMotor.setPower(0.65);
            LeftMotor.setPower(0.65);
        }
        else if (gamepad1.b) {
            RightMotor.setPower(-.65);
            LeftMotor.setPower(-.65);
        }
        else {
            RightMotor.setPower(0);
            LeftMotor.setPower(0);
        }
        /*
        if (gamepad1.x) {
            LeftServo.setPosition(0);
            RightServo.setPosition(1);
        }
        else if (gamepad1.y) {
            LeftServo.setPosition(1);
            RightServo.setPosition(0);
        }
        else {
            LeftServo.setPosition(.5);
            RightServo.setPosition(.5);
        } */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
