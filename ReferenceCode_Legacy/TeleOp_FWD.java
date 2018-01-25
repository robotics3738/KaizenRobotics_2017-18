package org.firstinspires.ftc.teamcode.ReferenceCode_Legacy;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp - Four Wheel Drive", group="TeleOp")
public class TeleOp_FWD extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    
    private DcMotor Lift = null;
    private DcMotor Lift2 = null;
    
    Servo LeftServo;
    Servo RightServo;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        FL  = hardwareMap.get(DcMotor.class, "Front Left");
        FR = hardwareMap.get(DcMotor.class, "Front Right");
        BL  = hardwareMap.get(DcMotor.class, "Back Left");
        BR = hardwareMap.get(DcMotor.class, "Back Right");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift 2");
        
        LeftServo  = hardwareMap.get(Servo.class, "Left Servo");
        RightServo = hardwareMap.get(Servo.class, "Right Servo");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
        
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        
        Lift.setPower(0);
        Lift2.setPower(0);
        
        LeftServo.setPosition(.5);
        RightServo.setPosition(.5);
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
     
     boolean flag = false;
     boolean flag2 = false;
     double divide = 1;
     boolean xflag = false;
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        /*
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        */
        
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        double rightPower  = -gamepad1.left_stick_y;
        double leftPower = -gamepad1.right_stick_y;

        // Send calculated power to wheels
        FL.setPower(leftPower/divide);
        FR.setPower(rightPower/divide);
        BL.setPower(leftPower/divide);
        BR.setPower(rightPower/divide);

        /*        
        if (gamepad1.right_bumper && !xflag) {
            if (flag2) {
                divide = 2;
                flag2 = false;
            }
            else {
                divide = 1;
                flag2 = true;
            }
            //flag2 = !flag2;
            xflag = true;
        }
        if (!gamepad1.right_bumper) {
            xflag = false;
        }
*/
        if(gamepad1.right_trigger != 0) {
            Lift.setPower(gamepad1.right_trigger);
            Lift2.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger != 0) {
            Lift.setPower(-gamepad1.left_trigger);
            Lift2.setPower(-gamepad1.left_trigger);
        }
        else {
            Lift.setPower(0);
            Lift2.setPower(0);
        }


        if (gamepad1.b) {
            LeftServo.setPosition(.65);
            RightServo.setPosition(.35);
            flag = false;
        }
        else if (gamepad1.a || flag) {
            LeftServo.setPosition(0);
            RightServo.setPosition(1);
            flag = true;
        }
        else {
            LeftServo.setPosition(.59);
            RightServo.setPosition(.43);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
