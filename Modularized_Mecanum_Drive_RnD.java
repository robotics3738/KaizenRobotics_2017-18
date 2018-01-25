package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mechanum TeleOp", group="Drive Testing")
public class Modularized_Mecanum_Drive_RnD extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private MecanumRobot robot = new MecanumRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }
    
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
      
    @Override
    public void loop() {
        
        double move_x =  gamepad1.left_stick_x;
        double move_y = -gamepad1.left_stick_y;
        
        //double turn_x =  gamepad1.right_stick_x;
        //double turn_y = -gamepad1.right_stick_y;
        
        double move_speed = gamepad1.right_trigger;
        
        double robotHeading = Math.toRadians(360 - robot.gyro.getHeading());
                 
        if ( move_speed > 0 ) {
            double angleToMove = robot.getAngleRelativeToRobot(move_x, move_y, robotHeading);
            
            //if (Math.abs(turn_x) > 0.2 || Math.abs(turn_y) > 0.2) {
            //    double turnModifier = robot.calculateTurnModifier(turn_x, turn_y, robotHeading);
            //    robot.move(move_speed, angleToMove, turnModifier);
            //}
            //else {
            //    robot.move(move_speed, angleToMove);
            //}
            
            if (gamepad1.right_bumper) {
                robot.move(move_speed, angleToMove, .2);
            }
            else if(gamepad1.left_bumper) {
                robot.move(move_speed, angleToMove, -.2);
            }
            else {
                robot.move(move_speed, angleToMove);
            }
        }
        else if (gamepad1.right_bumper) {
            robot.turn(-0.2);
        }
        else if (gamepad1.left_bumper) {
            robot.turn(0.2);
        }
        else {
            robot.stopRobot();
        }
        
        if (!robot.LeftLift.isBusy()) {
            if(gamepad1.dpad_down) {
                robot.liftToLevel(LiftLevel.BOTTOM);
            }
            else if(gamepad1.dpad_right) {
                robot.liftToLevel(LiftLevel.ABOVE_ONE);
            }
            else if(gamepad1.dpad_up) {
                robot.liftToLevel(LiftLevel.ABOVE_TWO);
            }
            else if(gamepad1.dpad_left) {
                robot.liftToLevel(LiftLevel.ABOVE_GROUND);
            }
            else {
               robot.LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            
        }
        
        if(gamepad1.a) {
            robot.grabServo.setPosition(1);
        }
        else if(gamepad1.b) {
            robot.grabServo.setPosition(0);
        }
        else {
            robot.grabServo.setPosition(.5);
        }
    }
    
    @Override
    public void stop() {
        robot.stopRobot();
    }
}
