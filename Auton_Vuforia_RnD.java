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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */


@Autonomous(name="Auton Testing", group ="Testing")

public class Auton_Vuforia_RnD extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    public DcMotor LeftLift = null;
    public DcMotor RightLift = null;

    public Servo grabServo = null;

    ModernRoboticsI2cGyro gyro = null;

    public static double PID4 = (Math.PI / 4);
    
    // {@link #vuforia} is the variable we will use to store our instance of the Vuforia
    // localization engine.
    
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        FR = hardwareMap.dcMotor.get("Front Right");
        FL = hardwareMap.dcMotor.get("Front Left");
        BR = hardwareMap.dcMotor.get("Back Right");
        BL = hardwareMap.dcMotor.get("Back Left");

        LeftLift = hardwareMap.dcMotor.get("Left Lift");
        RightLift = hardwareMap.dcMotor.get("Right Lift");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        LeftLift.setDirection(DcMotor.Direction.FORWARD);
        RightLift.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        LeftLift.setPower(0);
        RightLift.setPower(0);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabServo = hardwareMap.get(Servo.class, "Grab Servo");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");
        gyro.calibrate();
        
        // To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        // If no camera monitor is desired, use the parameterless constructor instead (commented out below).
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //
        // IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
        // 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
        // A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
        // web site at https://developer.vuforia.com/license-manager.
        //
        // Vuforia license keys are always 380 characters long, and look as if they contain mostly
        // random data. As an example, here is a example of a fragment of a valid key:
        //      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
        // Once you've obtained a license key, copy the string from the Vuforia web site
        // and paste it in to your code onthe next line, between the double quotes.
        
        parameters.vuforiaLicenseKey = "ARtNGjD/////AAAAGVBR9gWTrkcJrseNopR9ABl/yRMwzKubQQFemwI6iXdMUSGmrBbWXmyvhhDL46ltLECdhKrLEDv/3jseDqvPMEP5T+CRkbZFm53khLQ9V/hrtFbxWWGrx+bzQYsnNBVChWaLeI4A/Y8Pb8cV0csF7fcbOtIyDaPdWRg1EJ9JPxakQw29eaCfKjHcLggXnAUPQBSXHkVZKBxcM+YLMiNg0ePI7RkZyOiJHZDLlQH20LorVyEkb/n55xeoWws+fud+PQZwpDllNLRWLh3gYKm7V/laI6xyXGoQ6o0/TqFUV9yO6LMoHNreCSFsDtrVRrGGjwyUhxZa79sJwcmO5rZ+0wHgZMNWQafjl/o3Y/1+9BI8";
        
        // We also indicate which camera on the RC that we wish to use.
        // Here we chose the back (HiRes) camera (for greater range), but
        // for a competition robot, the front camera might be more convenient.
        
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        
        // Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
        // in this data set: all three of the VuMarks in the game were created from this one template,
        // but differ in their instance id information.
        // @see VuMarkInstanceId
        
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        relicTrackables.activate();

        //DO PRE VUFORIA STUFF HERE

        //TO HERE THEN GET THE IMAGE
        
        while (opModeIsActive()) {
            
            // See if any of the instances of {@link relicTemplate} are currently visible.
            // {@link RelicRecoveryVuMark} is an enum which can have the following values:
            // UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
            // UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
            
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            /*
            while(vuMark = RelicRecoveryVuMark.UNKNOWN) {
                //For now we will try waiting... this might become a slow move towards its expected location
            }
            
            double lastDistanceToWall = trans.get(0); //or 1 or 2... idk which yet
            double lastDistanceFromBlocks = trans.get(whatever);
            
            while (distanceToWall < DesiredDistance) {
                VectorF trans = pose.getTranslation();
                
                distanceToWall = trans.get(0); //Or Something
                
                if(lastDistanceFromBlocks != (or close enough to it) trans.get(whatever)) {
                    some value thing to do a thing
                }
                
                move(in the direction I want it to);
            }
            
            while (robot is not close enough to the scoring box) {
                move closer to the scoring box
                if(moving closer or farther from wall) {
                    fix it
                }
            }
            
            release cube to score
            
            back up a small distance so that you stay in the parking zone
            */
            
            /*
            Robot is looking at image
            Robot backs up to the required amount
            Drives towards the cryptobox while rotating towards the image
            
            if (VuMark == right) {
                Robot.moveX(-x);
            }
            else if (VuMark == center) {
                Robot.moveX(-x-6);
            }
            else {
                Robot.moveX(-x-12);
            }
            
            Robot.moveY(24, turn = 63.435 degrees);
            */
            
            //EXAMPLE
            
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                // Found an instance of the template. In the actual game, you will probably
                //loop until this condition occurs, then move on to act accordingly depending
                // on which VuMark was visible.
                telemetry.addData("VuMark", "%s visible", vuMark);

                // For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                // it is perhaps unlikely that you will actually need to act on this pose information, but
                // we illustrate it nevertheless, for completeness.
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                // We further illustrate how to decompose the pose into useful rotational and
                // translational components
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    
                    telemetry.addData("X Pos", tX);
                    telemetry.addData("Y Pos", tY);
                    telemetry.addData("Z Pos", tZ);
                    
                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            

            
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
