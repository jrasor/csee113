/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below)
 * provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 *  endorse or
 * promote products derived from this software without specific prior written
 *  permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 *  THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia
 * localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and
 * orientation of the
 * image relative to the camera.  This sample code then combines that
 * information with a
 * knowledge of where the target images are on the field, to determine the
 * location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right
 * and the
 * Blue Alliance Station is on the left.
 * <p>
 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance,
 * Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of
 * each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to
 * determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own
 * Vuforia license key as
 * is explained below.
 */


@TeleOp(name = "Drive2XYHeading", group = "Concept")
//@Disabled
public class Drive2XYHeading extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();

  /*                     = = Vuforia initialization. = =                  */
  // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the
  // orientation, based on how your phone is mounted:
  // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT
  // (selfie side)
  // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait)
  // or PHONE_IS_PORTRAIT = false (landscape)
  //
  // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you
  // must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
  //
  private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  private static final boolean PHONE_IS_PORTRAIT = false;

  /*
   * IMPORTANT: You need to obtain your own license key to use Vuforia. The
   * string below with which
   * 'parameters.vuforiaLicenseKey' is initialized is for illustration only,
   * and will not function.
   * A Vuforia 'Development' license key, can be obtained free of charge from
   *  the Vuforia developer
   * web site at https://developer.vuforia.com/license-manager.
   *
   * Vuforia license keys are always 380 characters long, and look as if they
   *  contain mostly
   * random data. As an example, here is a example of a fragment of a valid key:
   *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
   * Once you've obtained a license key, copy the string from the Vuforia web
   *  site
   * and paste it in to your code on the next line, between the double quotes.
   */
  private static final String VUFORIA_KEY =
      GenericFTCRobot.VUFORIA_KEY;

  // Since ImageTarget trackables use mm to specifiy their dimensions, we
  // must use mm for all the physical dimension.
  // We will define some constants and conversions here
  private static final float mmPerInch = 25.4f;
  private static final float mmTargetHeight = (6) * mmPerInch;          //
    // the height of the center of the target image above the floor

  // Constants for perimeter targets
  private static final float halfField = 72 * mmPerInch;
  private static final float quadField = 36 * mmPerInch;

  // Class Members
  private OpenGLMatrix lastLocation = null;
  private VuforiaLocalizer vuforia = null;
  private boolean targetVisible = false;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;

  double yCorrection = -0.04;
  double headingCorrection = 0.5;
  static final double STRAIGHT_SPEED = 0.6;
  static final double TURN_SPEED = 0.2;
  static final double MAX_CORRECTION = TURN_SPEED;
  static final double MIN_CORRECTION = -TURN_SPEED;
  double targetX = 74.0; // Aim for robot front to end up near the picture.
  double currentX = 0;  // We'll refine this by Vuforia if target image is
  // visible.
  double errorX = currentX - targetX;
  static final double ERROR_X_TOLERANCE = 16.0;
  // avoids large and unstable bearing changes on final approach.
  double targetY = 35.5; // Also so robot can be near the picture.
  double currentY;
  double errorY;
  static final double ERROR_Y_TOLERANCE = 1.0;
  double targetHeadingDegrees = 0.0;
  double targetHeadingRadians = targetHeadingDegrees * Math.PI / 180.0;
  double currentHeadingDegrees;
  double currentHeadingRadians;
  double errorHeadingDegrees;
  double errorHeadingRadians;

  double targetBearingRadians = 0.0;
  double targetBearingDegrees = targetBearingRadians * 180 / Math.PI;
  double currentBearingRadians;
  double errorBearingRadians;
  double currentBearingDegrees;
  double errorBearingDegrees;

  double correction = 0.0;

  @Override
  public void runOpMode() {
    while (!gamepad1.y) {
      telemetry.addLine("Tuning controls. Press yellow Y button to " +
          "finish tuning, even if no tuning done.");
      telemetry.addLine("  Pad left/right: more/less Y error correction" +
          ".");
      telemetry.addLine("  Green B button: more heading correction.");
      telemetry.addLine("  Blue X button: less heading correction.");
      telemetry.addLine("Press the Yellow Y button to finish tuning.");
      if (gamepad1.dpad_right) {
        gamepad1.dpad_right = false;
        yCorrection *= 1.1;
        sleep(300);
      }
      if (gamepad1.dpad_left) {
        gamepad1.dpad_left = false;
        yCorrection /= 1.1;
        sleep(300);
      }
      if (gamepad1.b) {
        gamepad1.b = false;
        headingCorrection *= 1.1;
        sleep(300);
      }
      if (gamepad1.x) {
        gamepad1.x = false;
        headingCorrection /= 1.1;
        sleep(300);
      }
      telemetry.addData("Bearing correction",
          " %5.3f  Heading correction %5.3f", yCorrection,
          headingCorrection);
      telemetry.update();
    }
    Pullbot robot = new Pullbot(this);
    String initReport = robot.init(hardwareMap);
    telemetry.addData("Robot status", "initialized.");
    telemetry.addData("Initialization report", initReport);
    telemetry.update();
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to
     * the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC
     *  phone);
     * If no camera monitor is desired, use the parameter-less constructor
     * instead (commented out below).
     */
    int cameraMonitorViewId =
        hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters =
        new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer
      // .Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CAMERA_CHOICE;

    // Make sure extended tracking is disabled for this example.
    parameters.useExtendedTracking = false;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Load the data sets for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsUltimateGoal =
        this.vuforia.loadTrackablesFromAsset("UltimateGoal");
    VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
    blueTowerGoalTarget.setName("Blue Tower Goal Target");
    VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
    redTowerGoalTarget.setName("Red Tower Goal Target");
    VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
    redAllianceTarget.setName("Red Alliance Target");
    VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
    blueAllianceTarget.setName("Blue Alliance Target");
    VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
    frontWallTarget.setName("Front Wall Target");

    // For convenience, gather together all the trackable objects in one
      // easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsUltimateGoal);

    /**
     * In order for localization to work, we need to tell the system where
     * each target is on the field, and
     * where the phone resides on the robot.  These specifications are in the
     * form of <em>transformation matrices.</em>
     * Transformation matrices are a central, important concept in the math
     * here involved in localization.
     * See
     * <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     * for detailed information. Commonly, you'll encounter transformation
     * matrices as instances
     * of the {@link OpenGLMatrix} class.
     *
     * If you are standing in the Red Alliance Station looking towards the
     * center of the field,
     *     - The X axis runs from your left to the right. (positive from the
     *     center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other
     *     side of the field
     *       where the Blue Alliance Station is. (Positive is from the
     *       center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.
     *     (Positive is above the floor)
     *
     * Before being transformed, each target image is conceptually located at
     * the origin of the field's
     *  coordinate system (the center of the field), facing up.
     */

    //Set the position of the perimeter targets with relation to origin
      // (center of field)
    redAllianceTarget.setLocation(OpenGLMatrix
        .translation(0, -halfField, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90
            , 0, 180)));

    blueAllianceTarget.setLocation(OpenGLMatrix
        .translation(0, halfField, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90
            , 0, 0)));
    frontWallTarget.setLocation(OpenGLMatrix
        .translation(-halfField, 0, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90
            , 0, 90)));

    // The tower goal targets are located a quarter field length from the
      // ends of the back perimeter wall.
    blueTowerGoalTarget.setLocation(OpenGLMatrix
        .translation(halfField, quadField, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90
            , 0, -90)));
    redTowerGoalTarget.setLocation(OpenGLMatrix
        .translation(halfField, -quadField, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90
            , 0, -90)));

    //
    // Create a transformation matrix describing where the phone is on the
      // robot.
    //
    // NOTE !!!!  It's very important that you turn OFF your phone's
      // Auto-Screen-Rotation option.
    // Lock it into Portrait for these numbers to work.
    //
    // Info:  The coordinate frame for the robot looks the same as the field.
    // The robot's "forward" direction is facing out along X axis, with the
      // LEFT side facing out along the Y axis.
    // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
    //
    // The phone starts out lying flat, with the screen facing Up and with
      // the physical top of the phone
    // pointing to the LEFT side of the Robot.
    // The two examples below assume that the camera is facing forward out
      // the front of the robot.

    // We need to rotate the camera around it's long axis to bring the
      // correct camera forward.
    if (CAMERA_CHOICE == BACK) {
      phoneYRotate = -90;
    } else {
      phoneYRotate = 90;
    }

    // Rotate the phone vertical about the X axis if it's in portrait mode
    if (PHONE_IS_PORTRAIT) {
      phoneXRotate = 90;
    }

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the
      // middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg:
      // Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg:
      // Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the
      // robot's center line

    OpenGLMatrix robotFromCamera = OpenGLMatrix
        .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
            CAMERA_VERTICAL_DISPLACEMENT)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
            phoneYRotate, phoneZRotate, phoneXRotate));

    /**  Let all the trackable listeners know where the phone is.  */
    for (VuforiaTrackable trackable : allTrackables) {
      ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    // WARNING:
    // In this sample, we do not wait for PLAY to be pressed.  Target
      // Tracking is started immediately when INIT is pressed.
    // This sequence is used to enable the new remote DS Camera Preview
      // feature to be used with this sample.
    // CONSEQUENTLY do not put any driving commands in this loop.
    // To restore the normal opmode structure, just un-comment the following
      // line:

    // waitForStart();

    // Note: To use the remote camera preview:
    // AFTER you hit Init on the Driver Station, use the "options menu" to
      // select "Camera Stream"
    // Tap the preview window to receive a fresh image.

    targetsUltimateGoal.activate();

    while (!isStopRequested()) {
      targetVisible = false;
      for (VuforiaTrackable trackable : allTrackables) {
        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
          telemetry.addData("Visible Target", trackable.getName());
          targetVisible = true;
          OpenGLMatrix robotLocationTransform =
              ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
          if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
          }
          break;
        }
      }

      // Report robot location and heading (if we know).
      if (targetVisible) {
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos ", "{X, Y} = %5.1f\", %5.1f\"",
            translation.get(0) / GenericFTCRobot.mmPerInch,
            translation.get(1) / GenericFTCRobot.mmPerInch);
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC,
                XYZ, DEGREES);
        telemetry.addData("Heading", "%4.0f\u00B0", rotation.thirdAngle);
      } else {
        telemetry.addData("Visible Target", "none");
      }
      telemetry.update();
      // Report where the robot is located, if we can see a Vuforia image.
      if (targetVisible && Math.abs(errorX) > ERROR_X_TOLERANCE) {

        // Report position (translation) and position error of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        currentX = translation.get(0) / GenericFTCRobot.mmPerInch;
        currentY = translation.get(1) / GenericFTCRobot.mmPerInch;
        errorX = currentX - targetX;
        errorY = currentY - targetY;
        telemetry.addData("Position error", "X, Y = %4.1f, %4.1f",
            errorX, errorY);

        // Report bearing and bearing error of target from robot.
        currentBearingRadians = Math.atan2(-errorY, -errorX);
        currentBearingDegrees = currentBearingRadians * 180.0 / Math.PI;
        errorBearingRadians = currentBearingRadians - targetBearingRadians;
        errorBearingDegrees = errorBearingRadians * 180.0 / Math.PI;
        telemetry.addData("Bearing", " %4.0f\u00B0  error: %4.0f\u00B0",
            currentBearingDegrees, errorBearingDegrees);

        // Report robot heading in degrees, and error of that heading.
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC,
                XYZ,
                DEGREES);
        currentHeadingDegrees = rotation.thirdAngle;
        currentHeadingRadians = currentHeadingDegrees * Math.PI / 180.0;
        errorHeadingDegrees = currentHeadingDegrees - targetHeadingDegrees;
        errorBearingRadians = currentBearingRadians - targetBearingRadians;
        errorHeadingRadians = errorHeadingDegrees * Math.PI / 180.0;
        telemetry.addData(
            "Heading", "%4.0f\u00B0  Heading error %4.0f\u00B0",
            currentHeadingDegrees, errorHeadingDegrees);

        // find motor speed corrections.
        correction =
            yCorrection * errorY - headingCorrection * errorHeadingRadians;
        correction = Math.min(Math.max(correction, MIN_CORRECTION),
            MAX_CORRECTION);
        // Todo: slow down when errorX gets small.

        //  Apply those corrections to drive motors.
        // This is a Pullbot, not a Pushbot.
        robot.leftDrive.setPower(-TURN_SPEED + correction);
        robot.rightDrive.setPower(-TURN_SPEED - correction);
        telemetry.addData("Motor speeds",
            "correction %5.3f left %5.3f right %5.3f",
            correction, TURN_SPEED - correction, TURN_SPEED + correction);

      } else if (!targetVisible) {
        telemetry.addLine("Visible target lost. Stopping.");
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        // Todo: try to recover from this by turning on axis, guessing
        //  from last known position.
      } else {
        telemetry.addLine("We have arrived. Stopping.");
        // Clean up residual heading error.
        robot.turnAngle(TURN_SPEED, -errorHeadingRadians);
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        // Disable tracking when we are done.
        targetsUltimateGoal.deactivate();
        stop();
      }
    }

    // Disable Tracking when we are done;
    targetsUltimateGoal.deactivate();
  }
}