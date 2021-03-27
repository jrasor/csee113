/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

//import static org.firstinspires.ftc.robotcore.external
// .BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.Navigator
// .RingOrientationAnalysisPipeline.Stage.values;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all needed for an FTC robot to determine
 * its position on the Field.
 */

/* Version history
 * ======= =======
 * v 0.1    12/24/20 copied from Pullbot, then hollowed out to near stub
 *          status. It only uses Vuforia. Later versions may use OpenCV to
 *          detect known Field features, like Walls, floor markings, Ultimate
 *          Goal Towers.
 * v 0.2    12/28/20 hollowed out, then filled with Vuforia initialization
 *          code from Drive2XYHeading. That's the same code as used by
 *          Scrimmage3Teleop. Those two opmodes will eventually instantiate
 *          a Navigator. 12/29/20: used in Scrimmage3TeleOp and Drive2XYHeading.
 */

public class Navigator extends GenericFTCRobot {

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
  public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  // Todo: should this be Pullbot.BACK?
  public static final boolean PHONE_IS_PORTRAIT = false;

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
  public static final String VUFORIA_KEY =
      GenericFTCRobot.VUFORIA_KEY;

  // Since ImageTarget trackables use mm to specifiy their dimensions, we
  // must use mm for all the physical dimension.
  // We will define some constants and conversions here

  public static final float mmTargetHeight =
      (6) * GenericFTCRobot.mmPerInch;          //
  // the height of the center of the target image above the floor

  // Constants for perimeter targets
  private static final float halfField = 72 * GenericFTCRobot.mmPerInch;
  private static final float quadField = 36 * GenericFTCRobot.mmPerInch;

  // Class Members
  public OpenGLMatrix robotFromCamera;
  public OpenGLMatrix lastLocation = null;
  public VuforiaLocalizer vuforia = null;
  public boolean targetVisible = false;
  public float phoneXRotate = 0;
  public float phoneYRotate = 0;
  public float phoneZRotate = 0;

  // Initialization.
  HardwareMap hwMap = null;
  private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();

  /* Constructors */
  public Navigator() {
    super();
  }
  public Navigator(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public int cameraMonitorViewId;

  public VuforiaTrackables targetsUltimateGoal;
  public List<VuforiaTrackable> allTrackables;
  public String init(HardwareMap someHWMap) {
    String initializationReport = "Navigator initialization: ";
    hwMap = someHWMap;

    /*
     * Configure Vuforia by creating a Parameter object, and passing it to
     * the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC
     *  phone);
     * If no camera monitor is desired, use the parameter-less constructor
     * instead (commented out below).
     */
    cameraMonitorViewId =
        hwMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            hwMap.appContext.getPackageName());
    initializationReport += "Camera Id: " + cameraMonitorViewId + ". ";

    VuforiaLocalizer.Parameters parameters =
        new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer
    // .Parameters();

    parameters.vuforiaLicenseKey = GenericFTCRobot.VUFORIA_KEY;
    parameters.cameraDirection = CAMERA_CHOICE;

    // Make sure extended tracking is disabled for this example.
    parameters.useExtendedTracking = false;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Load the data sets for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsUltimateGoal =
        vuforia.loadTrackablesFromAsset("UltimateGoal");
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
    initializationReport += "Loaded " + targetsUltimateGoal.size() + " " +
        "targets" +
        ". ";
    // For convenience, gather together all the trackable objects in one
    // easily-iterable collection */
    allTrackables = new ArrayList<VuforiaTrackable>();
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
    final float CAMERA_FORWARD_DISPLACEMENT =
        4.0f * GenericFTCRobot.mmPerInch;   // eg:
    // Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT =
        8.0f * GenericFTCRobot.mmPerInch;   // eg:
    // Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the
    // robot's center line

    robotFromCamera = OpenGLMatrix
        .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
            CAMERA_VERTICAL_DISPLACEMENT)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
            phoneYRotate, phoneZRotate, phoneXRotate));

    /**  Let all the trackable listeners know where the phone is.  */
    for (VuforiaTrackable trackable : allTrackables) {
      ((VuforiaTrackableDefaultListener)
          trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }
    initializationReport += allTrackables.size() + " trackables got their " +
        "camera location and direction. ";
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
    return initializationReport;
    };

  //  Usage methods go here.

}


