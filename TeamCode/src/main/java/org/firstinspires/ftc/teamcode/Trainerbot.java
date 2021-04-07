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
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single
 * robot. In this case that robot is a Trainerbot, a front wheel drive
 * version of the Pushbot in the external samples. It is modified from the
 * Pullbot, inheriting most of that class.
 * <p>
 * The Trainerbot class assumes the following device names have been configured
 * on the robot:
 * Note:  All names are lower case.
 * <p>
 * Motor channel:  Left front  drive motor:        "motor0"
 * Motor channel:  Right front drive motor:        "motor1"
 * Servo channel:  Flexes the arm elbow:           "arm"
 * Color sensor:                                   "colorSensor"
 *
 */

/* Version history
 * ======= =======
 * v 0.1   4/6/21. Initial conversion from the Pullbot.
*/

public class Trainerbot extends Pullbot {

  // Arm related properties
  //public final double ARMSPEED = 0.5;
  public final double DEPLOYED = 0.7;   // arm extended in front of the Pullbot
  //public final int OVER_WALL = 1450;
  public final double STOWED = 0.0;     // arm retracted back over the Pullbot


  // Trainerbot specific sensor members.
  //public ColorSensor colorSensor;
  //public static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

  // Initialization.
  HardwareMap hwMap = null;
  //private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();
  private Servo arm;

  /* Constructors */

  public Trainerbot() {
    super();
  }
  public Trainerbot(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public String init(HardwareMap someHWMap) {
    hwMap = someHWMap;
    super.init(someHWMap);
    String initializationReport = "Trainerbot initialization: ";

    arm = hwMap.get(Servo.class, "arm");

    return initializationReport;
  }


  // Macros can go here. Most will be used in the opmodes.
}