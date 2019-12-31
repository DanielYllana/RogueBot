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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RogueBot;
import org.firstinspires.ftc.CV_Algorithm.skystoneDetectorClass;

import java.text.DecimalFormat;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue Side Skystone", group="Pushbot")
//@Disabled
public class BlueSideSkystone extends MovementMethod {

    /* Declare OpMode members. */
    RogueBot robot = new RogueBot();   // Use a Pushbot's hardware

    skystoneDetectorClass detector = new skystoneDetectorClass();
    int[] vals;
    private ElapsedTime     runtime = new ElapsedTime();


    private static final double COUNTS_PER_MOTOR_REV = 134;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    int leftFrontCurrentPosition = 0;
    int rightFrontCurrentPositon = 0;
    int leftBackCurrentPosition = 0;
    int rightBackCurrentPosition = 0;

    boolean skystoneFound= false;


    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
       robot.init(hardwareMap);

        detector.setOffset(-0.8f/8f, 0.6f/8f);
        detector.camSetup(hardwareMap);

        robot.useEncoders(true);

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//
//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive() && !skystoneFound ) {
                detector.updateVals();
                vals = detector.getVals();
                telemetry.addData("Values", vals[1] + "   " + vals[0] + "   " + vals[2]);
                telemetry.addLine("hi");

                telemetry.update();

                if (vals[0] == 0) {//middle
                    telemetry.addLine("Position: Middle");
                    telemetry.update();
                    skystoneFound = true;
                    middleStone(0.2);


                } else if (vals[1] == 0) {//left
                    telemetry.addLine("Position: Left");
                    telemetry.update();
                    skystoneFound = true;
                    leftStone(0.4);

                } else {//right
                    telemetry.addLine("Position: Right");
                    telemetry.update();
                    skystoneFound = true;
                    rightStone(0.4);
                }


                telemetry.addData("Path", "Complete");
                telemetry.update();
            }
        }
    }

    public void leftStone(double speed){
        //Strafe to face the stone
        encoderMovement(50, 180, speed, 3, robot);
        //Move forward to the stone
        encoderMovement(76, 90, speed, 3, robot);
    }

    public void rightStone(double speed){
        //Strafe to face the stone
        encoderMovement(50, 0, speed, 3, robot);
        //Move forward to the stone
        encoderMovement(76, 90, speed, 3, robot);
    }

    public void middleStone(double speed){
        //Move forward to the stone
        encoderMovement(60, 90, speed, 3, robot);
        closeIntake(robot);
    }



    public void rotateAngle(double power, double angle, double threshold) {
        angle = -angle;
        if(angle < 0)
            power = -power;

        robot.useEncoders(false);

        robot.leftBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(-power);
        robot.leftFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(-power);

        double newAngle = robot.getAngle() + angle;
        boolean run = true;

        runtime.reset();
        while (opModeIsActive() && run) {
            if(Math.abs(robot.getAngle() - newAngle) < threshold)
                run = false;
            if(runtime.seconds() > 4)
                run = false;
        }

        // Step 4:  Stop and close the claw.
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        sleep(200);

        robot.useEncoders(true);

    }



}