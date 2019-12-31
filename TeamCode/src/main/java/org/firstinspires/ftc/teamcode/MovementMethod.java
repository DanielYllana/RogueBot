



package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Locale;

//import static org.firstinspires.ftc.teamcode.RobotMovement.followCurve;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Autonomous(name="Movement Method", group="Pushbot")
@Disabled
public abstract class MovementMethod extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();


    private static final double COUNTS_PER_MOTOR_REV = 134;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.3;
    private static final double TURN_SPEED = 0.3;
    private static final double Lift_Speed = 0.4;

    int leftFrontCurrentPosition = 0;
    int rightFrontCurrentPositon = 0;
    int leftBackCurrentPosition = 0;
    int rightBackCurrentPosition = 0;



    public void encoderMovement(double cm, double angleHeading,
                                double timeoutS, RogueBot robot) {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double angleHeadingDegrees = angleHeading;
        angleHeading = Math.toRadians(angleHeading);


        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double distance;
        double frontLeftDistance = 0;
        double backLeftDistance = 0;
        double frontRightDistance = 0;
        double backRightDistance = 0;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double xDist;
        double yDist;
        distance = cm/0.52;

        DecimalFormat rounding = new DecimalFormat("#.###");
        rounding.setRoundingMode(RoundingMode.CEILING);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            xDist = distance * Math.cos(angleHeading);
            yDist = distance * Math.sin(angleHeading);

            if (Math.abs(yDist) >= Math.abs(xDist)) {
                if (angleHeadingDegrees > 90 && angleHeadingDegrees < 180) {
                    frontLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    frontRightDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                    backLeftDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                    backRightDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                } else if (angleHeadingDegrees > 180) {
                    frontLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist) * -1;
                    frontRightDistance = Math.sqrt(yDist * yDist + xDist * xDist) * -1;
                    backLeftDistance = Math.sqrt(yDist * yDist + xDist * xDist) * -1;
                    backRightDistance = Math.sqrt(yDist * yDist - xDist * xDist) * -1;
                } else {
                    frontLeftDistance = Math.sqrt((yDist * yDist) + (xDist * xDist));
                    frontRightDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    backLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    backRightDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                }
            } else if (Math.abs(xDist) > Math.abs(yDist)) {
                if (angleHeadingDegrees > 90 && angleHeadingDegrees <= 180) {
                    frontLeftDistance = Math.sqrt(xDist * xDist + yDist * yDist)*-1;
                    frontRightDistance = Math.sqrt(xDist * xDist - yDist * yDist);
                    backLeftDistance = Math.sqrt(xDist * xDist - yDist * yDist);
                    backRightDistance = Math.sqrt(xDist * xDist + yDist * yDist)*-1;
                } else if (angleHeadingDegrees < 90) {
                    frontLeftDistance = Math.sqrt(xDist * xDist + yDist * yDist);
                    frontRightDistance = Math.sqrt(xDist * xDist - yDist * yDist) * -1;
                    backLeftDistance = Math.sqrt(xDist * xDist - yDist * yDist) * -1;
                    backRightDistance = Math.sqrt(xDist * xDist + yDist * yDist);
                }

            }

            frontLeftPower = Double.parseDouble(rounding.format((frontLeftDistance / distance)));
            frontRightPower = Double.parseDouble(rounding.format((frontRightDistance / distance)));
            backLeftPower = Double.parseDouble(rounding.format((backLeftDistance / distance)));
            backRightPower = Double.parseDouble(rounding.format((backRightDistance / distance)));


            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (backRightDistance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (frontRightDistance * COUNTS_PER_INCH);
            newLeftBackTarget = (int) (backLeftDistance * COUNTS_PER_INCH);
            newRightBackTarget = (int) (backRightDistance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(frontLeftPower*0.4);
            robot.rightFrontMotor.setPower(frontRightPower*0.4);
            robot.leftBackMotor.setPower(backLeftPower*0.4);
            robot.rightBackMotor.setPower(backRightPower*0.4);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.leftFrontMotor.isBusy() || robot.rightFrontMotor.isBusy() || robot.leftBackMotor.isBusy() || robot.rightBackMotor.isBusy()) &&  (runtime.seconds() < timeoutS)) {
                telemetry.addData("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                RobotLog.d("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                RobotLog.d("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                RobotLog.d("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.update();
            }

            leftFrontCurrentPosition = newLeftFrontTarget;
            leftBackCurrentPosition = newLeftBackTarget;
            rightBackCurrentPosition = newRightBackTarget;
            rightFrontCurrentPositon = newRightFrontTarget;

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Power of left front", "FLP (%.2f", robot.leftFrontMotor.getPower());
            telemetry.addData("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
            telemetry.addData("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.update();
            sleep(500);   // optional pause after each move
        }
    }


    // Overload method to move at a certain speed given as a parameter
    public void encoderMovement(double cm, double angleHeading, double speedMultiplier,
                                double timeoutS, RogueBot robot) {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double angleHeadingDegrees = angleHeading;
        angleHeading = Math.toRadians(angleHeading);


        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double distance;
        double frontLeftDistance = 0;
        double backLeftDistance = 0;
        double frontRightDistance = 0;
        double backRightDistance = 0;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double xDist;

        double yDist;
        distance = cm/0.52;

        DecimalFormat rounding = new DecimalFormat("#.###");
        //rounding.setRoundingMode(RoundingMode.CEILING);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            xDist = distance * Math.cos(angleHeading);
            yDist = distance * Math.sin(angleHeading);

            if (Math.abs(yDist) >= Math.abs(xDist)) {
                if (angleHeadingDegrees > 90 && angleHeadingDegrees < 180) {
                    frontLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    frontRightDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                    backLeftDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                    backRightDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                } else if (angleHeadingDegrees > 180) {
                    frontLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist) * -1;
                    frontRightDistance = Math.sqrt(yDist * yDist + xDist * xDist) * -1;
                    backLeftDistance = Math.sqrt(yDist * yDist + xDist * xDist) * -1;
                    backRightDistance = Math.sqrt(yDist * yDist - xDist * xDist) * -1;
                } else {
                    frontLeftDistance = Math.sqrt((yDist * yDist) + (xDist * xDist));
                    frontRightDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    backLeftDistance = Math.sqrt(yDist * yDist - xDist * xDist);
                    backRightDistance = Math.sqrt(yDist * yDist + xDist * xDist);
                }
            } else if (Math.abs(xDist) > Math.abs(yDist)) {
                if (angleHeadingDegrees > 90 && angleHeadingDegrees <= 180) {
                    frontLeftDistance = Math.sqrt(xDist * xDist + yDist * yDist)*-1;
                    frontRightDistance = Math.sqrt(xDist * xDist - yDist * yDist);
                    backLeftDistance = Math.sqrt(xDist * xDist - yDist * yDist);
                    backRightDistance = Math.sqrt(xDist * xDist + yDist * yDist)*-1;
                } else if (angleHeadingDegrees < 90) {
                    frontLeftDistance = Math.sqrt(xDist * xDist + yDist * yDist);
                    frontRightDistance = Math.sqrt(xDist * xDist - yDist * yDist) * -1;
                    backLeftDistance = Math.sqrt(xDist * xDist - yDist * yDist) * -1;
                    backRightDistance = Math.sqrt(xDist * xDist + yDist * yDist);
                }

            }

            frontLeftPower = Double.parseDouble(rounding.format((frontLeftDistance / distance)));
            frontRightPower = Double.parseDouble(rounding.format((frontRightDistance / distance)));
            backLeftPower = Double.parseDouble(rounding.format((backLeftDistance / distance)));
            backRightPower = Double.parseDouble(rounding.format((backRightDistance / distance)));


            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (backRightDistance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (frontRightDistance * COUNTS_PER_INCH);
            newLeftBackTarget = (int) (backLeftDistance * COUNTS_PER_INCH);
            newRightBackTarget = (int) (backRightDistance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(frontLeftPower*speedMultiplier);
            robot.rightFrontMotor.setPower(frontRightPower*speedMultiplier);
            robot.leftBackMotor.setPower(backLeftPower*speedMultiplier);
            robot.rightBackMotor.setPower(backRightPower*speedMultiplier);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.leftFrontMotor.isBusy() || robot.rightFrontMotor.isBusy() || robot.leftBackMotor.isBusy() || robot.rightBackMotor.isBusy()) &&  (runtime.seconds() < timeoutS)) {


                telemetry.addData("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                RobotLog.d("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                RobotLog.d("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                RobotLog.d("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
                telemetry.update();

                if(newLeftFrontTarget-50<robot.leftFrontMotor.getCurrentPosition() || robot.leftFrontMotor.getCurrentPosition()>newLeftFrontTarget+50){
                    break;
                }
            }

            leftFrontCurrentPosition = newLeftFrontTarget;
            leftBackCurrentPosition = newLeftBackTarget;
            rightBackCurrentPosition = newRightBackTarget;
            rightFrontCurrentPositon = newRightFrontTarget;

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Power of left front", "FLP (%.2f", robot.leftFrontMotor.getPower());
            telemetry.addData("Motor Power, Motor Distance ", "FL (%.2f), FR (%.2f), FLD (%d), FRD (%d)", robot.leftFrontMotor.getPower(), robot.rightFrontMotor.getPower(), robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("Motor Power ", "BLP (%.2f), BRP (%.2f), BLD (%d), BRD (%d)", robot.leftBackMotor.getPower(), robot.rightBackMotor.getPower(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
            telemetry.addData("Motor Power ", "BLT (%d), BRT (%d), FLT (%d), FRT (%d)", newLeftBackTarget, newRightBackTarget, newLeftFrontTarget, newRightFrontTarget);
            telemetry.update();
            sleep(500);   // optional pause after each move
        }
    }


    public void closeIntake(RogueBot robot){
        telemetry.addData("CLose", "INtake");
        telemetry.update();
        sleep(5000);
        robot.intakeServo.setPosition(0.4);
        sleep(100);
    }

    public void hookDown(RogueBot robot){

        robot.hookServo_1.setPower(1);//up
        robot.hookServo_2.setPower(-1);//up

        sleep(1000);
        telemetry.addData("down",' ');
        telemetry.update();
    }

    public void hookUp(RogueBot robot){
        robot.hookServo_1.setPower(-1);//up
        robot.hookServo_2.setPower(1);//up

        telemetry.addData("up",' ');
        telemetry.update();
        sleep(1000);

        robot.hookServo_1.setPower(0);//up
        robot.hookServo_2.setPower(0);//up
        telemetry.addData("up",' ');
        telemetry.update();
    }

    public void rotateServo(RogueBot robot){
        robot.rotateServo.setPosition(0.25);
        telemetry.addData("rotate","");
        sleep(1000);
    }

    public void turn(float degrees, RogueBot robot){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float degreesMoved = 0;
        float direction = Math.signum(degrees);
        boolean done = false;
        float lastAngle = angles.firstAngle;
        while(opModeIsActive() && !done){

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("first angle", angles.firstAngle);
            telemetry.addData("target", degrees);
            telemetry.addData("Done, fuck meeee i wannna dieeeee", done);
            telemetry.update();

            robot.leftFrontMotor.setPower(direction*TURN_SPEED);
            robot.rightFrontMotor.setPower(-1*direction*TURN_SPEED);
            robot.leftBackMotor.setPower(direction*TURN_SPEED);
            robot.rightBackMotor.setPower(-1*direction*TURN_SPEED);

            float currentAngle = angles.firstAngle;
            if(currentAngle - lastAngle > 90){
                currentAngle -= 360;
            }else if(currentAngle - lastAngle < -90){
                currentAngle+= 360;
            }
            degreesMoved += currentAngle - lastAngle;
            if(direction<0){
                done = degreesMoved < degrees;

            }else if(direction > 0){
                done = degreesMoved > degrees;
            }


            if(Math.abs(currentAngle) >=  Math.abs(degrees)){
                done = true;

            }

            lastAngle = currentAngle;

        }

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }

    public void encoderLift(double height, double timeoutS, RogueBot robot){
        int newLiftTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Reset Encoders
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(height * COUNTS_PER_INCH);

            // Assign new raget position
            robot.liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(Lift_Speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

}
