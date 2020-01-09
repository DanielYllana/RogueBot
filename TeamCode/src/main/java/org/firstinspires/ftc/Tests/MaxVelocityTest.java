
package org.firstinspires.ftc.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RogueBot;
import org.firstinspires.ftc.CV_Algorithm.skystoneDetectorClass;

import java.text.DecimalFormat;

@TeleOp

public class MaxVelocityTest extends LinearOpMode {

    DcMotorEx motor;

    double currentVelocity;

    double maxVelocity = 0.0;


    @Override

    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "CoreHex");

        waitForStart();


        while (opModeIsActive()) {

            currentVelocity = motor.getVelocity();



            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }



            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();

        }
    }
}