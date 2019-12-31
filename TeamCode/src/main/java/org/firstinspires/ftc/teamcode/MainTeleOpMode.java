package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainTeleOpMode", group = "TeleOp")

public class MainTeleOpMode extends OpMode{


    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double frontLeftPower,backLeftPower, frontRightPower, backRightPower;
    double drive;   // Power for forward and back motion
    double strafe;  // Power for left and right motion
    double rotate;  // Power for rotating the robot
    double startingAngle = 0;
    double currentAngle = 0;

    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        (WHEEL_DIAMETER_INCHES * 3.1415);


    double speedMultiplier = 0.6;

    double liftSpeed = 1.0;

    int maxLiftHeight = 3000;



    private RogueBot robot;

    @Override
    public void init() {

        robot = new RogueBot();
        robot.init(hardwareMap);

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//

    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }
    public void Init() {

    }



    public void loop() {
        //Finding the value of each joystick
        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        // Joystick Deadband
        if (Math.abs(drive) < JOYSTICK_DEADBAND){ drive = 0;}
        if (Math.abs(strafe) < JOYSTICK_DEADBAND){ strafe = 0;}
        if (Math.abs(strafe) < JOYSTICK_DEADBAND){ strafe = 0;}


        //Finding the power to assign for each motor
        backRightPower = drive + strafe - rotate;
        backLeftPower = drive - strafe + rotate;
        frontRightPower = drive - strafe - rotate;
        frontLeftPower = drive + strafe + rotate;


        // Setting the power to each motor
        robot.leftFrontMotor.setPower(frontLeftPower*speedMultiplier);
        robot.leftBackMotor.setPower(backLeftPower*speedMultiplier);
        robot.rightFrontMotor.setPower(frontRightPower*speedMultiplier);
        robot.rightBackMotor.setPower(backRightPower*speedMultiplier);




        // Setting the power to each motor
        robot.leftFrontMotor.setPower(frontLeftPower*speedMultiplier);
        robot.leftBackMotor.setPower(backLeftPower*speedMultiplier);
        robot.rightFrontMotor.setPower(frontRightPower*speedMultiplier);
        robot.rightBackMotor.setPower(backRightPower*speedMultiplier);



        robot.liftMotor.setPower(gamepad2.left_stick_y*liftSpeed);
        if(gamepad2.dpad_down && robot.liftMotor.getCurrentPosition()>0){
            telemetry.addData("Down","nothing");
            robot.liftMotor.setPower(liftSpeed*-1);
        }else if(gamepad2.dpad_up&& robot.liftMotor.getCurrentPosition()<5000){
            telemetry.addData("Up","nothing");
            robot.liftMotor.setPower(liftSpeed);
        }else{
            robot.liftMotor.setPower(0);
        }
//        if(robot.liftMotorRight.getCurrentPosition()<maxLiftHeight){
//            robot.liftMotorRight.setPower(-gamepad2.left_stick_y * liftSpeed);
//            robot.liftMotorLeft.setPower(gamepad2.left_stick_y * liftSpeed);
//        }else{
//            robot.liftMotorRight.setPower(0);
//            robot.liftMotorLeft.setPower(0);
//        }


        // Lift down
//        if(gamepad2.dpad_down){
//            robot.liftMotorRight.setPower(-0.5);
//            robot.liftMotorLeft.setPower(0.5);
//        } else if(gamepad2.dpad_up){ // Lift up
//            robot.liftMotorRight.setPower(0.5);
//            robot.liftMotorLeft.setPower(-0.5);
//        }else{ // Stop lift
//            robot.liftMotorRight.setPower(0);
//            robot.liftMotorLeft.setPower(0);
//        }




        // Open intake
        if(gamepad2.a){
            robot.intakeServo.setPosition(1);
        } else if(gamepad2.b){ // Close intake
            robot.intakeServo.setPosition(0.4);
        }

        if(gamepad2.y){ // Rotate intake outwards
            robot.rotateServo.setPosition(0);
        }else if(gamepad2.x){ // Rotate intake inwards
            robot.rotateServo.setPosition(0.71);
        }




        // Change the speedMultiplier for fast mode and reverse
        // Reverse mode is in testing and might not work
        // Reverse mode will switch the front of the robot to be the back
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            speedMultiplier = -1;
        }else if(gamepad1.left_bumper){
            speedMultiplier = 1;
        }else if (gamepad1.right_bumper){
            speedMultiplier = -0.6;
        }else{
            speedMultiplier = 0.6;
        }





        //Telemetry is not used to control the robot, it is purely to help debug by showing
        //Information on the phone
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Value of joystick = ", "Drive (%.2f), Strafe (%.2f)", gamepad2.right_stick_y, gamepad2.right_stick_x);

        // Display all motor power
        telemetry.addData("Motor Power ", "FL (%.2f), FR (%.2f)", robot.leftFrontMotor.getPower() , robot.rightFrontMotor.getPower());
        telemetry.addData("Motor Power ", "BL (%.2f), BR (%.2f)" , robot.leftBackMotor.getPower(), robot.leftBackMotor.getPower());


        telemetry.addData("Lift Pos", "Nothing (%d)", robot.liftMotor.getCurrentPosition());
        telemetry.update();

    }

}