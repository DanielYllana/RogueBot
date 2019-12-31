package org.firstinspires.ftc.com.company;

import org.firstinspires.ftc.RobotUtilities.MovementVars;
import org.firstinspires.ftc.RobotUtilities.SpeedOmeter;

import static org.firstinspires.ftc.Assembler.POI.*;


public class Robot {
    public static boolean usingComputer = true;

    /**
     * Creates a robot simulation
     */
    public Robot(){
        worldXPosition = 50;
        worldYPosition = 50;
        worldAngle_rad = Math.toRadians(180);

//        worldXPosition = robotInitialX;
//        worldYPosition = robotInitialY;
//        worldAngle_rad = robotInitialAngle;
    }

    //the actual speed the robot is moving
    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;
    double elapsedTime;

    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos(){
        return worldYPosition;
    }


    public double getWorldAngle_rad() {
        return worldAngle_rad;
    }


    public double getElapsedTime() {
        return elapsedTime;
    }
    //last update time
    private long lastUpdateTime = 0;

    /**
     * Calculates the change in position of the robot
     */
    public void update(){
        //get the current time
        long currentTimeMillis = System.currentTimeMillis();
        //get the elapsed time
        double elapsedTime = (currentTimeMillis - lastUpdateTime)/1000.0;
        //remember the lastUpdateTime
        lastUpdateTime = currentTimeMillis;
        if(elapsedTime > 1){return;}



        //increment the positions
        double totalSpeed = Math.hypot(xSpeed,ySpeed);
        double angle = Math.atan2(ySpeed,xSpeed) - Math.toRadians(90);
        double outputAngle = worldAngle_rad + angle;
        worldXPosition += totalSpeed * Math.cos(outputAngle) * elapsedTime * 1000 * 0.2;
        worldYPosition += totalSpeed * Math.sin(outputAngle) * elapsedTime * 1000 * 0.2;

        worldAngle_rad += MovementVars.movement_turn * elapsedTime * 20 / (2 * Math.PI);


        xSpeed += Range.clip((MovementVars.movement_x-xSpeed)/0.2,-1,1) * elapsedTime;
        ySpeed += Range.clip((MovementVars.movement_y-ySpeed)/0.2,-1,1) * elapsedTime;
        turnSpeed += Range.clip((MovementVars.movement_turn-turnSpeed)/0.2,-1,1) * elapsedTime;


//        SpeedOmeter.yDistTraveled += ySpeed * elapsedTime * 1000;
//        SpeedOmeter.xDistTraveled += xSpeed * elapsedTime * 1000;

//        SpeedOmeter.update();


        xSpeed *= 1.0 - (elapsedTime);
        ySpeed *= 1.0 - (elapsedTime);
        turnSpeed *= 1.0 - (elapsedTime);



    }
}
