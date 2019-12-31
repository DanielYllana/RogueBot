package org.firstinspires.ftc.Main;

import org.firstinspires.ftc.com.company.ComputerDebugging;
import org.firstinspires.ftc.com.company.FloatPoint;
import org.firstinspires.ftc.com.company.Robot;
import org.firstinspires.ftc.com.company.Robot.*;
import org.firstinspires.ftc.com.company.UdpServer;
import org.firstinspires.ftc.Assembler.MyOpMode;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.Assembler.RobotMovement.*;
import org.firstinspires.ftc.Assembler.POI.*;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import static org.firstinspires.ftc.RobotUtilities.MovementVars.*;


public class Main {
    public static boolean running = true;
    public static boolean endAtTarget = false;

    public static void main(String[] args) {
        new Main().run();
    }

    /**
     * The program runs here
     */
    public void run(){


        //squareToTriangleBlue();

        //this is a test of the coding
        ComputerDebugging computerDebugging = new ComputerDebugging();
        Robot robot = new Robot();
        MyOpMode opMode = new MyOpMode();
        opMode.init();

        ComputerDebugging.clearLogPoints();


        long startTime = System.currentTimeMillis();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(running){

            //squareToTriangleBlue();
            opMode.loop();

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.update();
            ComputerDebugging.sendRobotLocation(robot);
            ComputerDebugging.sendLogPoint(new FloatPoint(Robot.worldXPosition,Robot.worldYPosition));
            ComputerDebugging.markEndOfUpdate();

            if(distanceToTarget < 10 && endAtTarget){
                break;
            }
            //System.out.println(robot.getElapsedTime());
        }
    }




}
