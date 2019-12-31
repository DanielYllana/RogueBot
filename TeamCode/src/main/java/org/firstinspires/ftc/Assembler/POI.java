package org.firstinspires.ftc.Assembler;

import java.util.ArrayList;

import static org.firstinspires.ftc.RobotUtilities.MovementVars.finalRobotLocationX;
import static org.firstinspires.ftc.RobotUtilities.MovementVars.robotLocationY;
import static org.firstinspires.ftc.Assembler.RobotMovement.followCurve;



public class POI {
    public static double robotInitialX;
    public static double robotInitialY;
    public static double robotInitialAngle;

    public static void squareToTriangleRed(){
        robotInitialAngle = Math.toRadians(0);
        robotInitialX = 50;
        robotInitialY = 50;


        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(80,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(100,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(270,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(finalRobotLocationX = 320,robotLocationY = 320,1.0,1.0,50, Math.toRadians(0), 1.0));

        //followCurve(allPoints, Math.toRadians(90));

    }

    public static void squareToTriangleBlue(){

        robotInitialAngle = Math.toRadians(180);
        robotInitialX = 250;
        robotInitialY = 50;


        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(350,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(200,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        //allPoints.add(new CurvePoint(250,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(80,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(finalRobotLocationX = 30,robotLocationY = 350,1.0,1.0,50, Math.toRadians(0), 1.0));

        //followCurve(allPoints, Math.toRadians(90));

    }

}
