package org.firstinspires.ftc.Assembler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.Assembler.CurvePoint;

import java.util.ArrayList;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.Assembler.RobotMovement.followCurve;
import static org.firstinspires.ftc.com.company.Robot.*;
import static org.firstinspires.ftc.RobotUtilities.MovementVars.*;
//import static main.java.org.firstinspires.ftc.teamcode.PointOfInterest.*;


public class MyOpMode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop(){



        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(80,50,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(100,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(270,150,1.0,1.0,50, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(finalRobotLocationX = 271,robotLocationY = 330,1.0,1.0,50, Math.toRadians(0), 1.0));

        //followCurve(allPoints, Math.toRadians(90));


    }
}
