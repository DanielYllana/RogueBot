package org.firstinspires.ftc.Assembler;

import org.firstinspires.ftc.com.company.ComputerDebugging;
import org.firstinspires.ftc.com.company.FloatPoint;
import org.firstinspires.ftc.com.company.Range;
import org.firstinspires.ftc.org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.com.company.Robot.*;
import static org.firstinspires.ftc.Assembler.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.Assembler.MathFunctions.lineCircleIntersection;

public class RobotMovement {

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){

        //Sends some weird shit for the server
        for(int i = 0; i< allPoints.size() - 1; i++){
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y), new FloatPoint(allPoints.get(i+1).x, allPoints.get(i+1).y));
        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);


        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(pathPoints.size()-1));
        robotLocationX = robotLocation.x;
        robotLocationY = robotLocation.y;

        for(int i = 0; i< pathPoints.size() - 1; i++){
            CurvePoint starLine = pathPoints.get(i);
            CurvePoint endLIne = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, starLine.toPoint(), endLIne.toPoint());

//            while(intersections.size() ==0) {
//                followRadius = followRadius +10;
//                intersections = lineCircleIntersection(robotLocation, followRadius, starLine.toPoint(), endLIne.toPoint());
//            }

            double closerAngle = 100000000;


            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);


                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if (deltaAngle < closerAngle){
                    closerAngle = deltaAngle;
                    followMe.setPoi(thisIntersection);
                }
            }

        }
        return followMe;
    }



    /**
     *  @param x
     * @param y
     * @param movementSpeed
     * @param preferredAngle
     * @param turnSpeed
     * @return
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        //System.out.print("Distance to target= " + distanceToTarget);

        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        //System.out.println(" Angle to turn= " + relativeTurnAngle);

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10){
             movement_turn = 0;
        }

    }
}
