package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import xyz.nin1275.custom.PPMP;
import xyz.nin1275.custom.PPPoint;

@Config("4+0 Auto Paths")
public class FourSpecimenAutoPaths {
    /* start pos */
    public  static final Pose startPos = new Pose(9, 63.4, Math.toRadians(0));
    /* line1 */
    public static PPPoint.beizerLine preloadPoints = new PPPoint.beizerLine(
            37.1,
            61.1,
            0
    );
    /* line2 */
    public static PPPoint.beizerLine grabSpecimen1Points = new PPPoint.beizerLine(
            21.65,
            35.8,
            0
    );
    /* line3 */
    public static PPPoint.beizerLine scoreSpecimen1Points = new PPPoint.beizerLine(
            37,
            69,
            0
    );
    /* line4a */
    public static PPPoint.beizerCurve moveToPushLoc1Points = new PPPoint.beizerCurve(
            54.74,
            28,
            0,
            new PPMP(27.5, 21.8),
            new PPMP(55.2, 45.1)
    );
    /* line4b */
    public static PPPoint.beizerLine pushBlock1Points = new PPPoint.beizerLine(
            17.6,
            28,
            0
    );
    /* line4c */
    public static PPPoint.beizerCurve moveToPushLoc2Points = new PPPoint.beizerCurve(
            53.94,
            20.3,
            0,
            new PPMP(58.74, 31)
    );
    /* line4d */
    public static PPPoint.beizerLine pushBlock2Points = new PPPoint.beizerLine(
            17.6,
            20.3,
            0
    );
    /* line4e */
    public static PPPoint.beizerCurve moveToPushLoc3Points = new PPPoint.beizerCurve(
            53.9,
            13.3,
            0,
            new PPMP(53.49, 22.5)
    );
    /* line4f */
    public static PPPoint.beizerLine pushBlock3Points = new PPPoint.beizerLine(
            17.3,
            11.8,
            0
    );
    /* line10 */
    public static PPPoint.beizerLine scoreSpecimen2Points = new PPPoint.beizerLine(
            34,
            68,
            0
    );
    /* line11 */
    public static PPPoint.beizerLine grabSpecimen2Points = new PPPoint.beizerLine(
            18.5,
            35.8,
            0
    );
    /* line12 */
    public static PPPoint.beizerLine scoreSpecimen3Points = new PPPoint.beizerLine(
            33,
            67,
            0
    );
    /* line13 */
    public static PPPoint.beizerCurve parkPoints = new PPPoint.beizerCurve(
            16.8,
            49.23,
            -130,
            new PPMP(27.2, 69.3)
    );

    /* line1 */
    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                new Point(startPos),
                                preloadPoints.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(preloadPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line2 */
    public static PathChain grabSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                preloadPoints.getEndPoint(),
                                grabSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line3 */
    public static PathChain scoreSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen1Points.getEndPoint(),
                                scoreSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line4a */
    public static PathChain pushBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
    }
    /* line4b */
    public static PathChain pushBlock2() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        pushBlock1Points.getEndPoint(),
                        moveToPushLoc2Points.getMiddlePoint(),
                        moveToPushLoc2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc2Points.getEndPoint(),
                        pushBlock2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(pushBlock2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line4c */
    public static PathChain pushBlock3() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        pushBlock2Points.getEndPoint(),
                        moveToPushLoc3Points.getMiddlePoint(),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc3Points.getEndPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(pushBlock3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();
    }
    /* line5 */
    public static PathChain scoreSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        pushBlock3Points.getEndPoint(),
                        scoreSpecimen2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line6 */
    public static PathChain grabSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen2Points.getEndPoint(),
                                grabSpecimen2Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line7 */
    public static PathChain scoreSpecimen3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen2Points.getEndPoint(),
                                scoreSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line8 */
    public static PathChain park() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen3Points.getEndPoint(),
                        parkPoints.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
}