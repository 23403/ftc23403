package org.firstinspires.ftc.teamcode.auto.V6.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import xyz.nin1275.custom.PPMP;
import xyz.nin1275.custom.PPPoint;

@Config("5+0 Auto Paths")
public class FiveSpecAutoPaths {
    /***
     * POINTS
    ***/
    /* start pos */
    public static Pose startPos = new Pose(7.6, 63.9, Math.toRadians(0));
    /* line1 */
    public static PPPoint.beizerLine scoreSpecimen1Points = new PPPoint.beizerLine(
            34.3,
            70,
            0
    );
    /* line2 */
    public static PPPoint.beizerCurve moveToPushLoc1Points = new PPPoint.beizerCurve(
            57.3,
            28,
            -90,
            new PPMP(22.3, 64.77),
            new PPMP(22.54, 14.24),
            new PPMP(59, 48.2)
    );
    /* line2a */
    public static PPPoint.beizerLine pushBlock1Points = new PPPoint.beizerLine(
            23,
            28,
            -90
    );
    /* line3 */
    public static PPPoint.beizerCurve moveToPushLoc2Points = new PPPoint.beizerCurve(
            55.5,
            18.1,
            -90,
            new PPMP(63.8, 30.1)
    );
    /* line3a */
    public static PPPoint.beizerLine pushBlock2Points = new PPPoint.beizerLine(
            23,
            18.1,
            -90
    );
    /* line4 */
    public static PPPoint.beizerCurve moveToPushLoc3Points = new PPPoint.beizerCurve(
            55,
            12,
            -90,
            new PPMP(60.26, 20.64)
    );
    /* line4a */
    public static PPPoint.beizerLine pushBlock3Points = new PPPoint.beizerLine(
            23,
            10,
            -90
    );
    /* line5 */
    public static PPPoint.beizerCurve grabSpecimen1Points = new PPPoint.beizerCurve(
            8.3,
            29,
            -180,
            new PPMP(36.78, 11.87)
    );
    /* line6 */
    public static PPPoint.beizerLine scoreSpecimen2Points = new PPPoint.beizerLine(
            32.5,
            68,
            180
    );
    /* line7 */
    public static PPPoint.beizerLine grabSpecimen2Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            180
    );
    /* line8 */
    public static PPPoint.beizerLine scoreSpecimen3Points = new PPPoint.beizerLine(
            33.3,
            75.5,
            180
    );
    /* line9 */
    public static PPPoint.beizerLine grabSpecimen3Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            180
    );
    /* line10 */
    public static PPPoint.beizerLine scoreSpecimen4Points = new PPPoint.beizerLine(
            34,
            73,
            180
    );
    /* line11 */
    public static PPPoint.beizerLine grabSpecimen4Points = new PPPoint.beizerLine(
            8.9,
            29.7,
            180
    );
    /* line12 */
    public static PPPoint.beizerLine scoreSpecimen5Points = new PPPoint.beizerLine(
            35.3,
            74,
            180
    );
    /* line13 */
    public static PPPoint.beizerLine parkPoints = new PPPoint.beizerLine(
            21.3,
            50.6,
            -125
    );
    /***
     * PATHS
    ***/
    /* line1 */
    public static PathChain scoreSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(startPos),
                        scoreSpecimen1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(4.5)
                .setPathEndTimeoutConstraint(5)
                .build();
    }

    /* line2 */
    public static PathChain pushBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getMiddlePoint(2),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(6)
                .setPathEndTimeoutConstraint(50)
                .build();
    }

    /* line3 */
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
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }

    /* line4 */
    public static PathChain pushBlock3() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        pushBlock3Points.getEndPoint(),
                        moveToPushLoc3Points.getMiddlePoint(),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc3Points.getEndPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(pushBlock3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }

    /* line5 */
    public static PathChain grabSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        pushBlock3Points.getEndPoint(),
                        grabSpecimen1Points.getMiddlePoint(),
                        grabSpecimen1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock3Points.getEndHeading()), Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line6 */
    public static PathChain scoreSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen2Points.getEndPoint(),
                        scoreSpecimen2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line7 */
    public static PathChain grabSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen2Points.getEndPoint(),
                        grabSpecimen2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line8 */
    public static PathChain scoreSpecimen3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen2Points.getEndPoint(),
                                scoreSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line9 */
    public static PathChain grabSpecimen3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen3Points.getEndPoint(),
                                grabSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line10 */
    public static PathChain scoreSpecimen4() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen3Points.getEndPoint(),
                                scoreSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line11 */
    public static PathChain grabSpecimen4() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen4Points.getEndPoint(),
                                grabSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line12 */
    public static PathChain scoreSpecimen5() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen4Points.getEndPoint(),
                                scoreSpecimen5Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen5Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line13 */
    public static PathChain park() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getEndPoint(),
                                parkPoints.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
}