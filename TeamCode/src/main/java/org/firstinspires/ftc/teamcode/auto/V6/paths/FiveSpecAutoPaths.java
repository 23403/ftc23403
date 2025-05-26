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
    public static Pose startPos = new Pose(9, 63.4, Math.toRadians(0));
    /* line1 */
    public static PPPoint.beizerLine scoreSpecimen1Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line2a */
    public static PPPoint.beizerCurve returnBlockPoints = new PPPoint.beizerCurve(
            0,
            0,
            0,
            new PPMP(0, 0)
    );
    /* line2b */
    public static PPPoint.beizerCurve moveToPushLoc1Points = new PPPoint.beizerCurve(
            54.74,
            28,
            0,
            new PPMP(27.5, 21.8)
    );
    /* line2c */
    public static PPPoint.beizerLine pushBlock1Points = new PPPoint.beizerLine(
            19,
            28,
            0
    );
    /* line2d */
    public static PPPoint.beizerCurve moveToPushLoc2Points = new PPPoint.beizerCurve(
            53.94,
            20.3,
            0,
            new PPMP(58.74, 31)
    );
    /* line2e */
    public static PPPoint.beizerLine pushBlock2Points = new PPPoint.beizerLine(
            23.5,
            20.3,
            0
    );
    /* line2f */
    public static PPPoint.beizerCurve moveToPushLoc3Points = new PPPoint.beizerCurve(
            53.9,
            12.8,
            0,
            new PPMP(53.49, 22.5)
    );
    /* line2g */
    public static PPPoint.beizerLine pushBlock3Points = new PPPoint.beizerLine(
            15.3,
            12.8,
            0
    );
    /* line3 */
    public static PPPoint.beizerLine scoreSpecimen2Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line4 */
    public static PPPoint.beizerLine grabSpecimen3Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line5 */
    public static PPPoint.beizerLine scoreSpecimen3Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line6 */
    public static PPPoint.beizerLine grabSpecimen4Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line7 */
    public static PPPoint.beizerLine scoreSpecimen4Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line8 */
    public static PPPoint.beizerLine grabSpecimen5Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line9 */
    public static PPPoint.beizerLine scoreSpecimen5Points = new PPPoint.beizerLine(
            0,
            0,
            0
    );
    /* line10 */
    public static PPPoint.beizerLine parkPoints = new PPPoint.beizerLine(
            0,
            0,
            0
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
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line2a-b */
    public static PathChain returnAndPushBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        returnBlockPoints.getMiddlePoint(),
                        returnBlockPoints.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(returnBlockPoints.getEndHeading()))
                .addPath(new BezierCurve(
                        returnBlockPoints.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(returnBlockPoints.getEndHeading()), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line2b */
    public static PathChain pushBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line2c */
    public static PathChain pushBlock2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        pushBlock1Points.getEndPoint(),
                        moveToPushLoc2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc2Points.getEndPoint(),
                        pushBlock2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getEndHeading()), Math.toRadians(pushBlock2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line2d */
    public static PathChain pushBlock3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        pushBlock2Points.getEndPoint(),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc3Points.getEndPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getEndHeading()), Math.toRadians(pushBlock3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line3 */
    public static PathChain scoreSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                pushBlock3Points.getEndPoint(),
                                scoreSpecimen2Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line4 */
    public static PathChain grabSpecimen3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen2Points.getEndPoint(),
                                grabSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line5 */
    public static PathChain scoreSpecimen3() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen3Points.getEndPoint(),
                                scoreSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line6 */
    public static PathChain grabSpecimen4() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen3Points.getEndPoint(),
                                grabSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line7 */
    public static PathChain scoreSpecimen4() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen4Points.getEndPoint(),
                                scoreSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line8 */
    public static PathChain grabSpecimen5() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen4Points.getEndPoint(),
                                grabSpecimen5Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen5Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line9 */
    public static PathChain scoreSpecimen5() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen5Points.getEndPoint(),
                                scoreSpecimen5Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen5Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line10 */
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