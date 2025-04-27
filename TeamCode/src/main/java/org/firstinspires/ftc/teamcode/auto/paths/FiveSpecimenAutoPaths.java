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

@Config("5+0 Auto Paths")
public class FiveSpecimenAutoPaths {
    /* start pos */
    public static Pose startPos = new Pose(9, 63.4, Math.toRadians(-180));
    /* line1 */
    public static PPPoint.beizerLine scoreSpecimen1Points = new PPPoint.beizerLine(
            32,
            72.34,
            -180
    );
    /* line2a */
    public static PPPoint.beizerCurve moveToPushLoc1Points = new PPPoint.beizerCurve(
            40,
            43.8,
            309,
            new PPMP(21.67, 52.84)
    );
    /* line2b */
    public static PPPoint.beizerLine pushBlock1Points = new PPPoint.beizerLine(
            30.25,
            43.9,
            240
    );
    /* line2c */
    public static PPPoint.beizerLine moveToPushLoc2Points = new PPPoint.beizerLine(
            43.34,
            36,
            300
    );
    /* line2d */
    public static PPPoint.beizerLine pushBlock2Points = new PPPoint.beizerLine(
            33.1,
            29.48,
            224
    );
    /* line2e */
    public static PPPoint.beizerLine moveToPushLoc3Points = new PPPoint.beizerLine(
            44,
            29,
            295
    );
    /* line2f */
    public static PPPoint.beizerLine pushBlock3Points = new PPPoint.beizerLine(
            33.7,
            24,
            224
    );
    /* line3 */
    public static PPPoint.beizerLine grabSpecimen1Points = new PPPoint.beizerLine(
            26,
            36.74,
            360
    );
    /* line4 */
    public static PPPoint.beizerLine scoreSpecimen2Points = new PPPoint.beizerLine(
            37,
            69,
            360
    );
    /* line5 */
    public static PPPoint.beizerLine grabSpecimen2Points = new PPPoint.beizerLine(
            21.5,
            38.74,
            360
    );
    /* line6 */
    public static PPPoint.beizerLine scoreSpecimen3Points = new PPPoint.beizerLine(
            34,
            68,
            360
    );
    /* line7 */
    public static PPPoint.beizerLine grabSpecimen3Points = new PPPoint.beizerLine(
            20,
            38.74,
            360
    );
    /* line8 */
    public static PPPoint.beizerLine scoreSpecimen4Points = new PPPoint.beizerLine(
            33,
            67,
            360
    );
    /* line9 */
    public static PPPoint.beizerLine grabSpecimen4Points = new PPPoint.beizerLine(
            18.8,
            38.74,
            360
    );
    /* line10 */
    public static PPPoint.beizerLine scoreSpecimen5Points = new PPPoint.beizerLine(
            32,
            66,
            360
    );
    /* line11 */
    public static PPPoint.beizerLine parkPoints = new PPPoint.beizerLine(
            18.8,
            49.23,
            -130
    );

    /* line1-2a */
    public static PathChain scoreSpecimen1AndPushBlock1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                new Point(startPos),
                                scoreSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
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
    /* line2a */
    public static PathChain pushBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        moveToPushLoc1Points.getMiddlePoint(),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(scoreSpecimen1Points.getEndHeading(), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line2b */
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
    /* line2c */
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
    public static PathChain grabSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                pushBlock3Points.getEndPoint(),
                                grabSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line4 */
    public static PathChain scoreSpecimen2() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen1Points.getEndPoint(),
                                scoreSpecimen2Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line5 */
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
    /* line6 */
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
    /* line7 */
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
    /* line8 */
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
    /* line9 */
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
    /* line10 */
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
    /* line11 */
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

    /***
     * TESTING AUTO METHODS
    ***/

    /* line1 */
    public static PathChain scoreSpecimen1(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                scoreSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line2a */
    public static PathChain pushBlock1(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen5Points.getStartPoint(),
                        moveToPushLoc1Points.getMiddlePoint(),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(startPos.getHeading(), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        moveToPushLoc1Points.getEndPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getEndHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line2b */
    public static PathChain pushBlock2(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen5Points.getStartPoint(),
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
    /* line2c */
    public static PathChain pushBlock3(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen5Points.getStartPoint(),
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
    public static PathChain grabSpecimen1(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                grabSpecimen1Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line4 */
    public static PathChain scoreSpecimen2(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                scoreSpecimen2Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line5 */
    public static PathChain grabSpecimen2(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                grabSpecimen2Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line6 */
    public static PathChain scoreSpecimen3(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                scoreSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line7 */
    public static PathChain grabSpecimen3(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                grabSpecimen3Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line8 */
    public static PathChain scoreSpecimen4(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                scoreSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line9 */
    public static PathChain grabSpecimen4(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                grabSpecimen4Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen4Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }
    /* line10 */
    public static PathChain scoreSpecimen5(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                scoreSpecimen5Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen5Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line11 */
    public static PathChain park(Pose startPos) {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getStartPoint(),
                                parkPoints.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
}