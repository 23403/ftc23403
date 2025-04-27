package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.utils.CustomPedroPathing;

import java.util.List;

@Config("5+0 PUSH Auto Paths")
public class FiveSpecimenAutoPushPaths {
    /* start pos */
    public static final Pose startPos = new Pose(9, 63.4, Math.toRadians(0));
    /* line1 */
    public static CustomPedroPathing.beizerLine preloadPoints = new CustomPedroPathing.beizerLine(
            32,
            72.34,
            0
    );
    /* line2 */
    public static CustomPedroPathing.beizerLine grabSpecimen1Points = new CustomPedroPathing.beizerLine(
            21.65,
            35.8,
            preloadPoints.endPointX,
            preloadPoints.endPointY,
            preloadPoints.getEndHeading(),
            0
    );
    /* line3 */
    public static CustomPedroPathing.beizerLine scoreSpecimen1Points = new CustomPedroPathing.beizerLine(
            36.85,
            62.35,
            grabSpecimen1Points.endPointX,
            grabSpecimen1Points.endPointY,
            grabSpecimen1Points.getEndHeading(),
            0
    );
    /* line4a */
    public static CustomPedroPathing.beizerCurve moveToPushLoc1Points = new CustomPedroPathing.beizerCurve(
            scoreSpecimen1Points.endPointX,
            scoreSpecimen1Points.endPointY,
            List.of(27.5, 55.2),
            List.of(21.8, 45.1),
            54.74,
            28,
            scoreSpecimen1Points.getEndHeading(),
            0
    );
    /* line4b */
    public static CustomPedroPathing.beizerLine pushBlock1Points = new CustomPedroPathing.beizerLine(
            21.5,
            28,
            moveToPushLoc1Points.endPointX,
            moveToPushLoc1Points.endPointY,
            moveToPushLoc1Points.getEndHeading(),
            0
    );
    /* line4c */
    public static CustomPedroPathing.beizerCurve moveToPushLoc2Points = new CustomPedroPathing.beizerCurve(
            pushBlock1Points.endPointX,
            pushBlock1Points.endPointY,
            58.74,
            31,
            53.94,
            20.3,
            pushBlock1Points.getEndHeading(),
            0
    );
    /* line4d */
    public static CustomPedroPathing.beizerLine pushBlock2Points = new CustomPedroPathing.beizerLine(
            23.5,
            20.3,
            moveToPushLoc2Points.endPointX,
            moveToPushLoc2Points.endPointY,
            moveToPushLoc2Points.getEndHeading(),
            0
    );
    /* line4e */
    public static CustomPedroPathing.beizerCurve moveToPushLoc3Points = new CustomPedroPathing.beizerCurve(
            pushBlock2Points.endPointX,
            pushBlock2Points.endPointY,
            53.49,
            22.5,
            53.9,
            13.3,
            pushBlock2Points.getEndHeading(),
            0
    );
    /* line4f */
    public static CustomPedroPathing.beizerLine pushBlock3Points = new CustomPedroPathing.beizerLine(
            17.3,
            11.8,
            moveToPushLoc3Points.endPointX,
            moveToPushLoc3Points.endPointY,
            moveToPushLoc3Points.getEndHeading(),
            0
    );
    /* line5 */
    public static CustomPedroPathing.beizerLine scoreSpecimen2Points = new CustomPedroPathing.beizerLine(
            34,
            68,
            0
    );
    /* line6 */
    public static CustomPedroPathing.beizerLine grabSpecimen2Points = new CustomPedroPathing.beizerLine(
            18.5,
            35.8,
            scoreSpecimen2Points.endPointX,
            scoreSpecimen2Points.endPointY,
            scoreSpecimen2Points.getEndHeading(),
            0
    );
    /* line7 */
    public static CustomPedroPathing.beizerLine scoreSpecimen3Points = new CustomPedroPathing.beizerLine(
            33,
            67,
            0
    );
    /* line8 */
    public static CustomPedroPathing.beizerLine grabSpecimen3Points = new CustomPedroPathing.beizerLine(
            18.1,
            35.8,
            scoreSpecimen3Points.endPointX,
            scoreSpecimen3Points.endPointY,
            scoreSpecimen3Points.getEndHeading(),
            0
    );
    /* line9 */
    public static CustomPedroPathing.beizerLine scoreSpecimen4Points = new CustomPedroPathing.beizerLine(
            32,
            66,
            0
    );
    /* line10 */
    public static CustomPedroPathing.beizerCurve parkPoints = new CustomPedroPathing.beizerCurve(
            scoreSpecimen4Points.endPointX,
            scoreSpecimen4Points.endPointY,
            27.2,
            69.3,
            16.8,
            49.23,
            scoreSpecimen4Points.getEndHeading(),
            -130
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
    /* line6 */
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
    public static PathChain park() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen4Points.getEndPoint(),
                        parkPoints.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
}