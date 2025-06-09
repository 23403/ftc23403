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
            40.8,
            71.6,
            0
    );
    /* line2 */
    public static PPPoint.beizerCurve grabBlock1Points = new PPPoint.beizerCurve(
            28.5,
            24,
            0,
            new PPMP(31.3, 68.1)
    );
    /* line3 */
    public static PPPoint.beizerLine grabBlock2Points = new PPPoint.beizerLine(
            28.5,
            14.7,
            0
    );
    /* line4 */
    public static PPPoint.beizerLine grabBlock3Points = new PPPoint.beizerLine(
            31.3,
            18.3,
            0
    );
    /* line5 */
    public static PPPoint.beizerLine grabSpecimen1Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            0
    );
    /* line6 */
    public static PPPoint.beizerLine scoreSpecimen2Points = new PPPoint.beizerLine(
            36.9,
            74.5,
            0
    );
    /* line7 */
    public static PPPoint.beizerLine grabSpecimen2Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            0
    );
    /* line8 */
    public static PPPoint.beizerLine scoreSpecimen3Points = new PPPoint.beizerLine(
            36.7,
            74.5,
            0
    );
    /* line9 */
    public static PPPoint.beizerLine grabSpecimen3Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            0
    );
    /* line10 */
    public static PPPoint.beizerLine scoreSpecimen4Points = new PPPoint.beizerLine(
            36.5,
            74.5,
            0
    );
    /* line11 */
    public static PPPoint.beizerLine grabSpecimen4Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            0
    );
    /* line12 */
    public static PPPoint.beizerLine scoreSpecimen5Points = new PPPoint.beizerLine(
            36.3,
            74.5,
            0
    );
    /* line13 */
    public static PPPoint.beizerLine grabSpecimen5Points = new PPPoint.beizerLine(
            9.1,
            29.7,
            0
    );
    /* line14 */
    public static PPPoint.beizerLine scoreSpecimen6Points = new PPPoint.beizerLine(
            36.1,
            74.5,
            0
    );
    /* line15 */
    public static PPPoint.beizerLine parkPoints = new PPPoint.beizerLine(
            21.3,
            50.6,
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
                ))
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line2 */
    public static PathChain grabBlock1() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen1Points.getEndPoint(),
                        grabBlock1Points.getMiddlePoint(),
                        grabBlock1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(grabBlock1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line3 */
    public static PathChain grabBlock2() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        grabBlock1Points.getEndPoint(),
                        grabBlock2Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(grabBlock2Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line4 */
    public static PathChain grabBlock3() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        grabBlock2Points.getEndPoint(),
                        grabBlock3Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(grabBlock3Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }

    /* line5 */
    public static PathChain grabSpecimen1() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        grabBlock3Points.getEndPoint(),
                        grabSpecimen1Points.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
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
    public static PathChain grabSpecimen5() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen5Points.getEndPoint(),
                                grabSpecimen5Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(grabSpecimen5Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
    }

    /* line14 */
    public static PathChain scoreSpecimen6() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                grabSpecimen5Points.getEndPoint(),
                                scoreSpecimen6Points.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(scoreSpecimen6Points.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
    /* line15 */
    public static PathChain park() {
        return new PathBuilder()
                .addPath(new BezierLine(
                                scoreSpecimen6Points.getEndPoint(),
                                parkPoints.getEndPoint()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .setZeroPowerAccelerationMultiplier(8)
                .build();
    }
}