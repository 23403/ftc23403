package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.pedropathing.pathgen.Point;
import com.pedropathing.util.FeedForwardConstant;

import java.util.List;

public class CustomPedroPathing {

    public static class beizerLine {
        public double startPointX;
        public double startPointY;
        public double endPointX;
        public double endPointY;
        public double startHeading;
        public double endHeading;
        public beizerLine(double endPointX, double endPointY, double startPointX, double startPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        public Point getStartPoint() {
            return new Point(startPointX, startPointY);
        }
        public Point getEndPoint() {
            return new Point(endPointX, endPointY);
        }
        public double getStartHeading() {
            return startHeading;
        }
        public double getEndHeading() {
            return endHeading;
        }
    }

    public static class beizerCurve {
        public double startPointX;
        public double startPointY;
        public double middlePointX;
        public double middlePointY;
        public List<Double> middlePointXL;
        public List<Double> middlePointYL;
        public double endPointX;
        public double endPointY;
        public double startHeading;
        public double endHeading;
        public beizerCurve(double startPointX, double startPointY, double middlePointX, double middlePointY, double endPointX, double endPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.middlePointX = middlePointX;
            this.middlePointY = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        public beizerCurve(double startPointX, double startPointY, List<Double> middlePointX, List<Double> middlePointY, double endPointX, double endPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.middlePointXL = middlePointX;
            this.middlePointYL = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        public Point getStartPoint() {
            return new Point(startPointX, startPointY);
        }
        public Point getEndPoint() {
            return new Point(endPointX, endPointY);
        }
        public Point getMiddlePoint() {
            return new Point(middlePointX, middlePointY);
        }
        public Point getMiddlePoint(int index) {
            return new Point(middlePointXL.get(index), middlePointYL.get(index));
        }
        public double getStartHeading() {
            return startHeading;
        }
        public double getEndHeading() {
            return endHeading;
        }
    }

}