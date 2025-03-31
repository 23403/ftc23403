package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.pedropathing.pathgen.Point;
import com.pedropathing.util.FeedForwardConstant;

public class CustomPedroPathing {

    public static class beizerLine {
        public double startPointX;
        public double startPointY;
        public double endPointX;
        public double endPointY;
        public beizerLine(double startPointX, double startPointY, double endPointX, double endPointY) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
        }

        public Point getStartPoint() {
            return new Point(startPointX, startPointY);
        }

        public Point getEndPoint() {
            return new Point(endPointX, endPointY);
        }
    }


    public static class beizerCurve {
        private Point start;
        private Point middle;
        private Point end;

        public beizerCurve(Point start, Point middle, Point end) {
            this.start = start;
            this.middle = middle;
            this.end = end;
        }

        public Point getStartPoint() {
            return start;
        }

        public Point getMiddlePoint() {
            return middle;
        }

        public Point getEndPoint() {
            return end;
        }
    }

}