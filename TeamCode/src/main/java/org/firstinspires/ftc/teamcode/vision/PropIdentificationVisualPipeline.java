package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropIdentificationVisualPipeline extends OpenCvPipeline {
    //    Changes how "zoomed-in" the camera sees
    static final double SQUARE_SIZE_PX = 90; // og 75
    // If the camera size is too small, these boxes could end up overlapping and causing problems. For the 22-23 and 23-24 years, the camera was 1280x720.
    static final Rect LeftROI = new Rect(
            new Point(constants.DETECTION_BOX_OFFSET_SIDES_PX, constants.FRONT_CAMERA_HEIGHT / 2.0 - SQUARE_SIZE_PX / 2.0),
            new Point(SQUARE_SIZE_PX+ constants.DETECTION_BOX_OFFSET_SIDES_PX, constants.FRONT_CAMERA_HEIGHT / 2.0 + SQUARE_SIZE_PX / 2.0)
    );
    static final Rect CenterROI = new Rect(
            new Point(constants.FRONT_CAMERA_WIDTH / 2.0 - SQUARE_SIZE_PX / 2.0,
                    constants.FRONT_CAMERA_HEIGHT / 2.0 - SQUARE_SIZE_PX / 2.0),
            new Point(constants.FRONT_CAMERA_WIDTH / 2.0 + SQUARE_SIZE_PX / 2.0,
                    constants.FRONT_CAMERA_HEIGHT / 2.0 + SQUARE_SIZE_PX / 2.0)
    );
    static final Rect RightROI = new Rect(
            new Point(constants.FRONT_CAMERA_WIDTH - SQUARE_SIZE_PX - constants.DETECTION_BOX_OFFSET_SIDES_PX, constants.FRONT_CAMERA_HEIGHT / 2.0 - SQUARE_SIZE_PX / 2.0),
            new Point(constants.FRONT_CAMERA_WIDTH - constants.DETECTION_BOX_OFFSET_SIDES_PX, constants.FRONT_CAMERA_HEIGHT / 2.0 + SQUARE_SIZE_PX / 2.0)
    );
    static final double HUE_DIFF = 15;
    static final double SAT_DIFF = 205;
    static final double VAL_DIFF = 185;

    // OpenCV uses the HSV range H(0-180), S(0-255), V(0-255).

    //    The color red is centered around 0, so we need two ranges to measure high-end and low-end values.
    // todo does the implementation of ^ mean that the camera is reading more red val than it should compared to blue?
//    Red color range below 180
    static final Scalar redHSVLow1 = new Scalar(180 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar redHSVHigh1 = new Scalar(180, 255, 255);
    //    Red color range above 0
    static final Scalar redHSVLow2 = new Scalar(0, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar redHSVHigh2 = new Scalar(0 + HUE_DIFF, 255, 255);
    //    Blue color range
    static final Scalar blueHSVLow = new Scalar(120 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
    static final Scalar blueHSVHigh = new Scalar(120 + HUE_DIFF, 255, 255);

    private final Mat hsv = new Mat();
    private final Mat grey = new Mat();

    public enum PropColors {
        RED,
        BLUE
    }
    public enum PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }
    private final PropColors colorToSearch;
    private PropLocation propLocation;

    public PropIdentificationVisualPipeline(PropColors colorToSearchFor){
        this.colorToSearch = colorToSearchFor;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        if(colorToSearch == PropColors.BLUE){
            Mat box;

            // todo figure out what exactly this func does - grey out everything outside scalar range?
            Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
            box = grey.submat(LeftROI);
            double leftBluePercentage = Core.sumElems(box).val[0] / LeftROI.area() / 255;

            Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
            box = grey.submat(CenterROI);
            double centerBluePercentage = Core.sumElems(box).val[0] / CenterROI.area() / 255;

            Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
            box = grey.submat(RightROI);
            double rightBluePercentage = Core.sumElems(box).val[0] / RightROI.area() / 255;

            double max = Math.max(leftBluePercentage, Math.max(centerBluePercentage, rightBluePercentage));
            // The box most detected is made purple
            if(max <= constants.COLOR_UNKNOWN_THRESHOLD_PERCENT){
                propLocation = PropLocation.UNKNOWN;
                Imgproc.rectangle(input, LeftROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(0, 0, 255), 3);
            } else if (max == leftBluePercentage) {
                propLocation = PropLocation.LEFT;
                Imgproc.rectangle(input, LeftROI, new Scalar(255, 0, 255), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(0, 0, 255), 3);
            } else if (max == centerBluePercentage) {
                propLocation = PropLocation.CENTER;
                Imgproc.rectangle(input, LeftROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(255, 0, 255), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(0, 0, 255), 3);
            } else if (max == rightBluePercentage) {
                propLocation = PropLocation.RIGHT;
                Imgproc.rectangle(input, LeftROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(255, 0, 255), 3);
            }
        } else {
            // must be red
            Mat box;

            // left stuff
            Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
            box = grey.submat(LeftROI);
            double leftRedPercentage = Core.sumElems(box).val[0] / LeftROI.area() / 255;

            Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
            box = grey.submat(LeftROI);
            leftRedPercentage += leftRedPercentage + Core.sumElems(box).val[0] / LeftROI.area() / 255;
            leftRedPercentage /= 2.0;

            // center stuff
            Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
            box = grey.submat(CenterROI);
            double centerRedPercentage = Core.sumElems(box).val[0] / CenterROI.area() / 255;

            Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
            box = grey.submat(CenterROI);
            centerRedPercentage += centerRedPercentage + Core.sumElems(box).val[0] / CenterROI.area() / 255;
            centerRedPercentage /= 2.0;

            // right stuff
            Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
            box = grey.submat(RightROI);
            double rightRedPercentage = Core.sumElems(box).val[0] / RightROI.area() / 255;

            Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
            box = grey.submat(RightROI);
            rightRedPercentage += rightRedPercentage + Core.sumElems(box).val[0] / RightROI.area() / 255;
            rightRedPercentage /= 2.0;

            double max = Math.max(leftRedPercentage, Math.max(centerRedPercentage, rightRedPercentage));
            if(max <= constants.COLOR_UNKNOWN_THRESHOLD_PERCENT){
                propLocation = PropLocation.UNKNOWN;
                Imgproc.rectangle(input, LeftROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(255, 0, 0), 3);
            } else if (max == leftRedPercentage) {
                propLocation = PropLocation.LEFT;
                Imgproc.rectangle(input, LeftROI, new Scalar(255, 0, 255), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(255, 0, 0), 3);
            } else if (max == centerRedPercentage) {
                propLocation = PropLocation.CENTER;
                Imgproc.rectangle(input, LeftROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(255, 0, 255), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(255, 0, 0), 3);
            } else if (max == rightRedPercentage) {
                propLocation = PropLocation.RIGHT;
                Imgproc.rectangle(input, LeftROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, CenterROI, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, RightROI, new Scalar(255, 0, 255), 3);
            }
        }
        return input;
    }

    public PropLocation getLastPropLocation(){
        return propLocation;
    }
}
