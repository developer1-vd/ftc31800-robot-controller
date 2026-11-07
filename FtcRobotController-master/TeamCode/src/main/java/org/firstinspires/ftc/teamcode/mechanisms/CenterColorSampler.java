package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

/**
 * Measures the color in a small square at the center of the camera image (YCrCb mean and
 * standard deviation), and draws that square on the camera stream so you can aim it at a ball.
 * Used by BallDetectionTest for live color readout and ball color calibration.
 */
public class CenterColorSampler implements VisionProcessor {

    private static final int SIZE = 24;   // pixels, square side

    private final Mat ycrcb = new Mat();
    private final Mat rgb = new Mat();
    private volatile double[] mean = null;
    private volatile double[] std = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int x = frame.cols() / 2 - SIZE / 2, y = frame.rows() / 2 - SIZE / 2;
        Mat center = frame.submat(new Rect(x, y, SIZE, SIZE));
        // Same conversion as the color blob detector: RGB(A) frame -> YCrCb
        if (center.channels() == 4) {
            Imgproc.cvtColor(center, rgb, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(rgb, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        } else {
            Imgproc.cvtColor(center, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        }
        center.release();
        MatOfDouble m = new MatOfDouble(), s = new MatOfDouble();
        Core.meanStdDev(ycrcb, m, s);
        mean = m.toArray();
        std = s.toArray();
        m.release();
        s.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.WHITE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3 * scaleCanvasDensity);
        float half = SIZE / 2f * scaleBmpPxToCanvasPx;
        float cx = onscreenWidth / 2f, cy = onscreenHeight / 2f;
        canvas.drawRect(cx - half, cy - half, cx + half, cy + half, paint);
    }

    /** {Y, Cr, Cb} average at the center, or null before the first frame. */
    public double[] getMean() {
        return mean;
    }

    /** {Y, Cr, Cb} standard deviation at the center, or null before the first frame. */
    public double[] getStd() {
        return std;
    }
}
