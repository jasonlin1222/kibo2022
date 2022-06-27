package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;
import android.util.Pair;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.google.common.primitives.Ints.max;
import static com.google.common.primitives.Ints.min;
import static java.lang.Math.abs;
import static org.opencv.core.CvType.CV_32F;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String LOG_PREFIX = "WillPower_";
    private final Pair<Integer, Integer> nullPII = new Pair<>(-1,-1);
    private final Scalar silver = new Scalar(192,192,192);
    private final double T1_BASIC_SLEEP = 5;
    private final double T1_CALIB_SLEEP = 5;
    private final double T1_FINAL_SLEEP = 7;
    private final double T2_BASIC_SLEEP = 5;
    private final double T2_CALIB_SLEEP = 5;
    private final double T2_FINAL_SLEEP = 5;
    private final double FIXED_M = 0.000976652;

    private double mX, mY, mZ;

    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();

        // initialize opencv can be removed because the inherited class already has a function
        // ``onGuestScienceStart()`` that initializes it

        // get camera intrinsics for fisheye calibration
        double[][] intrinsics = api.getNavCamIntrinsics();
        log("getIntrinsics", "" + intrinsics);
        double[] K_val = intrinsics[0];
        double[] D_val = intrinsics[1];

        // convert double[][] to two mats
        Mat K = new Mat(3, 3, CV_32F), D = new Mat(5, 1, CV_32F);
        int cnt = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K.put(i, j, K_val[cnt++]);
            }
        }
        for (int i = 0; i < 5; i++) {
            D.put(i, 0, D_val[i]);
        }
        log("getIntrinsics", K.dump());
        log("getIntrinsics", D.dump());


        // move to point 1
        Point point = new Point(10.71f, -7.7f, 4.48f);
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        moveTo2(point, quaternion, true);

        // report point1 arrival
        api.reportPoint1Arrival();

        Point closerPoint1 = new Point(10.71f, -7.7f, 4.33f);
        moveTo2(closerPoint1, quaternion, true);

        sleep(T1_BASIC_SLEEP);
        Mat image1 = getNavCamAndCalibrateFisheye(K, D);
//        api.saveMatImage(image1, "calibrated_point1.png");

        Pair<Integer, Integer> target1Loc = getTarget1Loc(image1);

        target1Loc = calibrateScaleAtT1(target1Loc, quaternion, K, D);

        if (target1Loc != nullPII) {
            if (moveToPointOnImageForT1(target1Loc.first, target1Loc.second, quaternion, K, D)) {
                // irradiate the laser
                api.laserControl(true);
                // take target1 snapshots
                api.takeTarget1Snapshot();
                // turn the laser off
                api.saveMatImage(getNavCamAndCalibrateFisheye(K, D), "laser_t1.png");
                api.laserControl(false);
            }
        }

        log("pos", "start of moving to point 2");

        //move to point 2
        Point point2 = new Point(11.2746f, -9.92284f,  5.29881f);
        Point closerPoint2 = new Point(11.2746f, -10.05f,  5.29881f);
        //avoid koz for penalty
        Point avoid = new Point(11f, -8.2f, 4.75f);
        Point avoid2 = new Point(11.1f, -9.5f, 4.75f);
        Quaternion quaternion2 = new Quaternion(0.5f, -0.5f, -0.5f, 0.5f);

        //move to
        moveTo2(avoid, quaternion, true);
        log("pos", "move to avoid");
        moveTo2(avoid2, quaternion2, true);
        log("pos", "move to avoid2");

        moveTo2(point2, quaternion2, true);
        log("pos", "move to point 2");

        moveTo2(closerPoint2, quaternion2, true);
        log("pos", "move to closer point 2");

        //save debug image for point 2
        sleep(T2_BASIC_SLEEP);
        Mat image2 = getNavCamAndCalibrateFisheye(K, D);
//        api.saveMatImage(image2, "calibrated_point2.png");

        Pair<Integer, Integer> target2Loc = getTarget2Loc(image2);

        target2Loc = calibrateScaleAtT2(target2Loc, quaternion2, K, D);

        if (target2Loc != nullPII) {
            if (moveToPointOnImageForT2(target2Loc.first, target2Loc.second, quaternion2, K, D)) {
                log("testDebugSomethingStuff", "In IF");
                api.laserControl(true);
                log("testDebugSomethingStuff", "Laser on");
                api.takeTarget2Snapshot();
                log("testDebugSomethingStuff", "Snapshots over");
                api.saveMatImage(getNavCamAndCalibrateFisheye(K, D), "laser_t2.png");
                log("testDebugSomethingStuff", "Saved laser_t2.png");
                api.laserControl(false);
                log("testDebugSomethingStuff", "Laser off. T2 over.");
            }
        }

//        moveTo2(point2, quaternion2, true);
//        log("pos", "move back to regular point 2");

        //move to goal
        Quaternion missionCompleteQuaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Point goal = new Point(11.27460f, -7.89178f, 4.96538f);
        moveTo2(avoid2, quaternion2, true);
        log("pos", "move to avoid2");
        moveTo2(avoid, missionCompleteQuaternion, true);
        log("pos", "move to avoid");
        moveTo2(goal, missionCompleteQuaternion, true);
        log("pos", "move to goal");

        // send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method
    //move to that deals with randomness
    private void moveTo2(Point point, Quaternion quaternion, boolean print){
        final int LOOP_MAX = 5;
        Result success;
        success = api.moveTo(point, quaternion, print);
        //loop count
        int i = 0;
        //detect if move on point succeeded, if not, try again. Wont try over 5 times
        while(!success.hasSucceeded() && i < LOOP_MAX){
            success = api.moveTo(point, quaternion, print);
            i++;
        }
    }

    private void relativeMoveTo2(Point relativePoint, Quaternion quaternion, boolean print){
        final int LOOP_MAX = 5;
        Result success;
        Point currPos = api.getRobotKinematics().getPosition();
        Point point = new Point(currPos.getX() + relativePoint.getX(), currPos.getY() + relativePoint.getY(), currPos.getZ() + relativePoint.getZ());
        success = api.moveTo(point, quaternion, print);
        //loop count
        int i = 0;
        //detect if move on point succeeded, if not, try again. Wont try over 5 times
        while(!success.hasSucceeded() && i < LOOP_MAX){
            success = api.moveTo(point, quaternion, print);
            i++;
        }
    }

    private void log(String id, String msg) {
        Log.i(LOG_PREFIX + id, msg);
    }

    private void sleep(double seconds) {
        log("sleepFunction", "Start sleeping for " + seconds + " seconds");
        int delay = (int)(seconds * 1000); // number of milliseconds to sleep

        long start = System.currentTimeMillis();
        while (start >= System.currentTimeMillis() - delay);

        log("sleepFunction", "Done sleeping for " + (System.currentTimeMillis() - start));
    }

//    private boolean api.saveMatImage(Mat img, String fileName) {
////        api.api.saveMatImage(img, fileName);
//        return true;
//    }

    private Mat getNavCamAndCalibrateFisheyeAndDrawCenter(Mat K, Mat D) {
        Mat img = getNavCamAndCalibrateFisheye(K, D);
        org.opencv.core.Point center = new org.opencv.core.Point(640, 480);
        Imgproc.circle(img, center, 10, silver, -1);
        return img;
    }

    private Mat getNavCamAndCalibrateFisheye(Mat K, Mat D) {
        return calibrateNavFisheyeCam(api.getMatNavCam(), K, D);
    }

    private Mat calibrateNavFisheyeCam(Mat img, Mat K, Mat D) {
        Mat undistorted = new Mat();
        Imgproc.undistort(img, undistorted, K, D);
        return undistorted;
    }

    private Pair<Integer, Integer> getTarget1Loc(Mat image1) {
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        //detect artags
        Aruco.detectMarkers(
                image1,
                Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                corners,
                ids);

        log("aruco", "detect finish");

        //draw markers on image1
//        Aruco.drawDetectedMarkers(image1, corners, ids);
//        api.saveMatImage(image1, "draw markers 1_" + (image_id++) + ".png");

        if (ids.size().height < 4) {
            // not all artags are detected, this algorithm will fail
            log("reportPoint2", "Less ARtags than expected. Skipping Point 1.");
            return nullPII;
        }

        //get target position on the photo
        double targetPixelX = 0;
        double targetPixelY = 0;

        for(int i = 0; i < ids.size().height; i++){
            Mat mat = corners.get(i);
            double[] pLU = mat.get(0, 0);
            double[] pRU = mat.get(0, 1);
            double[] pRD = mat.get(0, 2);
            double[] pLD = mat.get(0, 3);
            targetPixelX += pLU[0] + pLD[0] + pRD[0] + pRU[0];
            targetPixelY += pLU[1] + pLD[1] + pRD[1] + pRU[1];
            log("ARtag",  Arrays.toString(ids.get(i, 0)) + "Data: {"+ Arrays.toString(pLU) + ", " + Arrays.toString(pLD) + ", " + Arrays.toString(pRU) + ", " + Arrays.toString(pRD) + "}");
        }

        targetPixelX /= 16;
        targetPixelY /= 16;

        //draw a circle at target
//        org.opencv.core.Point target1 = new org.opencv.core.Point(targetPixelX, targetPixelY);
//        Imgproc.circle(image1, target1, 10, silver, -1);
//        api.saveMatImage(image1, "draw target 1_"+ (image_id++) +".png");

        Pair<Integer, Integer> ans = new Pair<>((int)targetPixelX, (int)targetPixelY);
        return ans;
    }

    private Pair<Integer, Integer> getTarget2Loc(Mat image2) {
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        //detect artags
        Aruco.detectMarkers(
                image2,
                Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                corners,
                ids);

        log("aruco", "detect 2 finish");

        //draw markers on image2
//        Mat draw = new Mat(image2.nativeObj);
//        Aruco.drawDetectedMarkers(draw, corners, ids);
//        api.saveMatImage(draw, "draw markers 2_" + (image_id++) + ".png");

        if (ids.size().height < 4) {
            // did not detect all ARtags
            log("reportPoint2", "Less ARtags than expected. Skipping Point 2.");
            return nullPII;
        }
        int tY = -1, bY = 99999, lX= -1, rX = 99999;
        log("reportPoint2", "Scanning...");
        for(int i = 0; i < ids.size().height; i++) {
            Mat mat = corners.get(i);
            double[] currentIdDA = ids.get(i, 0);
            int currentId = (int) currentIdDA[0];
            log("reportPoint2", "Scanning id: " + currentId);

            double[] pLD = mat.get(0, 0);
            double[] pLU = mat.get(0, 1);
            double[] pRU = mat.get(0, 2);
            double[] pRD = mat.get(0, 3);

            log("reportPoint2", "pLU: (" + pLU[0] + "," + pLU[1] + ")");
            log("reportPoint2", "pRU: (" + pRU[0] + "," + pRU[1] + ")");
            log("reportPoint2", "pLD: (" + pLD[0] + "," + pLD[1] + ")");
            log("reportPoint2", "pRD: (" + pRD[0] + "," + pRD[1] + ")");

            if (currentId == 11) {
                lX = max(lX, (int)pLD[0]);
                tY = max(tY, (int)pLD[1], (int)pRD[1]);
            } else if (currentId == 12) {
                lX = max(lX, (int)pLU[0]);
                bY = min(bY, (int)pLU[1], (int)pRU[1]);
            } else if (currentId == 13) {
                rX = min(rX, (int)pRU[0]);
                bY = min(bY, (int)pLU[1], (int)pRU[1]);
            } else if (currentId == 14) {
                rX = min(rX, (int)pRD[0]);
                tY = max(tY, (int)pLD[1], (int)pRD[1]);
            }

        }

        // sometimes the target is higher or lower than the highest artag
        lX -= 30;
        rX += 15;

        log("reportPoint2","ok: lX: " + lX + "; rX: " + rX + "; tY: " + tY + "; bY: " + bY);

        Rect rect = new Rect(lX, tY, rX-lX, bY-tY);
        // cardArea is the card's area, without the ARtags
        Mat cardArea = image2.submat(rect);
//        log("reportPoint2", "processed submat");
//        api.saveMatImage(cardArea, "point2_cardArea_" + (image_id++) + ".png");

        // The camera takes 256-bit color pictures
        // each pixel is an integer between 0 and 255
        // AKA if cardArea.get(1,2)[0] is 6, then the color of (1,2) is (6,6,6), close to black
        // This is averaging out the location of each pixel with the value lower than 80, leading us to the target
        int targetX = 0, targetY = 0, targetPxCount = 0;
        for (int i = 0; i < cardArea.size().height; i++) {
            for (int j = 0; j < cardArea.size().width; j++) {
                if (cardArea.get(i, j)[0] < 80) {
                    targetPxCount++; targetX += j; targetY += i;
                }
            }
        }

        // Averaging out
        targetX /= targetPxCount; targetY /= targetPxCount;
        // Because it is using cardArea, add it back to using image2
        targetX += lX; targetY += tY;

        // draw a circle at target
//        org.opencv.core.Point target2 = new org.opencv.core.Point(targetX, targetY);
//        Imgproc.circle(image2, target2, 10, silver, -1);
//        api.saveMatImage(image2, "draw target 2_" + (image_id++) + ".png");

        Pair<Integer, Integer> ans = new Pair<>(targetX, targetY);
        return ans;
    }

    private Pair<Integer, Integer> calibrateScaleAtT1(Pair<Integer, Integer> beforeMove, Quaternion quaternion, Mat K, Mat D) {

//        Point relativePoint = new Point(0, 0.2, 0);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T1_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForX = getTarget1Loc(getNavCamAndCalibrateFisheye(K, D));
//        mY = 0.2 / (beforeMove.first - afterMoveForX.first);
//
//        relativePoint = new Point(0.2, 0, 0);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T1_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForY = getTarget1Loc(getNavCamAndCalibrateFisheye(K, D));
//        mX = 0.2 / (afterMoveForX.second - afterMoveForY.second);
//
//        log("afterCalibrateAtT1", "mY: " + mY + "; mX: " + mX);

        // 0.0013 ~ 0.0006 -> 0.000975

//        return afterMoveForY;

        mY = mX = FIXED_M;
        return beforeMove;
    }

    private Pair<Integer, Integer> calibrateScaleAtT2(Pair<Integer, Integer> beforeMove, Quaternion quaternion, Mat K, Mat D) {

//        Point relativePoint = new Point(0, 0, -0.2);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T2_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForX = getTarget2Loc(getNavCamAndCalibrateFisheye(K, D));
//        mZ = (-0.2) / (beforeMove.first - afterMoveForX.first);
//
//        relativePoint = new Point(0, 0, 0.2);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T2_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForX2 = getTarget2Loc(getNavCamAndCalibrateFisheye(K, D));
//        mZ = (mZ + (0.2) / (afterMoveForX.first - afterMoveForX2.first)) / 2;
//
//        relativePoint = new Point(-0.2, 0, 0);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T2_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForY = getTarget2Loc(getNavCamAndCalibrateFisheye(K, D));
//        mX = (-0.2) / (afterMoveForX2.second - afterMoveForY.second);
//
//        relativePoint = new Point(0.2, 0, 0);
//        relativeMoveTo2(relativePoint, quaternion, true);
//        sleep(T2_CALIB_SLEEP);
//        Pair<Integer, Integer> afterMoveForY2 = getTarget2Loc(getNavCamAndCalibrateFisheye(K, D));
//        mX = (mX + (0.2) / (afterMoveForY.second - afterMoveForY2.second)) / 2;
//
//        log("afterCalibrateAtT2", "mX: " + mX + "; mZ: " + mZ);
//
//        return afterMoveForY2;

        mX = -1 * FIXED_M;
        mZ = FIXED_M;
        return beforeMove;
    }

    private boolean moveToPointOnImageForT1(int targetX, int targetY, Quaternion quaternion, Mat K, Mat D) {
        double tX = targetX - 640, tY = targetY - 480;

        Point relativePoint = new Point(tY * mX + 0.0285, tX * mY - 0.0994, 0);
        relativeMoveTo2(relativePoint, quaternion, true);

        int beforeX = targetX, beforeY = targetY;

        for (int i = 0; i < 1; i++) {
            double nmX, nmY;
            sleep(T1_FINAL_SLEEP);

            Pair<Integer, Integer> loc = getTarget1Loc(getNavCamAndCalibrateFisheye(K, D));

            log("moveToPointOnImageForT1", "loc: " + loc.first + ", " + loc.second);

            nmY = (tX * mY - 0.0994) / (beforeX - loc.first);
            nmX = (tY * mX + 0.0285) / (beforeY - loc.second);

            log("moveToPointOnImageForT1", "nmY: " + nmY + "; nmX: " + nmX);

            mY = (mY + nmY) / 2;
            mX = (mX + nmX) / 2;

            log("moveToPointOnImageForT1", "mY: " + mY + "; mX: " + mX);

            beforeX = loc.first; beforeY = loc.second;

            tX = loc.first - 640;
            tY = loc.second - 480;

            relativePoint = new Point(tY * mX + 0.0285, tX * mY - 0.0994, 0);
            relativeMoveTo2(relativePoint, quaternion, true);
        }

//        Mat img = getNavCamAndCalibrateFisheye(K, D);
//        Pair<Integer, Integer> t1 = getTarget1Loc(img);
//        log("t1_afterMoveLoc", "x: " + t1.first + "; y: " + t1.second);
//        api.saveMatImage(img, "t1_afteroffset.png");

        return true;
    }

    private boolean moveToPointOnImageForT2(int targetX, int targetY, Quaternion quaternion, Mat K, Mat D) {
        double tX = targetX - 640, tY = targetY - 480;

        Point relativePoint = new Point(tY * mX - 0.0285, 0, tX * mZ - 0.0994);
        relativeMoveTo2(relativePoint, quaternion, true);

        int beforeX = targetX, beforeY = targetY;

        for (int i = 0; i < 2; i++) {
            double nmX, nmZ;
            sleep(T2_FINAL_SLEEP);

            Pair<Integer, Integer> loc = getTarget2Loc(getNavCamAndCalibrateFisheye(K, D));

            log("moveToPointOnImageForT2", "loc: " + loc.first + ", " + loc.second);

            nmZ = (tX * mZ - 0.0994) / (beforeX - loc.first);
            nmX = -1 * Math.abs((tY * mX - 0.0285) / (beforeY - loc.second));

            log("moveToPointOnImageForT2", "nmZ: " + nmZ + "; nmX: " + nmX);

            mZ = (mZ + nmZ) / 2;
            mX = (mX + nmX) / 2;

            log("moveToPointOnImageForT2", "mZ: " + mZ + "; mX: " + mX);

            beforeX = loc.first; beforeY = loc.second;

            tX = loc.first - 640;
            tY = loc.second - 480;

            relativePoint = new Point(tY * mX - 0.0285, 0, tX * mZ - 0.0994);
            relativeMoveTo2(relativePoint, quaternion, true);
        }

//        Mat img = getNavCamAndCalibrateFisheye(K, D);
//        Pair<Integer, Integer> t2 = getTarget2Loc(img);
//        log("t2_afterMoveLoc", "x: " + t2.first + "; y: " + t2.second);
//        api.saveMatImage(img, "t2_afteroffset.png");

//        log("testDebugSomethingStuff", "SAVED IMAGE AT T2");

        return true;
    }
}