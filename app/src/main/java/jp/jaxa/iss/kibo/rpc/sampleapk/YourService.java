package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;
import android.util.Pair;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.android.OpenCVLoader;
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
import static org.opencv.core.CvType.CV_32F;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private Pair<Integer, Integer> nullPII = new Pair<>(-1,-1);
    private Scalar silver = new Scalar(192,192,192);
    double mX = 0, mY = 0, mZ = 0;

    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();

        //initialize opencv
        if(OpenCVLoader.initDebug()){
            Log.i("init", "Successfully loaded OpenCV");
        }else{
            Log.i("init","Fail to load");
        }


        // move to point 1
        Point point = new Point(10.71f, -7.7f, 4.48f);
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        moveTo2(point, quaternion, true);

        // report point1 arrival
        api.reportPoint1Arrival();

        // get camera intrinsics for fisheye calibration
        double[][] intrinsics = api.getNavCamIntrinsics();
        Log.i("getIntrinsics", "" + intrinsics);
        double[] K_val = intrinsics[0];
        double[] D_val = intrinsics[1];

        // convert double[][] to two mats
        // cannot move K and D to global variable because it has to be after OpenCV has init
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
        Log.i("getIntrinsics", K.dump());
        Log.i("getIntrinsics", D.dump());

        Mat image1 = api.getMatNavCam();
        api.saveMatImage(image1, "point1.png");

        image1 = calibrateNavFisheyeCam(image1, K, D);
        api.saveMatImage(image1, "point1_calibrated.png");

        Pair<Integer, Integer> target1Loc = getTarget1Loc(image1);

        target1Loc = calibrateMovementAtTarget1(target1Loc.first, quaternion, K, D);

        if (target1Loc != nullPII) {
            if (moveToPointOnImageForTarget1(target1Loc.first, target1Loc.second, quaternion, K, D)) {
                // irradiate the laser
                api.laserControl(true);
                // take target1 snapshots
                api.takeTarget1Snapshot();
                // turn the laser off
                api.laserControl(false);
            }
        }

        Log.i("pos", "start of moving to point 2");

        //move to point 2
        Point point2 = new Point(11.2746f, -9.92284f,  5.29881f);
        //avoid koz for penalty
        Point avoid = new Point(11f, -8.2f, 4.75f);
        Point avoid2 = new Point(11.1f, -9.5f, 4.75f);
        Quaternion quaternion2 = new Quaternion(0f, 0f, -0.707f, 0.707f);

        //move to
        moveTo2(avoid, quaternion2, true);
        Log.i("pos", "move to avoid");
        moveTo2(avoid2, quaternion2, true);
        Log.i("pos", "move to avoid2");
        moveTo2(point2, quaternion2, true);
        Log.i("pos", "move to point 2");

        //save debug image for point 2
        Mat image2 = api.getMatNavCam();
        api.saveMatImage(image2, "point2.png");

        image2 = calibrateNavFisheyeCam(image2, K, D);
        api.saveMatImage(image2, "point2_calibrated.png");

        Pair<Integer, Integer> target2Loc = getTarget2Loc(image2);

        target2Loc = calibrateMovementAtTarget2(target2Loc.first, quaternion2, K, D);

        if (target2Loc != nullPII) {
            if (moveToPointOnImageForTarget2(target2Loc.first, target2Loc.second, quaternion2, K, D)) {
                api.laserControl(true);
                api.takeTarget2Snapshot();
                api.laserControl(false);
            }
        }

        //move to goal
        Point goal = new Point(11.27460f, -7.89178f, 4.96538f);
        moveTo2(avoid2, quaternion2, true);
        Log.i("pos", "move to avoid2");
        moveTo2(avoid, quaternion2, true);
        Log.i("pos", "move to avoid");
        moveTo2(goal, quaternion2, true);
        Log.i("pos", "move to goal");

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

    private void relativeMoveTo2(Point point, Quaternion quaternion, boolean print) {
        final int LOOP_MAX = 5;
        Point currRobotPoint = api.getRobotKinematics().getPosition();
        Point realPoint = new Point(currRobotPoint.getX() + point.getX(), currRobotPoint.getY() + point.getY(), currRobotPoint.getZ() + point.getZ());
        Result success;
        success = api.moveTo(realPoint, quaternion, print);
        //loop count
        int i = 0;
        //detect if move on realPoint succeeded, if not, try again. Wont try over 5 times
        while(!success.hasSucceeded() && i < LOOP_MAX){
            success = api.moveTo(realPoint, quaternion, print);
            i++;
        }
    }

    private Mat getNavCamAndCalibrate(Mat K, Mat D) {
        return calibrateNavFisheyeCam(api.getMatNavCam(), K, D);
    }

    private Mat calibrateNavFisheyeCam(Mat img, Mat K, Mat D) {
        Mat undistorted = new Mat();
        Imgproc.undistort(img, undistorted, K, D);
        return undistorted;
    }



    private Pair<Integer, Integer> calibrateMovementAtTarget1(int currX, Quaternion quaternion, Mat K, Mat D) {

        // (2) calibrate movement (find mY from change in X-axis of image after movement, mZ from change in Y-axis)
        // toMove = pxloc * v, where toMove is the value of movement, pxloc = beforeMoveTargetLoc - afterMoveTargetLoc, v is mY or mZ
        // i.e. v = toMove/pxloc
        // this returns the final position for further movement
        // please only use this function at point1

        Point moveForX = new Point(0, 0.2, 0);
        relativeMoveTo2(moveForX, quaternion, true);
        Mat curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveForX = getTarget1Loc(curr);
        mY = 0.2 / (currX - afterMoveForX.first);

        Point moveForY = new Point(0.2, 0, 0);
        relativeMoveTo2(moveForY, quaternion, true);
        curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveForY = getTarget1Loc(curr);
        mX = 0.2 / (afterMoveForX.second - afterMoveForY.second);

        Log.i("reportCalibrateMovement", "mX: " + mX + "; mY: " + mY);

        Point moveBack = new Point(-0.2, -0.2, 0);
        relativeMoveTo2(moveBack, quaternion, true);
        curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveBack = getTarget1Loc(curr);

        return afterMoveBack;
    }

    private Pair<Integer, Integer> calibrateMovementAtTarget2(int currX, Quaternion quaternion, Mat K, Mat D) {
        Point moveForX = new Point(0.2, 0, 0);
        relativeMoveTo2(moveForX, quaternion, true);
        Mat curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveForX = getTarget2Loc(curr);
        mX = 0.2 / (currX - afterMoveForX.first);

        Point moveForY = new Point(0, 0, 0.2);
        relativeMoveTo2(moveForY, quaternion, true);
        curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveForY = getTarget2Loc(curr);
        mZ = 0.2 / (afterMoveForX.second - afterMoveForY.second);

        Log.i("reportCalibrateMovement", "mX: " + mX + "; mZ: " + mZ);

        Point moveBack = new Point(-0.2, 0, -0.2);
        relativeMoveTo2(moveBack, quaternion, true);
        curr = getNavCamAndCalibrate(K, D);
        Pair<Integer, Integer> afterMoveBack = getTarget2Loc(curr);

        return afterMoveBack;

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

        Log.i("aruco", "detect finish");

        //draw markers on image1
        Aruco.drawDetectedMarkers(image1, corners, ids);
        api.saveMatImage(image1, "draw markers 1.png");

        if (ids.size().height < 4) {
            // not all artags are detected, this algorithm will fail
            Log.i("reportPoint2", "Less ARtags than expected. Skipping Point 1.");
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
            Log.i("ARtag",  Arrays.toString(ids.get(i, 0)) + "Data: {"+ Arrays.toString(pLU) + ", " + Arrays.toString(pLD) + ", " + Arrays.toString(pRU) + ", " + Arrays.toString(pRD) + "}");
        }

        targetPixelX /= 16;
        targetPixelY /= 16;

        //draw a circle at target
        org.opencv.core.Point target1 = new org.opencv.core.Point(targetPixelX, targetPixelY);
        Imgproc.circle(image1, target1, 10, silver, -1);
        api.saveMatImage(image1, "draw target 1.png");

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

        Log.i("aruco", "detect 2 finish");

        //draw markers on image2
        Mat drawMarkersImg = image2;
        Aruco.drawDetectedMarkers(drawMarkersImg, corners, ids);
        api.saveMatImage(drawMarkersImg, "draw markers 2.png");

        if (ids.size().height < 4) {
            // did not detect all ARtags
            Log.i("reportPoint2", "Less ARtags than expected. Skipping Point 2.");
            return nullPII;
        }
        int tY = -1, bY = 99999, lX= -1, rX = 99999;
        Log.i("reportPoint2", "Scanning...");
        for(int i = 0; i < ids.size().height; i++) {
            Mat mat = corners.get(i);
            double[] currentIdDA = ids.get(i, 0);
            int currentId = (int) currentIdDA[0];
            Log.i("reportPoint2", "Scanning id: " + currentId);

            double[] pLU = mat.get(0, 0);
            double[] pRU = mat.get(0, 1);
            double[] pRD = mat.get(0, 2);
            double[] pLD = mat.get(0, 3);

            Log.i("reportPoint2", "pLU: (" + pLU[0] + "," + pLU[1] + ")");
            Log.i("reportPoint2", "pRU: (" + pRU[0] + "," + pRU[1] + ")");
            Log.i("reportPoint2", "pLD: (" + pLD[0] + "," + pLD[1] + ")");
            Log.i("reportPoint2", "pRD: (" + pRD[0] + "," + pRD[1] + ")");

            if (currentId == 11) {
                tY = max(tY, (int)pLU[1]);
                rX = min(rX, (int)pLU[0], (int)pLD[0]);
            } else if (currentId == 12) {
                tY = max(tY, (int)pRU[1]);
                lX = max(lX, (int)pRU[0], (int)pRD[0]);
            } else if (currentId == 13) {
                bY = min(bY, (int)pRD[1]);
                lX = max(lX, (int)pRU[0], (int)pRD[0]);
            } else if (currentId == 14) {
                bY = min(bY, (int)pLD[1]);
                rX = min(rX, (int)pLU[0], (int)pLD[0]);
            }
        }

        // sometimes the target is higher or lower than the highest artag
        tY -= 30;
        bY += 15;

        Log.i("reportPoint2","ok: lX: " + lX + "; rX: " + rX + "; tY: " + tY + "; bY: " + bY);

        Rect rect = new Rect(lX, tY, rX-lX, bY-tY);
        // cardArea is the card's area, without the ARtags
        Mat cardArea = image2.submat(rect);
        Log.i("reportPoint2", "processed submat");
        api.saveMatImage(cardArea, "point2_cardArea.png");

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
        org.opencv.core.Point target2 = new org.opencv.core.Point(targetX, targetY);
        Imgproc.circle(image2, target2, 10, silver, -1);
        api.saveMatImage(image2, "draw target 2.png");

        Pair<Integer, Integer> ans = new Pair<>(targetX, targetY);
        return ans;
    }

    private boolean moveToPointOnImageForTarget1(int targetX, int targetY, Quaternion quaternion, Mat K, Mat D) {
        double dx = targetX - 640;
        double dy = targetY - 480;
        Log.i("reportMoveToPoint", "dx: " + dx + "; dy: " + dy);
        Point moveTo = new Point(dy * mX, dx * mY, 0);
        Log.i("reportMoveToPoint", "" + moveTo);
        relativeMoveTo2(moveTo, quaternion, true);

        // DEBUG
        Mat img = getNavCamAndCalibrate(K, D);
        api.saveMatImage(img, "T1_shouldbecenter.png");
        Pair<Integer, Integer> pxloc = getTarget1Loc(img);
        Log.i("reportMoveToPoint_T1", "(x, y) = " + pxloc.first + ", " + pxloc.second);

        moveTo = new Point(-0.0285, 0.0994, 0);
        relativeMoveTo2(moveTo, quaternion, true);

        img = getNavCamAndCalibrate(K, D);
        api.saveMatImage(img, "T1_afterMatchToLaser.png");
        pxloc = getTarget1Loc(img);
        Log.i("reportMoveToPoint_T1_afterCalib", "(x, y) = " + pxloc.first + ", " + pxloc.second);

        return true;
    }

    private boolean moveToPointOnImageForTarget2(int targetX, int targetY, Quaternion quaternion, Mat K, Mat D) {
        double dx = targetX - 640;
        double dy = targetY - 480;
        Log.i("reportMoveToPoint", "dx: " + dx + "; dy: " + dy);
        Point moveTo = new Point(dx * mX, 0, dy * mZ);
        Log.i("reportMoveToPoint", "" + moveTo);
        relativeMoveTo2(moveTo, quaternion, true);

        // DEBUG
        Mat img = getNavCamAndCalibrate(K, D);
        api.saveMatImage(img, "T2_shouldbecenter.png");
        Pair<Integer, Integer> pxloc = getTarget2Loc(img);
        Log.i("reportMoveToPoint_T2", "(x, y) = " + pxloc.first + ", " + pxloc.second);

        moveTo = new Point(-0.0994, 0, 0.0285);
        relativeMoveTo2(moveTo, quaternion, true);

        img = getNavCamAndCalibrate(K, D);
        api.saveMatImage(img, "T2_afterMatchToLaser.png");
        pxloc = getTarget2Loc(img);
        Log.i("reportMoveToPoint_T2_afterCalib", "(x, y) = " + pxloc.first + ", " + pxloc.second);

        return true;
    }
}