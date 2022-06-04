package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();

        //initialize opencv
        if(OpenCVLoader.initDebug()){
            Log.d("init", "Successfully loaded OpenCV");
        }else{
            Log.d("init","Fail to load");
        }

        // move to point 1
        Point point = new Point(10.71f, -7.7f, 4.48f);
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        moveTo2(point, quaternion, true);

        // report point1 arrival
        api.reportPoint1Arrival();

        // get a camera image
        Mat image = api.getMatNavCam();

        //target detection: TODO

        // irradiate the laser
        api.laserControl(true);

        //debug cam
        Mat image1 = api.getMatNavCam();
        api.saveMatImage(image1, "point1");

        //save corners and ar-tag ids
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        //detect artags
        Aruco.detectMarkers(
                api.getMatNavCam(),
                Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                corners,
                ids);

        Log.d("aruco", "detect finish");

        //draw markers on image1
        Aruco.drawDetectedMarkers(image1, corners, ids);
        api.saveMatImage(image1, "draw markers");

        //get target position on the photo
        double targetPixelX = 0;
        double targetPixelY = 0;

        for(int i = 0; i < ids.size().height; i++){
            Mat mat = corners.get(i);
            double[] pLU = mat.get(0, 0);
            double[] pLD = mat.get(0, 1);
            double[] pRU = mat.get(0, 2);
            double[] pRD = mat.get(0, 3);
            targetPixelX += pLU[0] + pLD[0] + pRD[0] + pRU[0];
            targetPixelY += pLU[1] + pLD[1] + pRD[1] + pRU[1];
            Log.d("ARtag",  Arrays.toString(ids.get(i, 0)) + "Data: {"+ Arrays.toString(pLU) + ", " + Arrays.toString(pLD) + ", " + Arrays.toString(pRU) + ", " + Arrays.toString(pRD) + "}");
        }

        targetPixelX /= 16;
        targetPixelY /= 16;

        //draw a circle at target
        org.opencv.core.Point target1 = new org.opencv.core.Point(targetPixelX, targetPixelY);
        Scalar blue = new Scalar(0,0,255);
        Imgproc.circle(image1, target1, 10, blue, -1);
        api.saveMatImage(image1, "Draw taget");

        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Aruco.estimatePoseSingleMarkers(image1, 0.05, revcs, revcs, rvecs, tvecs );

        // take target1 snapshots
        api.takeTarget1Snapshot();

        // turn the laser off
        api.laserControl(false);

        //record message to android studios log
        Log.d("pos", "start of moving to point 2");

        //move to point 2
        Point point2 = new Point(11.2746f, -9.92284f,  5.29881f);
        //avoid koz for penalty
        //**change the quaternion later
        Point avoid = new Point(11f, -8.2f, 4.75f);
        Point avoid2 = new Point(11.1f, -9.5f, 4.75f);
        Quaternion quaternion2 = new Quaternion(0f, 0f, -0.707f, 0.707f);

        //move to
        moveTo2(avoid, quaternion2, true);
        Log.d("pos", "move to avoid");
        moveTo2(avoid2, quaternion2, true);
        Log.d("pos", "move to avoid2");
        moveTo2(point2, quaternion2, true);
        Log.d("pos", "move to point 2");

        //save debug image for point 2
        Bitmap image2 = api.getBitmapNavCam();
        api.saveBitmapImage(image2, "point2");

        //move to goal
        Point goal = new Point(11.27460f, -7.89178f, 4.96538f);
        moveTo2(avoid2, quaternion2, true);
        Log.d("pos", "move to avoid2");
        moveTo2(avoid, quaternion2, true);
        Log.d("pos", "move to avoid");
        moveTo2(goal, quaternion2, true);
        Log.d("pos", "move to goal");



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

    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        moveTo2(point, quaternion, true);
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        api.relativeMoveTo(point, quaternion, true);
    }

}