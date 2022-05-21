package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();

        // move to point 1
        Point point = new Point(10.71f, -7.7f, 4.48f);
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f); // TODO: adjust angle
        api.moveTo(point, quaternion, true);

        // report point1 arrival
        api.reportPoint1Arrival();

        // first target detection: TODO

        // irradiate the laser
        api.laserControl(true);

        // first debug cam
        Mat debug_point_1 = api.getMatNavCam();
        api.saveMatImage(debug_point_1, "point1");

        // take target1 snapshots
        api.takeTarget1Snapshot();

        // turn the laser off
        api.laserControl(false);

        // debug console
        Log.d("game", "Point 1 complete");

        //move to point 2
        Point point2 = new Point(11.2746f, -9.92284f,  5.29881f);
        Point avoid = new Point(11.3f, -8.2f, 4.48f);
        Point avoid2 = new Point(11.3f, -9.5f, 4.48f);
        Quaternion quaternion2 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(avoid, quaternion2, true);
        Log.d("movement", "move to avoid");
        api.moveTo(avoid2, quaternion2, true);
        Log.d("movement", "move to avoid 2");
        api.moveTo(point2, quaternion2, true);
        Log.d("movement", "move to point 2");

        // second target detection: TODO

        // irradiate the laser
        api.laserControl(true);

        // second debug cam
        Mat debug_point_2 = api.getMatNavCam();
        api.saveMatImage(debug_point_2, "point2");

        // take target2 snapshots
        api.takeTarget2Snapshot();

        // turn the laser off
        api.laserControl(false);

        // debug console
        Log.d("game", "Point 2 complete");

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
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        api.moveTo(point, quaternion, true);
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

