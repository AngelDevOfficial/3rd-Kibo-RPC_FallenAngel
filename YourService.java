package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.os.SystemClock;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Board;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

/* ******************************************** */
/* write your own code and repair the air leak! */
/* ******************************************** */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1(){

        //---------------------------------- variables -------------------------------------------//

        int dictID = Aruco.DICT_5X5_250;
        Mat ids;
        ArrayList<Mat> corners;
        Mat ids2;
        ArrayList<Mat> corners2;
        Dictionary dict;

        Mat camMatrix;
        Mat dstMatrix;
        double[] distortionArray;
        MatOfDouble distortion;
        List<Mat> objP1;
        MatOfInt board1ID;
        List<Mat> objP2;
        MatOfInt board2ID;
        Mat t1_rvec;
        Mat t1_tvec;
        Mat t2_rvec;
        Mat t2_tvec;
        Mat processedImg;
        Mat originalImg;
        Mat cropped_img;

        //----------------------------------------------------------------------------------------//

        // the mission starts
        api.startMission();

        //------------------------------ cam calibration -----------------------------------------//

        double[][] calibrate = api.getNavCamIntrinsics();
        distortionArray = calibrate[1];
        dstMatrix = new Mat(1, 5, CvType.CV_64FC1);
        dstMatrix.put(0, 0, distortionArray);
        camMatrix = new Mat(3, 3, CvType.CV_64FC1);
        camMatrix.put(0, 0, calibrate[0]);
        distortion = new MatOfDouble();
        distortion.fromArray(distortionArray);

        // set cam
//        double fx = 567.22931;
//        double cx = 659.07721;
//        double fy = 574.19293;
//        double cy = 517.00757;
//
//        distortionArray = new double[] { -0.21624701, 0.03875, -0.010157, 0.0019690001, 0 };
//
//        dstMatrix = new Mat(1, 5, CvType.CV_64FC1);
//        dstMatrix.put(0, 0, distortionArray);
//
//        camMatrix = new Mat(3, 3, CvType.CV_64FC1);
//        camMatrix.put(0, 0, new double[] { fx, 0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 });
//
//        distortion = new MatOfDouble();
//        distortion.fromArray(distortionArray);

        //----------------------------------------------------------------------------------------//

        //----------------------------- set target board -----------------------------------------//

        int[] id = new int[] { 1, 2, 3, 4 };
        board1ID = new MatOfInt();
        board1ID.fromArray(id);

        int[] id2 = new int[] { 11, 12, 13, 14 };
        board2ID = new MatOfInt();
        board2ID.fromArray(id2);

        objP1 = set_target_board1();
        objP2 = set_target_board2();

        //----------------------------------------------------------------------------------------//

        //------------------------------- point 1 ------------------------------------------------//

        Quaternion quaternion = eulerToQuaternion(-90, 90, -90);
        moveToWrapper(10.71f, -7.7f, 4.48f, quaternion);
        SystemClock.sleep(5000);

        // report point1 arrival and get image
        api.reportPoint1Arrival();
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "Target1.png");

        // find angle
        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Board t1_board = Board.create(objP1, dict, board1ID);
        Aruco.detectMarkers(image, dict, corners, ids);

        t1_rvec = new Mat();
        t1_tvec = new Mat();
        Aruco.estimatePoseBoard(corners, ids, t1_board, camMatrix, dstMatrix, t1_rvec, t1_tvec); //t1

        double tar_x = t1_tvec.get(0, 0)[0];
        double tar_y = t1_tvec.get(1, 0)[0];
        double tar_z = t1_tvec.get(2, 0)[0];

        Point3 tar_pos = new Point3(tar_x, tar_y, tar_z);

        double x_laser_offset = Math.atan(-0.0572 / (tar_pos.z + 0.1177)) * 180 / Math.PI;
        double y_laser_offset = Math.atan(-0.1111 / (tar_pos.z + 0.1177)) * 180 / Math.PI;
        double x_deg = Math.atan(-(tar_pos.x - 0.0422) / (tar_pos.z + 0.1177)) * 180 / Math.PI; // laser offset
        double y_deg = Math.atan(-(tar_pos.y - 0.0826) / (tar_pos.z + 0.1177)) * 180 / Math.PI;
        double horizon = x_laser_offset + x_deg - 4.2;
        double vertical = y_laser_offset + y_deg;

        quaternion = eulerToQuaternion(-90, 90 + horizon, -90 + vertical);
        moveToWrapper(10.71f, -7.7f, 4.48f, quaternion);
        SystemClock.sleep(5000);

        // irradiate the laser
        api.laserControl(true);
        // take target1 snapshots
        api.takeTarget1Snapshot();
        // turn the laser off
        api.laserControl(false);

        //----------------------------------------------------------------------------------------//

        //--------------------------------- point 2 ----------------------------------------------//

        // move to point 2
        Quaternion quaternion2 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveToWrapper(11.05f, -7.89178f, 4.5f, quaternion2);
        moveToWrapper(11.05f, -9.7f, 4.5f, quaternion2);
        moveToWrapper(11.27460f, -9.92284f, 5.29881f, quaternion2);
        SystemClock.sleep(5000);

        // get a camera image
        Mat image2 = api.getMatNavCam();
        api.saveMatImage(image2, "Target2.png");

        processedImg = new Mat(image2.rows(), image2.cols(), image2.type());
        originalImg = new Mat(image2.rows(), image2.cols(), image2.type());
        image2.copyTo(processedImg);
        image2.copyTo(originalImg);

        corners2 = new ArrayList<>();
        ids2 = new Mat();
        Board t2_board = Board.create(objP2, dict, board2ID);
        Aruco.detectMarkers(processedImg, dict, corners2, ids2);

        t2_rvec = new Mat();
        t2_tvec = new Mat();
        Aruco.estimatePoseBoard(corners2, ids2, t2_board, camMatrix, dstMatrix, t2_rvec, t2_tvec);
        List<MatOfPoint3f> offset = find_ROI3D(t2_rvec, t2_tvec);

        double tar_x2 = t2_tvec.get(0, 0)[0];
        double tar_y2 = t2_tvec.get(1, 0)[0];
        double tar_z2 = t2_tvec.get(2, 0)[0];

        Point3 tar_pos2 = new Point3(tar_x2, tar_y2, tar_z2);

        MatOfPoint3f _target3D = new MatOfPoint3f();
        _target3D.fromArray(tar_pos2);

        MatOfPoint2f _targetImagePlane = new MatOfPoint2f();
        Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
        Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);

        double[] _r = new double[] { 0.0f, 0.0f, 0.0f };
        double[] _t = new double[] { 0.0f, 0.0f, 0.0f };
        _rvec.put(0, 0, _r);
        _tvec.put(0, 0, _t);

        // find center of marker in 2D image
        Calib3d.projectPoints(_target3D, _rvec, _tvec, camMatrix, distortion, _targetImagePlane);

        List<double[]> ROI_points = new ArrayList<double[]>();

        for (int i = 0; i < 4; i++) {

            Calib3d.projectPoints(offset.get(i), _rvec, _tvec, camMatrix, distortion, _targetImagePlane);
            int _cpx = (int) _targetImagePlane.get(0, 0)[0];
            int _cpy = (int) _targetImagePlane.get(0, 0)[1];
            double[] _center = new double[] {_cpx, _cpy};
            ROI_points.add(_center);
        }

        // find ratio
//        double ratio = Math.abs(offset.get(2).get(0, 0)[1] / (ROI_points.get(2)[1] - 480));
        double avg_ar_size = 0;
        for (Mat corner : corners2) {
            avg_ar_size += Math.abs(corner.get(0, 0)[0] - corner.get(0, 1)[0]);
            avg_ar_size += Math.abs(corner.get(0, 2)[0] - corner.get(0, 3)[0]);
            avg_ar_size += Math.abs(corner.get(0, 0)[1] - corner.get(0, 3)[1]);
            avg_ar_size += Math.abs(corner.get(0, 1)[1] - corner.get(0, 2)[1]);
        }
        avg_ar_size /= 16;
		double ratio = 0.05 / avg_ar_size;

        // find Region of interest
        cropped_img = find_paper(originalImg, processedImg, ROI_points);
        // now we got warped_img , and cropped_img

        // find contour
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Mat cropped_gray = new Mat();
        Mat binaryImg = new Mat();

        if (cropped_img.channels() != 1) {
            Imgproc.cvtColor(cropped_img, cropped_gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(cropped_gray, binaryImg, 100, 200, Imgproc.THRESH_BINARY_INV);
        }
        else
            Imgproc.threshold(cropped_img, binaryImg, 100, 200, Imgproc.THRESH_BINARY_INV);

        Imgproc.findContours(binaryImg, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        double x_2 = 0.0;
        double y_2 = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            // Drawing Contours
            if (hierarchey.get(0, i)[2] == -1.0) {
                MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
                Moments moment = Imgproc.moments(ct2f);

                int x = (int) (moment.get_m10() / moment.get_m00());
                int y = (int) (moment.get_m01() / moment.get_m00());
                x_2 = (ROI_points.get(1)[0] + x - 640) * ratio;
                y_2 = (ROI_points.get(1)[1] + y - 480) * ratio;
            }
        }

        double x_laser_offset2 = Math.atan(-0.0572 / (tar_pos2.z - 0.1177)) * 180 / Math.PI;
        double y_laser_offset2 = Math.atan(-0.1111 / (tar_pos2.z - 0.1177)) * 180 / Math.PI;
        double x_deg2 = Math.atan((x_2 + tar_pos2.x - 0.0422) / (tar_pos2.z + 0.1177)) * 180 / Math.PI; // laser offset
        double y_deg2 = Math.atan(-(y_2 + tar_pos.y - 0.0826) / (tar_pos2.z + 0.1177)) * 180 / Math.PI;
        double horizon2 = x_laser_offset2 + x_deg2 - 0.24;
        double vertical2 = y_laser_offset2 + y_deg2 - 0.24;

        quaternion2 = eulerToQuaternion(-90 + horizon2, vertical2, 0);
        moveToWrapper(11.27460f, -9.92284f, 5.29881f, quaternion2);
        SystemClock.sleep(5000);

        // irradiate the laser
        api.laserControl(true);
//        // take target2 snapshots
        api.takeTarget2Snapshot();
        // turn the laser off
        api.laserControl(false);

        //---------------------- move to astronaut and complete ----------------------------------//

        quaternion2 = eulerToQuaternion(-90, 0, 0);
        moveToWrapper(10.6f, -9.92284f, 5.29881f, quaternion2);
        moveToWrapper(10.6f, -7.89178f, 5.29881f, quaternion2);
        moveToWrapper(11.27460f, -7.89178f, 4.96538f, quaternion2);
        api.reportMissionCompletion();

        //----------------------------------------------------------------------------------------//
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
    private void moveToWrapper(double pos_x, double pos_y, double pos_z, Quaternion quaternion){

        final Point point = new Point(pos_x, pos_y, pos_z);

        Result result = api.moveTo(point, quaternion, true);

        // check result and loop while moveTo api is not succeeded.
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 4) {
            // retry
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }

    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        api.relativeMoveTo(point, quaternion, true);
    }

    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree) {

        // yaw = radder +right -left, pitch = +takeoff-landing, roll = +right-left
        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    static double multiplyMatricesCell(double[][] firstMatrix, double[][] secondMatrix, int row, int col) {
        double cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[row][i] * secondMatrix[i][col];
        }
        return cell;
    }

    static double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }
        return result;
    }

    private static List<MatOfPoint3f> find_ROI3D(Mat rvec, Mat tvec) {

        // define paper_corner offset from its center
        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot); //rvec to rotation matrix

        double[][] offset_corner = { new double[] { 0.075f, -0.075f, -0.075f, 0.075f },
                new double[] { 0.0625f, 0.0625f, -0.0625f, -0.0625f }, new double[] { 0f, 0f, 0f, 0f }, };

        double[][] rotationMatrix = { new double[] { rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0] },
                new double[] { rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0] },
                new double[] { rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0] } };

        double[][] global_offset = multiplyMatrices(rotationMatrix, offset_corner);

        // c1------c0
        // | |
        // | |
        // c2------c3

        Point3 tar_c0 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][0],
                (double) tvec.get(1, 0)[0] + global_offset[1][0], (double) tvec.get(2, 0)[0] + global_offset[2][0]);

        Point3 tar_c1 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][1],
                (double) tvec.get(1, 0)[0] + global_offset[1][1], (double) tvec.get(2, 0)[0] + global_offset[2][1]);

        Point3 tar_c2 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][2],
                (double) tvec.get(1, 0)[0] + global_offset[1][2], (double) tvec.get(2, 0)[0] + global_offset[2][2]);

        Point3 tar_c3 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][3],
                (double) tvec.get(1, 0)[0] + global_offset[1][3], (double) tvec.get(2, 0)[0] + global_offset[2][3]);

        MatOfPoint3f offset_c0 = new MatOfPoint3f();
        offset_c0.fromArray(tar_c0);

        MatOfPoint3f offset_c1 = new MatOfPoint3f();
        offset_c1.fromArray(tar_c1);

        MatOfPoint3f offset_c2 = new MatOfPoint3f();
        offset_c2.fromArray(tar_c2);

        MatOfPoint3f offset_c3 = new MatOfPoint3f();
        offset_c3.fromArray(tar_c3);

        List<MatOfPoint3f> offset = new ArrayList<MatOfPoint3f>();
        offset.add(offset_c0);
        offset.add(offset_c1);
        offset.add(offset_c2);
        offset.add(offset_c3);
        return offset;
    }

    private static Mat find_paper(Mat original, Mat img, List<double[]> src_pts) {

//		img.copyTo(processedImg);
        List<Integer> list_x = new ArrayList<Integer>();
        List<Integer> list_y = new ArrayList<Integer>();

        for (int i = 0; i < 4; i++) {
//			Imgproc.circle(img, src_pts.get(i), 5, new Scalar(255, 0, 0), -1);
            list_x.add((int) src_pts.get(i)[0]);
            list_y.add((int) src_pts.get(i)[1]);
        }
        Collections.sort(list_x);
        Collections.sort(list_y);

        double max_w = list_x.get(3) - list_x.get(0);
        double max_h = list_y.get(3) - list_y.get(0);
//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3
//		MatOfPoint2f dst_pts = new MatOfPoint2f(new Point(max_w - 1, 0), new Point(0, 0), new Point(0, max_h - 1),
//				new Point(max_w - 1, max_h - 1));
//		MatOfPoint2f _pts = new MatOfPoint2f();
//		_pts.fromList(src_pts);
        return cropped_ROI(original, list_x.get(0), list_y.get(0), max_w, max_h);
    }

    private static Mat cropped_ROI(Mat img, double x, double y, double max_w, double max_h) {
        Rect target_rect = new Rect((int)x, (int)y, (int)max_w, (int)max_h);
        Mat cropped_img = img.submat(target_rect);
        return cropped_img;
    }

    private List<Mat> set_target_board1() {
        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.075f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0125f, 0.0f));
        c0.add(new Point3(0.075f, 0.0125f, 0.0f));

        c1.add(new Point3(-0.125f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0125f, 0.0f));
        c1.add(new Point3(-0.125f, 0.0125f, 0.0f));

        c2.add(new Point3(-0.125f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0625f, 0.0f));
        c2.add(new Point3(-0.125f, -0.0625f, 0.0f));

        c3.add(new Point3(0.075f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0625f, 0.0f));
        c3.add(new Point3(0.075f, -0.0625f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        List<Mat> objP1 = new ArrayList<Mat>();
        objP1.add(c_id0);
        objP1.add(c_id1);
        objP1.add(c_id2);
        objP1.add(c_id3);

        return objP1;
    }

    private List<Mat> set_target_board2() {

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
        c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

        c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
        c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

        c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
        c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

        c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
        c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        List<Mat> objP2 = new ArrayList<Mat>();
        objP2.add(c_id0);
        objP2.add(c_id1);
        objP2.add(c_id2);
        objP2.add(c_id3);

        return objP2;
    }
}

