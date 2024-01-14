package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OpenC {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}    
    public void main(String[] args){
        Mat mat = Mat.eye(3,3, CvType.CV_8UC1);
        System.out.println("Mat = " + mat.dump());
        
        int numCornersHor = 7;
        int numCornersVer = 7; 

        int numSquares = numCornersHor * numCornersVer;
        Size boardSize = new Size(numCornersHor,numCornersVer);

        //objt pts X Y Z (0,0,0), (1,0,0), (2,0,0) --> (CornersH -1 , Corners Vertical-1,0)
        MatOfPoint3f obj = new MatOfPoint3f();

        for(int i = 0; i< numSquares; i++){
           //error not registering the double array and stating Mat was not defined
            // obj.push_back(new MatOfPoint3f(new Mat(new double[]{i / numCornersHor, i % numCornersVer, 0.0f})));
        }
        List<MatOfPoint3f> objPts = new ArrayList<>(); //3d
        List<MatOfPoint2f> imgPts = new ArrayList<>();//2d

        // load callibration images/process them.

        //import image???
        for(String imagePath : ) {
            Mat image = Imgcodecs.imread(imagePath);
            Mat grayImage = new Mat();
            Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);

            MatOfPoint2f corners = new MatOfPoint2f();
            boolean found = Calib3d.findChessboardCorners(grayImage, boardSize, corners);

            if (found) {
                objPts.add(obj);
                imgPts.add(corners);

            }
            
        }

        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        Mat distCoeffs = new Mat();
        List <Mat> rvecs = new ArrayList<>();
        List <Mat> tvecs = new ArrayList<>();
        
    //Callibrate Camera
    //ERROR: grayImage is inside the foor loop, therefore it is causing troubles
        Calib3d.calibrateCamera(objPts, imgPts, grayImage.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

        //Extract parameters
        double[] cameraMatrixArray = new double[(int) (cameraMatrix.total() * cameraMatrix.channels())];
        cameraMatrix.get(0, 0, cameraMatrixArray);
        double[] distCoeffsArray = new double[(int) (distCoeffs.total() * distCoeffs.channels())];
        distCoeffs.get(0, 0, distCoeffsArray);

        // Camera matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        double fx = cameraMatrixArray[0];
        double fy = cameraMatrixArray[4];
        double cx = cameraMatrixArray[2];
        double cy = cameraMatrixArray[5];

        // Distortion coefficients
        System.out.println("Focal lengths: fx = " + fx + ", fy = " + fy);
        System.out.println("Principal Points: cx = " + cx + ", cy = " + cy);
        System.out.println("Distortion Coefficients: " + Arrays.toString(distCoeffsArray));
       

        
    }
}
