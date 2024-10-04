//package org.firstinspires.ftc.teamcode.Auton;
//
//import android.util.Size;
//
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
////import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
////import org.firstinspires.ftc.teamcode.oldfiles.Constants;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
////import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//public class VisionSubB {
//
//    private VisionPortal vp;
//    private AprilTagProcessor atp;
//    private TfodProcessor tf;
//    private VisionProcessor vproc;
//
//    //private float[]  f={0.1208f, -0.261599f, 0.0f, 0.0f, 0.10308f, 0.0f, 0.0f, 0.0f};
//
//    //private CameraIntrinsics camint=new CameraIntrinsics(633.9f, 633.9f,314.7f,244.3975f, f );
//
//    private ExposureControl expoCntrl;
//    private GainControl gainCntrl;
//    private WhiteBalanceControl wbCntrl;
//    private FocusControl focCntrl;
//    private PtzControl ptzCntrl;
//
//
//    private WebcamName webcam1, webcam2;
//    private CameraName switchableCamera;
//
//
//    private AprilTagProcessor.Builder atpBuilder = new AprilTagProcessor.Builder();
//    private TfodProcessor.Builder tfBuilder = new TfodProcessor.Builder();
//    private VisionPortal.Builder vpBuilder = new VisionPortal.Builder();
//
//    // private AprilTagDetection apDect;
//    // private List<AprilTagDetection> apDects;
//
//    private Recognition propRecognition;
//    private List<Recognition> currentRecognitions;
//
//    private double tagX, tagY, tagZ, tagCenterX, tagCenterY, tagYaw, rang, bearing, tfodX, tfodY;
//    private double tagLocX, tagLocY, tagLocZ, x, y, temp;
//    private boolean tagExists = false, twoCameras, redDetected, blueDetected;
//    private int currentCamera = 1, currentMode = 1, tagID;
//    private double[] tagLocArray = new double[2];
//    private double[] tagDeltaArray = new double[2];
//    private double deltaX, deltaY, confid;
//
//
//
//    private int propXRed, propYRed, propXBlue, propYBlue;
//
//    private String cmNme = "Webcam 1";
//
//
//    private String[] modLabelsProp = {"BLUE", "RED"};
//
//    private HardwareMap hwMap = null;
//
//
//    public VisionSubB(HardwareMap hwMap, boolean twoCameras) {
//
//
//
//        this.twoCameras = twoCameras;
//        this.hwMap = hwMap;
//        webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
//        if (twoCameras) webcam2 = hwMap.get(WebcamName.class, "Webcam 2");
//        if (twoCameras) {
//            webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
//            webcam2 = hwMap.get(WebcamName.class, "Webcam 2");
//            switchableCamera = ClassFactory.getInstance()
//                    .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
//
//        } else {
//            webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
//            webcam2 = hwMap.get(WebcamName.class, "Webcam 1");
//            switchableCamera = ClassFactory.getInstance()
//                    .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
//
//        }
//
//
//
//
//        atp = atpBuilder
//                .setDrawTagOutline(true)
//                .setDrawTagID(true)
//                .setDrawCubeProjection(false)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setLensIntrinsics(633.9, 633.9, 314.7, 244.3975)
//                .setDrawAxes(true)
//                .build();
//
//        atp.setDecimation(3);
//
//        tf = tfBuilder
//                .setIsModelTensorFlow2(true)
//              //  .setModelAssetName("model_20231029_005528.tflite")
//               // .setModelAssetName("model_20231102_021211.tflite")
//                .setModelAssetName("model_20231119_082921.tflite")
//                .setModelLabels(modLabelsProp)
//                //.setModelFileName("PowerPlay.tflite")
//                // .setModelAssetName()
//                //.setModelInputSize(300)
//                .setModelAspectRatio(4.0 / 3.0)
//                .setMaxNumRecognitions(3)
//                .setTrackerMinCorrelation(0.7f)
//                .build();
//
//        vp = vpBuilder
//                //.setCamera(webcam1)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                // .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
//                .setCamera(switchableCamera)
//                //.setLiveViewContainerId
//                .setCamera(webcam1)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setCameraResolution(new Size(640, 480))
//                .addProcessor(tf)
//                .addProcessor(atp)
//                .build();
//
//
//
//    }
//
//    public void createControls() {
//        expoCntrl = vp.getCameraControl(ExposureControl.class);
//
//        //expoCntrl.setMode(ExposureControl.Mode.Manual);
//        gainCntrl = vp.getCameraControl(GainControl.class);
//
//        wbCntrl=vp.getCameraControl(WhiteBalanceControl.class);
//
//        ptzCntrl=vp.getCameraControl(PtzControl.class);
//
//    }
//
//
//
//    public boolean isExposureSupported() {
//        return expoCntrl.isExposureSupported();
//    }
//
//    public int returnMaxGain() {
//        return gainCntrl.getMaxGain();
//    }
//
//    public int returnMinGain() {
//        return gainCntrl.getMinGain();
//    }
//
//    public void reinitVisionPortal() {
//
//        vp = vpBuilder
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .setCamera(switchableCamera)
//                //  .setLiveViewContainerId()
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setCameraResolution(new Size(640, 480))
//                .addProcessor(tf)
//                .addProcessor(atp)
//                .build();
//
//
//        vp.setActiveCamera(webcam1);
//
//
//        //PtzControl ptzcntrl= vp.getCameraControl(PtzControl.class);
//        //ptzcntrl.setZoom(200);
//
//
//    }
//
//
//
//    public void switchToCamera(int cam) {
//
//        if (!twoCameras) cam = 1;
//
//        if (cam == 1) {
//            vp.setActiveCamera(webcam1);
//            currentCamera = 1;
//        } else {
//            vp.setActiveCamera(webcam2);
//            currentCamera = 2;
//        }
//
//
//    }
//
//    public void setExposure(ExposureControl.Mode mode, long value) {
//        if (isCameraStreming()) {
//            expoCntrl.setMode(mode);
//            if (mode == ExposureControl.Mode.Manual){
//                expoCntrl.setExposure(value, TimeUnit.MILLISECONDS);
//            }
//        }
//    }
//
//    public  void setZoom(int zm){
//        ptzCntrl.setZoom(zm);
//    }
//
//    public int maxZoom(){
//        return  ptzCntrl.getMaxZoom();
//
//    }
//
//    public int minZoom(){
//        return  ptzCntrl.getMinZoom();
//    }
//
//    public void setFocus(FocusControl.Mode mode, double value) {
//        if (isCameraStreming()) {
//            focCntrl.setMode(mode);
//            if (mode == FocusControl.Mode.Fixed){
//                focCntrl.setFocusLength(value);
//            }
//        }
//    }
//
//
//    public void setWhiteBalance(WhiteBalanceControl.Mode mode, int value) {
//        if (isCameraStreming()) {
//            wbCntrl.setMode(mode);
//            if (mode == WhiteBalanceControl.Mode.MANUAL) {
//                wbCntrl.setWhiteBalanceTemperature(value);
//            }
//
//
//        }
//    }
//
//
//
//    public void setGain(int value) {
//        if (isCameraStreming()) {
//
//           // gainCntrl.setGain(255);
//        }
//    }
//
//
//
//    public boolean foundProp() {
//        currentRecognitions = tf.getFreshRecognitions();
//
//        redDetected = false;
//        blueDetected = false;
//
//        for (Recognition recognition : currentRecognitions) {
//            if (recognition.getLabel() == "RED") {
//                propXRed = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                propYRed = (int) ((recognition.getTop() + recognition.getBottom()) / 2);
//                redDetected = true;
//            } else if (recognition.getLabel() == "BLUE") {
//                propXBlue = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                propYBlue = (int) ((recognition.getTop() + recognition.getBottom()) / 2);
//                blueDetected = false;
//            }
//            if (redDetected || blueDetected) return true;
//        }
//
//        return (redDetected || blueDetected);
//    }
//
//    public boolean foundBLUE() {
//        List<Recognition> currentRecognitions = tf.getRecognitions();;
//        confid=0;
//        redDetected = false;
//        blueDetected = false;
//
//        if(currentRecognitions==null) return blueDetected;
//
//        for (Recognition recognition : currentRecognitions) {
//
//            if (recognition.getLabel() == "BLUE") {
//                propXBlue = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                propYBlue = (int) ((recognition.getTop() + recognition.getBottom()) / 2);
//                blueDetected = false;
//                if(recognition.getBottom()>confid){
//                    confid=recognition.getBottom();
//                    tfodX=propXBlue;
//                    tfodY=propYBlue;
//                    blueDetected=true;
//                }
//            }
//
//
//        }
//
//        return (blueDetected);
//    }
//
//    public boolean foundRED() {
//        List<Recognition> currentRecognitions = tf.getRecognitions();
//        confid=0;
//        redDetected = false;
//        blueDetected = false;
//
//        if(currentRecognitions==null) return redDetected;
//
//        for (Recognition recognition : currentRecognitions) {
//
//            if (recognition.getLabel() == "RED") {
//                propXRed = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                propYRed = (int) ((recognition.getTop() + recognition.getBottom()) / 2);
//                blueDetected = false;
//                if(recognition.getBottom()>confid){
//                    confid=recognition.getBottom();
//                    tfodX=propXRed;
//                    tfodY=propYRed;
//                    blueDetected=true;
//                }
//            }
//
//
//        }
//
//        return (blueDetected);
//    }
//
//    public double getTFODX(){
//       return tfodX;
//    }
//
//    public double getTFODY(){
//        return tfodY;
//    }
//
//    public boolean foundTag(int id) {
//
//
//        int tagindex = -1;
//        tagExists = false;
//
//
//
//        List<AprilTagDetection> apDects=atp.getDetections();
//
//
//
//
//       // apDects.addAll(atp.getDetections());
//
//        if (apDects == null) {
//            tagID = -1;
//            return tagExists;
//        }
//
//
//        for (AprilTagDetection Detection : apDects) {
//
//            tagindex = (Detection.id == id) ? id : -1;
//
//            if (tagindex > -1) {
//                tagExists = true;
//                tagID = Detection.id;
//                tagX = Detection.ftcPose.x;
//                tagY = Detection.ftcPose.y;
//                // tagZ = Detection.ftcPose.z;
//                tagCenterX = Detection.center.x;
//                tagCenterY = Detection.center.y;
//                tagYaw = Detection.ftcPose.yaw;
//                rang = Detection.ftcPose.range;
//                bearing = Detection.ftcPose.bearing;
//                tagLocX = Detection.metadata.fieldPosition.get(0);
//                tagLocY = Detection.metadata.fieldPosition.get(1);
//                // tagLocZ=Detection.metadata.fieldPosition.get(2)
//
//            }
//            if (tagExists) break;
//        }
//
//
//        //apDects.clear();
//        return tagExists;
//    }
//
//    public boolean isCameraOpen() {
//        return vp.getCameraState() == VisionPortal.CameraState.OPENING_CAMERA_DEVICE;
//    }
//
//    public boolean isCameraStreming() {
//        return vp.getCameraState() == VisionPortal.CameraState.STREAMING;
//    }
//
//    public void setAutoStopLiveView(boolean truefalse) {
//        vpBuilder.setAutoStopLiveView(truefalse);
//    }
//
//    public void enableAprilTagProcessor(boolean truefalse) {
//        vp.setProcessorEnabled(atp, truefalse);
//    }
//
//    public void enableTFODProcessor(boolean truefalse) {
//        vp.setProcessorEnabled(tf, truefalse);
//    }
//
//    public void enableVisionProcessor(boolean truefalse) {
//        vp.setProcessorEnabled(vproc, truefalse);
//    }
//
//    public void closeVisionPortal() {
//        vp.close();
//    }
//
//    public VisionPortal.CameraState getVisionPortalState() {
//        return vp.getCameraState();
//    }
//
//    public boolean isVisionPortalClose() {
//        return vp.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED;
//    }
//
//    public int currentCamera() {
//        return currentCamera;
//    }
//
//    public boolean tagFound() {
//        return tagExists;
//    }
//
//    public Vector2d tagVEC() {
//        if (foundTag(tagID)) {
//
//            if (tagID < 7) {
//                return (new Vector2d(tagLocX - rang * Math.cos(bearing), tagLocY + rang * Math.sin(bearing)));
//            } else {
//                return (new Vector2d(tagLocX + rang * Math.cos(bearing), tagLocY - rang * Math.sin(bearing)));
//            }
//
//        } else {
//            return (new Vector2d(-999, -999));
//        }
//
//    }
//
//    public double getTagX() {
//        if (tagExists) {
//            return tagX;
//        } else {
//            return -999;
//        }
//    }
//
//    public double getTagY() {
//        if (tagExists) {
//            return tagY;
//        } else {
//            return -999;
//        }
//    }
//
//    public double getTagZ() {
//        if (tagExists) {
//            return tagZ;
//        } else {
//            return -999;
//        }
//    }
//
//    public double getTagYaw() {
//        if (tagExists) {
//            return tagYaw;
//        } else {
//            return -999;
//        }
//    }
//
//    public double[] getTagLoc() {
//        if (tagExists) {
//            tagLocArray[0] = tagLocX;
//            tagLocArray[1] = tagLocY;
//            return tagLocArray;
//        } else {
//            tagLocArray[0] = -999;
//            tagLocArray[1] = -999;
//            return tagLocArray;
//        }
//    }
//
//    /*
//        public Vector2d getBotLoc(double WheelBase){
//            if(tagExists){
//
//                // when facing tags 1 thru 6  x on the bot is -ve y on the field so -3 mean you are 3 further to the red side
//
//                // -ve yaw is the angle of the bot  for the bot -tagYaw is just the actual angle of the bot can update the gyro with it
//                if(tagID<7) {
//                     x = tagY * Math.cos(tagYaw) - tagX * Math.sin(tagYaw) + WheelBase / 2 * Math.cos(-tagYaw);
//                     y = tagY * Math.sin(tagYaw) + tagX * Math.cos(tagYaw) - WheelBase / 2 * Math.sin(-tagYaw);
//                    return new Vector2d(tagLocX-x,tagLocY+y);
//                }else{
//                     x = -tagY * Math.cos(tagYaw) + tagX * Math.sin(tagYaw) + WheelBase / 2 * Math.cos(-tagYaw);
//                     y = -tagY * Math.sin(tagYaw) - tagX * Math.cos(tagYaw) + WheelBase / 2 * Math.sin(-tagYaw);
//                    return new Vector2d(tagLocX-x,tagLocY+y);
//                }
//
//               // return new Vector2d(x,y);
//            }else{
//                return new Vector2d(-999,-999);
//            }
//
//
//        }
//    */
////    public Vector2d getBotLoc(double WheelBase) {
////        if (tagExists) {
////
////            // when facing tags 1 thru 6  x on the bot is -ve y on the field so -3 mean you are 3 further to the red side
////
////            /// first assume you are perpendicular and then rotate by the yaw  which -veyaw for the robot
////
////            // -ve yaw is the angle of the bot  for the bot -tagYaw is just the actual angle of the bot can update the gyro with it
////            if (tagID < 7) {
////
////                x = -tagY - WheelBase / 2;
////
////                if(currentCamera==1){
////                    y = tagX- Constants.Camera.midtoCamY1;
////                }else{
////                    y= tagX+Constants.Camera.midtoCamY2;
////                }
////
////                temp = x * Math.cos(-tagYaw) - y * Math.sin(-tagYaw);
////                y = x * Math.sin(-tagYaw) + y * Math.cos(-tagYaw);
////
////                return new Vector2d(tagLocX + temp, tagLocY + y);
////            } else {
////
////                x = tagY + WheelBase / 2;
////                y = -tagX;
////
////
////                if(currentCamera==2){
////                    y = -tagX-Constants.Camera.midtoCamY1;
////                }else{
////                    y= -tagX+Constants.Camera.midtoCamY2;
////                }
////
////
////                temp = x * Math.cos(-tagYaw) - y * Math.sin(-tagYaw);
////                y = x * Math.sin(-tagYaw) + y * Math.cos(-tagYaw);
////
////                return new Vector2d(tagLocX + temp, tagLocY + y);
////            }
////
////            // return new Vector2d(x,y);
////        } else {
////            return new Vector2d(-999, -999);
////        }
////
////
////    }
////
////    public Vector2d getBotDeltaLoc(double WheelBase) {
////        if (tagExists) {
////
////            // when facing tags 1 thru 6  x on the bot is -ve y on the field so -3 mean you are 3 further to the red side
////
////            /// first assume you are perpendicular and then rotate by the yaw  which -veyaw for the robot
////
////            // -ve yaw is the angle of the bot  for the bot -tagYaw is just the actual angle of the bot can update the gyro with it
////            if (tagID < 7) {
////
////                if (currentCamera == 1){
////                    x = -tagY - WheelBase / 2;
////                y = tagX + ConstantsB.Camera.midtoCamY1;
////            }else{
////                    x = -tagY - WheelBase / 2;
////                    y = tagX-ConstantsB.Camera.midtoCamY2;
////                }
////
////                temp = x * Math.cos(-tagYaw) - y * Math.sin(-tagYaw);
////                y = x * Math.sin(-tagYaw) + y * Math.cos(-tagYaw);
////
////                return new Vector2d(temp, - y);
////            } else {
////
////                x = tagY + WheelBase / 2;
////                y = -tagX;
////
////
////                temp = x * Math.cos(-tagYaw) - y * Math.sin(-tagYaw);
////                y = x * Math.sin(-tagYaw) + y * Math.cos(-tagYaw);
////
////                return new Vector2d(temp, - y);
////            }
////
////            // return new Vector2d(x,y);
////        } else {
////            return new Vector2d(-999, -999);
////        }
////
////
////    }
//
//    public double getBotHeading() {
//        if (tagExists) {
//            if (tagID < 7) {
//                return -tagYaw;
//            } else {
//                return Math.PI - tagYaw;
//            }
//        } else {
//            return -999;
//        }
//
//    }
//
//
//    public double getTagRange() {
//        if (tagExists) {
//            return rang;
//        } else {
//            return -999;
//        }
//    }
//
//    public double getTagBearing() {
//        if (tagExists) {
//            return getTagBearing();
//        } else {
//            return -999;
//        }
//    }
//
//    public boolean getTFdetection() {
//
//        return (redDetected || blueDetected);
//    }
//
//
//}
