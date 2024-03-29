package frc.robot;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static Supplier<NetworkTable> limelightTable = () -> NetworkTableInstance.getDefault().getTable("limelight");
//FOR USE WITH getTableEntry():
//For more data, consult: https://docs.limelightvision.io/en/latest/networktables_api.html
/*
/////////////////////////////////////////
/////       BASIC TARGETING         /////
/////////////////////////////////////////
    tv 	        Whether the limelight has any valid targets (0 or 1)
    tx 	        Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    ty 	        Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    ta 	        Target Area (0% of image to 100% of image)
    tl 	        The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    tshort 	    Sidelength of shortest side of the fitted bounding box (pixels)
    tlong 	    Sidelength of longest side of the fitted bounding box (pixels)
    thor 	    Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    tvert 	    Vertical sidelength of the rough bounding box (0 - 320 pixels)
    getpipe 	True active pipeline index of the camera (0 .. 9)
    json 	    Full JSON dump of targeting results
    tclass 	    Class ID of primary neural detector result or neural classifier result 
MUST USE getDouble(0.0) TO BE USEFUL IN CODE
/////////////////////////////////////////
/////       3D AND APRILTAGS        /////
/////////////////////////////////////////
    botpose 	            Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
    botpose_wpiblue 	    Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
    botpose_wpired 	        Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
    camerapose_targetspace 	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
    targetpose_cameraspace 	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
    targetpose_robotspace 	3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
    botpose_targetspace 	3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
    tid 	                ID of the primary in-view AprilTag 
MUST USE getDoubleArray(new double[6]) TO BE USEFUL
*/

    public static NetworkTableEntry getTableEntry(String key){
        return limelightTable.get().getEntry(key);
    }

    /// To be used in automation ///

    //Is there a valid target? if so, then return true.
    public static boolean isValidTarget(){
        double dval = getTableEntry("tv").getDouble(0.0);
        if (dval == 1) return true;
        return false;
    }
    //Gets the Primary apriltag in view.
    public static double getPrimaryAprilTag(){
        return getTableEntry("tid").getDouble(0.0);
    }
    //BotPose in Field Space. Helpful for Filtering + averaging
    public static double[] getBotPose(){
        if(isValidTarget()) {
            double[] pose = getTableEntry("botpose").getDoubleArray(new double[6]);
            if (pose.length < 6) {
                pose = new double[]{0,0,0,0,0,0};
            }

            return pose;
        } else
            return new double[]{0,0,0,0,0,0};
    }

    // Pipeline map:
    // 0 - Generic apriltag detection
    // 1 - Retroflective mode
    public static void setLimelightPipeline(int pipeline){
        getTableEntry("pipeline").setNumber(pipeline);
    }

    public static void setRetroflective() {
        setLimelightPipeline(1);
    }

    public static void setAprilTag(double aprilTag) {
        setLimelightPipeline(0); // possibly use aprilTag param to set to specific pipeline for specific apriltag
    }
}