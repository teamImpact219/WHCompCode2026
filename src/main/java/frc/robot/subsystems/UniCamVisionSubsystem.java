//base imports
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//camera imports
import org.photonvision.PhotonCamera;
//util imports
import java.util.ArrayList;
import java.util.Optional;
//geometric imports
import edu.wpi.first.math.geometry.Transform3d;
//field/position estimation imports
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class UniCamVisionSubsystem extends SubsystemBase {
  //variables
    //camera variables
    private PhotonCamera cam;
    private String camName;
    private Transform3d camPosRelBotCenter;
    //field/pose estimation variables
    public static final AprilTagFieldLayout apriltagPositions = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public final PhotonPoseEstimator poseEstimatorInator =
      new PhotonPoseEstimator(apriltagPositions, camPosRelBotCenter);  
 
  //constructors
  public UniCamVisionSubsystem(String name, Transform3d camPosRelBotCenter) {
    //creating the camera
    cam = new PhotonCamera(name);
    camName = name;
    //camera position data 
    this.camPosRelBotCenter = camPosRelBotCenter;
  }

  /*
   * finds all the pipeline results with actual tag detections in them
   * order of results: oldest to newest
   */
  public ArrayList<PhotonPipelineResult> getNewPiplineResults(){
    ArrayList<PhotonPipelineResult> results = new ArrayList<PhotonPipelineResult>();
    for (PhotonPipelineResult r: cam.getAllUnreadResults()){
      if (r.hasTargets()){
        results.add(r);
      }
    }
    return results;
  }

  /*
   * gets a field position estimate based on mutli and singular tag methods
   * multitag first, lowest ambiguity second
   */
  public Optional<EstimatedRobotPose> getFieldPoseEstimate(PhotonPipelineResult pipelineData){
    //tries a multitag solution
     Optional<EstimatedRobotPose> estimate = poseEstimatorInator.estimateCoprocMultiTagPose(pipelineData);
    if (estimate.isEmpty()){
      //fall back on single
      estimate = poseEstimatorInator.estimateLowestAmbiguityPose(pipelineData);
    }
    return estimate;
  }

  /*
   * finds a tag in the pipelien result
   * potential null pointer return :(
  */
  public PhotonTrackedTarget findTag(int id, PhotonPipelineResult PPR){
    for (PhotonTrackedTarget PTT: PPR.getTargets()){
      if (PTT.getFiducialId()==id){
        return PTT;
      }
    }
    return null;
  }

  @Override
  public void periodic() {}
}
