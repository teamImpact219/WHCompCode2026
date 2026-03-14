//base imports
package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//subsystem imports
import frc.robot.subsystems.UniCamVisionSubsystem;
//util imports
import java.util.ArrayList;

public class MultiCamVisionSubsytem extends SubsystemBase {
  //variables
    //subsystems
    private ArrayList<UniCamVisionSubsystem> uniCams = new ArrayList<UniCamVisionSubsystem>();
  
  //constructor
  public MultiCamVisionSubsytem(ArrayList<String> camNames, ArrayList<Transform3d> positionRelToBot) {
    for (int i = 0; i < camNames.size(); i++){
      uniCams.add(new UniCamVisionSubsystem(camNames.get(i), positionRelToBot.get(i)));
    }
  }

  //gets all of the unicam vision subsystems
  public ArrayList<UniCamVisionSubsystem> getVisionSubsystems(){
    return uniCams;
  }

  @Override
  public void periodic() {}
}
 