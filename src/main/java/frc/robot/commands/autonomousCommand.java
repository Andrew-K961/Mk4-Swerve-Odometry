package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Trajectory following */
public class autonomousCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem m_subsystem;
  private PathPlannerTrajectory trajectory;
  private HolonomicDriveController controller;
  private final Timer timer = new Timer();

  public autonomousCommand(DrivetrainSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath("Test", 5, 3);

    ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(new PIDController(1, 1, 1), new PIDController(1, 1, 1),
        thetaController);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    State desiredState = trajectory.sample(timer.get());

    ChassisSpeeds targetChassisSpeeds = controller.calculate(m_subsystem.getPose(), desiredState,
        desiredState.poseMeters.getRotation());

    m_subsystem.drive(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(new ChassisSpeeds(0,0,0));
  }

  @Override
  public boolean isFinished() {
    if (timer.get() <= trajectory.getTotalTimeSeconds()){
      return false;
    } else {
      return true;
    }
  }
}