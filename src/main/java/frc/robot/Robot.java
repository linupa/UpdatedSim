package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation. This
 * robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with the sim
 * GUI's {@link edu.wpi.first.wpilibj.simulation.Field2d} class.
 */

public class Robot extends TimedRobot {
  private Command m_teleopCommand;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
  private RobotContainer m_robotContainer;

  // private Spark leftMotor1 = new Spark(DriveConstants.kLeftMotor1Port);
  // private Spark leftMotor2 = new Spark(DriveConstants.kLeftMotor2Port);
  // private Spark rightMotor1 = new Spark(DriveConstants.kRightMotor1Port);
  // private Spark rightMotor2 = new Spark(DriveConstants.kRightMotor2Port);

  private Joystick joy1 = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every loop iteration.
    setNetworkTablesFlushEnabled(true);
    

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    DriveSubsystem.joystickMovement(joy1);
    m_teleopCommand = m_robotContainer.getCommand();
    System.out.println(m_teleopCommand.toString());

    // schedule the autonomous command (example)
    if (m_teleopCommand != null) {
      m_teleopCommand.schedule();
      //System.out.println("hi hi hi hi this works");
    }

  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroAllOutputs();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);


  }




}
