package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry m_hortizontalAngle = m_limelight.getEntry("tx");
    private final NetworkTableEntry m_verticalAngle = m_limelight.getEntry("ty");

    public double limelightAngle() {
        return m_hortizontalAngle.getDouble(0);
    }
}

