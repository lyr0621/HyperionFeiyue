package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry m_hortizontalAngle = m_limelight.getEntry("tx");
    private final NetworkTableEntry m_verticalAngle = m_limelight.getEntry("ty");


    public double limelightAngle() {
        return m_hortizontalAngle.getDouble(0);
    }

    public double get_distance(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

// distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 40.0;

// distance from the target to the floor
        double goalHeightInches = 97.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight angle", limelightAngle());
        SmartDashboard.putNumber("Limelight distance", get_distance());
    }
}

