package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

@Deprecated
public class Shooter extends SubsystemBase {
    protected final TalonFX m_left_follower;
    protected final TalonFX m_right_master;
    protected final TalonFX m_kicker;
    protected final ShooterLookupTable m_lookuptable;
    protected final double defaultSpeed = 3000;

    private SimpleMotorFeedforward m_ff;

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPM", 3000);

        m_left_follower = new TalonFX(21);
        m_right_master = new TalonFX(22);
        m_kicker = new TalonFX(18);


        m_lookuptable = new ShooterLookupTable();

        m_right_master.configFactoryDefault();
        m_left_follower.configFactoryDefault();
        m_kicker.configFactoryDefault();

        m_left_follower.follow(m_right_master);

        m_right_master.configVoltageCompSaturation(12.0);
        //m_left.configVoltageCompSaturation(12.0);
        m_right_master.enableVoltageCompensation(true);
        //m_left.enableVoltageCompensation(true);

        m_right_master.setInverted(InvertType.InvertMotorOutput);
        //m_left.setInverted(InvertType.InvertMotorOutput);
        m_kicker.setInverted(true);

        //0.2574
        //m_right.config_kP(0, 0.35);
        //m_right.config_kD(0,0.1);
        // m_right_master.config_kP(0, 0.1);
        //m_left.config_kP(0, 0.1);
        // m_right_master.config_kD(0,0.05);
        //m_left.config_kP(0, 0.1);
        //m_right.config_kI(0,0.001);

        //m_ff = new SimpleMotorFeedforward(0.090, 0.228, 0.0);
        // m_ff = new SimpleMotorFeedforward(0.3, 0.0, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Recorded RPM Left", Utils.falconToRPM(m_left_follower.getSelectedSensorVelocity(), 1.0));
        SmartDashboard.putNumber("Recorded RPM Right", Utils.falconToRPM(m_right_master.getSelectedSensorVelocity(), 1.0));
        // SmartDashboard.putNumber("Desired RPM", SmartDashboard.getNumber("Shooter RPM", m_lookuptable.getVelocity()));

    }


    public double get_shooter_rpm(){
        return Utils.falconToRPM(m_right_master.getSelectedSensorVelocity(), 1.0);
    }

    public double get_target_rpm(double distance){
        return m_lookuptable.getVelocity(distance);
    }

    public void shoot_from_anywhere(double distance){
        double rpm = get_target_rpm(distance);
        runFlywheel(rpm);
    }

    public void runFlywheel(double speed) {

        /*
        lime_light distance: |  2800  |
        dashboard distance:  |   71   |
         */
        SmartDashboard.putNumber("Shooter RPM", speed);
        double output = speed;
        // SmartDashboard.getNumber("Shooter RPM", speed);
        double arbOutput = m_ff.calculate(speed);
//        arbOutput = 0.0;

        m_right_master.set(TalonFXControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
        //m_left.set(TalonFXControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
    }

    public void runFlywheel() {
        runFlywheel(defaultSpeed);
    }

    public void runKicker() {
        m_kicker.set(TalonFXControlMode.PercentOutput, 1.0);
    }
    public void shoot() {
        runKicker();
        runFlywheel(defaultSpeed);
    }

    public void stop() {
       // m_left.set(TalonFXControlMode.PercentOutput, 0.0);
        m_right_master.set(TalonFXControlMode.PercentOutput, 0.0);
        m_kicker.set(TalonFXControlMode.PercentOutput, 0.0);
    }
    @Deprecated
    public double calcRpmForDistance(double distance) {
        return (0.051 * Math.pow(distance, 2)) - (7.834 * distance) + 3749.589;
    }
}
