package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class Shooter extends SubsystemBase {
    private TalonFX m_left;
    private TalonFX m_right;
    private TalonFX m_kicker;
    private ShooterLookupTable m_lookuptable;


    private SimpleMotorFeedforward m_ff;

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPM", 2300);

        m_left = new TalonFX(21);
        m_right = new TalonFX(22);
        m_kicker = new TalonFX(18);


        m_lookuptable = new ShooterLookupTable();

        m_right.configFactoryDefault();
        m_left.configFactoryDefault();
        m_kicker.configFactoryDefault();

        m_left.follow(m_right);

        m_right.configVoltageCompSaturation(12.0);
        m_right.enableVoltageCompensation(true);

        m_right.setInverted(InvertType.InvertMotorOutput);
        m_kicker.setInverted(true);

        //0.2574
        //m_right.config_kP(0, 0.35);
        //m_right.config_kD(0,0.1);
        m_right.config_kP(0, 0.3);
        m_right.config_kD(0,0.3);
        //m_right.config_kI(0,0.001);

        //m_ff = new SimpleMotorFeedforward(0.090, 0.228, 0.0);
        m_ff = new SimpleMotorFeedforward(0.18, 0.00025, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Recorded RPM", Utils.falconToRPM(m_left.getSelectedSensorVelocity(), 1.0));
        // SmartDashboard.putNumber("Desired RPM", SmartDashboard.getNumber("Shooter RPM", m_lookuptable.getVelocity()));

    }


    public double get_shooter_rpm(){
        return Utils.falconToRPM(m_left.getSelectedSensorVelocity(), 1.0);
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
        double output = SmartDashboard.getNumber("Shooter RPM", speed);
        double arbOutput = m_ff.calculate(speed);
//        arbOutput = 0.0;

        m_right.set(ControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
        m_left.set(TalonFXControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
    }

    public void runFlywheel() {

        double output = SmartDashboard.getNumber("Shooter RPM", 3000);
        double arbOutput = m_ff.calculate(3000);
//        arbOutput = 0.0;

        m_right.set(ControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
        m_left.set(TalonFXControlMode.Velocity, Utils.RPMToFalcon(output, 1.0), DemandType.ArbitraryFeedForward, arbOutput);
    }

    public void runKicker() {
        m_kicker.set(TalonFXControlMode.PercentOutput, 1.0);
    }

    public void shoot() {
        runKicker();
        runFlywheel(2300);
    }

    public void stop() {
        m_left.set(TalonFXControlMode.PercentOutput, 0.0);
        m_right.set(TalonFXControlMode.PercentOutput, 0.0);
        m_kicker.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public double calcRpmForDistance(double distance) {
        return (0.051 * Math.pow(distance, 2)) - (7.834 * distance) + 3749.589;
    }
}
