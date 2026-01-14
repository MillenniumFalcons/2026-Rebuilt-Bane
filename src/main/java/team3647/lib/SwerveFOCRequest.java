package team3647.lib;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SwerveFOCRequest implements SwerveRequest {
    private final MotionMagicExpoVoltage m_motionMagicControl =
            new MotionMagicExpoVoltage(Degree.of(0))
                    .withEnableFOC(false)
                    .withFeedForward(0)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false)
                    .withOverrideBrakeDurNeutral(false)
                    .withPosition(Degree.of(0))
                    .withSlot(0);
    // new MotionMagicVoltage(0, false, 0, 0, false, false, false);
    private final TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private double m_targetTorque = 0.0;
    private boolean m_driveType = true;

    public SwerveFOCRequest(boolean driveType) {
        m_driveType = driveType;
    }

    public SwerveFOCRequest() {
        m_driveType = true;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        for (var module : modulesToApply) {
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl);

                // Command drive motor to torque
                module.getDriveMotor()
                        .setControl(
                                m_torqueCurrentFOC
                                        .withOutput(m_targetTorque)
                                        .withMaxAbsDutyCycle(0.7));
            } else {
                // Command steer motor to torque
                module.getSteerMotor()
                        .setControl(
                                m_torqueCurrentFOC
                                        .withOutput(m_targetTorque)
                                        .withMaxAbsDutyCycle(0.7));

                // Command drive motor to zero
                module.getDriveMotor().setControl(m_motionMagicControl);
            }
        }

        return StatusCode.OK;
    }

    /**
     * @param targetTorque Torque for all modules to target
     * @return
     */
    public SwerveFOCRequest withVoltage(double targetTorque) {
        this.m_targetTorque = targetTorque;
        return this;
    }
}
