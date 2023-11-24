//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib;

public class SwerveDriveTrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int m_pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String m_canBusName = k.ROBOT.CANVORE_CANFD_NAME;

    public double m_rotateKp = 0;
    public double m_rotateKi = 0;
    public double m_rotateKd = 0;

    public SwerveDriveTrainConstants withPigeon2Id(int _id) {
        this.m_pigeon2Id = _id;
        return this;
    }

    public SwerveDriveTrainConstants withCANbusName(String _name) {
        this.m_canBusName = _name;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKp(double _turnKp) {
        this.m_rotateKp = _turnKp;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKi(double _turnKi) {
        this.m_rotateKi = _turnKi;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKd(double _turnKd) {
        this.m_rotateKd = _turnKd;
        return this;
    }
}
