package frc.robot.subsystems.shooter.hoodIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.RobotMap;
import frc.robot.subsystems.shooter.ShooterConstants;

public class HoodIOSim implements HoodIO {
    private final PWMSim m_servo;
    private final PWMSim s_servo;

    private double _servoPos1 = 0;
    private double _servoPos2 = 0;
    private double _servoVolt1 = 0;
    private double _servoVolt2 = 0;

    public HoodIOSim() {
        m_servo = new PWMSim(RobotMap.PWM.kServo1);
        s_servo = new PWMSim(RobotMap.PWM.kServo2);
    }

    @Override
    public void updateInputs(HoodIOInputsAutoLogged inputs) {
        inputs.servo1Position = _servoPos1;
        inputs.servo2Position = _servoPos2;
        inputs.servo1Voltage = _servoVolt1;
        inputs.servo2Voltage = _servoVolt2;
    }

    @Override
    public void setServosPositions(double angle) {
        angle = MathUtil.clamp(angle, 0, ShooterConstants.Hood.kHoodMaxAngle);
        _servoPos1 = angle;
        _servoPos2 = ShooterConstants.Hood.kHoodMaxAngle - angle;
        var voltages = Hood.hoodAngleToVoltages(angle);
        _servoVolt1 = voltages[0];
        _servoVolt2 = voltages[1];

        m_servo.setPosition(_servoVolt1);
        s_servo.setPosition(_servoVolt2);
    }
}
