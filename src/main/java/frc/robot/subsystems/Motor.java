package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase{
    
    private SparkMax m_sparkMax;
    private SparkFlex m_sparkFlex;  
    private final CANcoder encoder; 

    private double kp=0,ki=0,kd=0; 
    private PIDController c_pidController = new PIDController(kp, ki, kd); 

    private double speed; 
    private double target; 
    private double positions[] = new double[50];
    private double tolerance; 

    public Motor(SparkMax m_sparkMax, CANcoder encoder, double target, double tolerance) {
        this.m_sparkMax = m_sparkMax; 
        this.encoder = encoder; 

        this.target = target; 
        this.tolerance = tolerance; 

        c_pidController.setTolerance(this.tolerance);
    }

    public Motor(SparkFlex m_sparkFlex, CANcoder encoder, double target, double tolerance) {
        this.m_sparkFlex = m_sparkFlex; 
        this.encoder = encoder;
        
        this.target = target; 
        this.tolerance = tolerance; 

        c_pidController.setTolerance(this.tolerance);
    }

    @Override 
    public void periodic() {
        speed = c_pidController.calculate(get_position());

        m_sparkMax.set(speed); 

        if (get_position() < target-tolerance) {
            kp+=0.01; 
            set_PID(); 
        } else if (get_position() > target+tolerance) {
            kp-=0.01; 
            set_PID();
        }
    }

    public void set_setpoint() {
        c_pidController.setSetpoint(250);
    }

    public boolean at_setpoint() {
        return c_pidController.atSetpoint();
    }

    public double get_position() {
        return Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    }

    public void set_PID() {
        c_pidController.setPID(kp, ki, kd);
    }
}
