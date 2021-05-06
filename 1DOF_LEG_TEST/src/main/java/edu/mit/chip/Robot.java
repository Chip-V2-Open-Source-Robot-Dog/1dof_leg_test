/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.utils.Networking;
import edu.mit.chip.utils.PIDConstants;
import edu.mit.chip.utils.DS;

import edu.wpi.first.wpilibj.TimedRobot;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    //GET CONTROL TYPE ENUM
    public enum MotorControlType {
        VOLTAGE(ControlType.kVoltage),
        CURRENT(ControlType.kCurrent),
        VELOCITY(ControlType.kVelocity),
        POSITION(ControlType.kPosition),
        SMARTMOTION(ControlType.kSmartMotion);

        public ControlType sparkMaxType;

        private MotorControlType(ControlType sparkMaxType) {
            this.sparkMaxType = sparkMaxType;
        }
    }

    /*
    FIRST LETS CREATE ALL THE KPS AND ETC
    */
    //MOTOR TORQUE CONSTANT
    //https://motors.vex.com/other-motors/neo#narp186
    private final double KT = 0.018121547; //calcualted from torque-speed curve
    private final double R = 10.0; //10:1 gearbox
    //COMMON
    private final double kIz = 0;
    private final double kFF = 0;
    private final double maxRPM = 5700;
    //SHOULDER CONSTANTS
    private final double kP = 0.2;
    private final double kI = 0.00001;
    private final double kD = 0.4;
    private final double kMaxOutput = 0.14;
    private final double kMinOutput = -0.14;

    private Networking networking;
    // private Thread networkingThread;

    private DS DS = new DS();

    //Create SPARKMAX Objects
    private CANSparkMax motor_1, motor_2;
    /**
     * This function is run when the robot code is first started up (or restarted).
     */
    @Override
    public void robotInit() {
        System.out.println("Initializing robot...");
        System.out.println("Attempting to construct CAN SparkMAXs.");

        //CREATE SPARKMAXS AND DEAL WITH REVERSALS
        motor_1 =  new CANSparkMax(3, MotorType.kBrushless);
        motor_2 =     new CANSparkMax(10, MotorType.kBrushless);

        System.out.println("Controllers constructed.");

        //CREATE PID CONSTANTS OBJECT
        PIDConstants motorPID = new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM);
        //actually set the PID constants
        motorPID.load(motor_1.getPIDController());
        motorPID.load(motor_2.getPIDController());

        //=====================================================//
        //                                                     //
        //                REST OF ROBOT SETUP                  //
        //                                                     //
        //=====================================================//



        System.out.println("Robot initialized.");

        //DO NETWORKING SETUP
        networking = Networking.getInstance();
        networking.addReadouts(
            //BUS VOLTAGE
            "VBUS",
            //GENERAL MOTOR PARAMS
            "m1_p", "m2_p", "m1_current", "m2_current", 
            //ADDING IN VELOCITIES FOR P-PDOT CONTROL
            "m1_v", "m2_v",
            //ADDING IN TORQUES
            "m1_tau", "m2_tau"
        );
        networking.addInputs(
            "m1_p",
            "m2_p",
            "m1_pd",
            "m2_pd",
            "m1_tau",
            "m2_tau",
            "m1_mode",
            "m2_mode"
        );

        //ENABLE THE ROBOT WITH DS
        DS.start();
    }
    
    /**
    * This function is called every robot packet, no matter the mode.
    */
    @Override
    public void robotPeriodic() {
        //PERIODICALLY PUBLISH VALUES
        //MOTOR1
        networking.pushReadout("m1_p", motor_1.getEncoder().getPosition());
        networking.pushReadout("m1_v", motor_1.getEncoder().getVelocity());
        networking.pushReadout("m1_current", motor_1.getOutputCurrent());
        networking.pushReadout("m1_tau", motor_1.getOutputCurrent()*KT*R);
        
        //MOTOR2
        networking.pushReadout("m2_p", motor_2.getEncoder().getPosition());
        networking.pushReadout("m2_v", motor_2.getEncoder().getVelocity());
        networking.pushReadout("m2_current", motor_2.getOutputCurrent());
        networking.pushReadout("m2_tau", motor_2.getOutputCurrent()*KT*R);

        //CALCULATE AVERAGE BUS VOLTAGE
        double AVG_VOLTAGE = (motor_1.getBusVoltage()+motor_2.getBusVoltage())/2.0;
        //PUSH AVERAGE BUS VOLTAGE
        networking.pushReadout("VBUS", AVG_VOLTAGE);
    }
    
    /**
    * This function is called at the very beginning of the teleoperated period.
    */
    @Override
    public void teleopInit() {
        //SET ALL THE LEGS TO ZERO POSITION 
        motor_1.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        motor_2.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {

        //=====================================================//
        //                                                     //
        //                SET ALL JOINT SETPOINTS              //
        //                                                     //
        //=====================================================//


        //SET THE VALUES TO WHAT IS PULLED FROM NETWORK TABLES --> POSITION + SMART MOTION CONTROL!!!
        motor_1.getPIDController().setReference(networking.pullInput("m1_p", 0.0), MotorControlType.POSITION.sparkMaxType); //UPDATE THE MOTOR CONTROL STRUCT ABOVE LATER
        motor_2.getPIDController().setReference(networking.pullInput("m2_p", 0.0), MotorControlType.POSITION.sparkMaxType);
    }
    
    /**
    * This fetches which SetupAction the user has selected and places it in the 
    * CommandQueue.
    */
    @Override
    public void autonomousInit() {
        //DO NOTHING
    }
    
    /**
    * This function is called periodically during autonomous. We use it to run
    * SetupAction commands.
    */
    @Override
    public void autonomousPeriodic() {
        //DO NOTHING
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        //SET ALL THE MOTORS TO NEUTRAL/VOLTAGE CONTROL and 0 VOLTS
        motor_1.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        motor_2.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
    }
}
