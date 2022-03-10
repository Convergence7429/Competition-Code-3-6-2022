package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {

    public void climberInit() {
    }

    public void climberTeleop() {

        // System.out.println(Robot.solenoid0.get());
        // System.out.println(Robot.solenoid1.get());
        // System.out.println("*******************************");

        if(Constants.xbox.getRawButtonPressed(7)){
            Robot.solenoid0.set(false);
            Robot.solenoid1.set(true);
            // if(solenoid0.get()){
            //   solenoid0.set(false);
            //   solenoid1.set(true);
            // } else if(solenoid1.get()){
            //   solenoid1.set(false);
            //   solenoid0.set(true);
            // }
            //solenoid1.set(true);
            //doubleSolenoid.toggle();
          }
      
          if(Constants.xbox.getRawButtonPressed(8)){
            Robot.solenoid1.set(false);
            Robot.solenoid0.set(true);
          }
    }
}