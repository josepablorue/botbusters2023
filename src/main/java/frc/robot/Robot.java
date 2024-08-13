// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.channels.AsynchronousServerSocketChannel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // 4 motores chasis
  CANSparkMax chasisDerecho1 = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax chasisDerecho2 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax chasisIzquierdo1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax chasisIzquierdo2 = new CANSparkMax(3, MotorType.kBrushless);
  DifferentialDrive chasis = new DifferentialDrive(chasisIzquierdo1, chasisDerecho1);
  // Encoders del chasis
  RelativeEncoder encoderChasisDerecho;
  RelativeEncoder encoderChasisIzquierdo;
  double encodersChasis[];

  // limit switches
  DigitalInput switchIntake = new DigitalInput(0);
  DigitalInput switchPivoteoArriba = new DigitalInput(1);
  DigitalInput switchPivoteoAbajo = new DigitalInput(2);

  // Motores intake
  CANSparkMax intake1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax intake2 = new CANSparkMax(6, MotorType.kBrushless);
  // Encoder intake
  RelativeEncoder encoderIntake = intake1.getEncoder();

  // Motores shooter
  CANSparkMax shooter1 = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(8, MotorType.kBrushless);

  // Variable para condicional del pivoteo
  boolean pivoteo;

  // Motores pivoteo
  CANSparkMax pivoteo1 = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax pivoteo2 = new CANSparkMax(10, MotorType.kBrushless);
  // Encoder pivoteo
  RelativeEncoder encoderPivoteo;

  // Declaración del control en el puerto 0
  Joystick control = new Joystick(0);

  // Declaración del temporizador
  Timer tiempo = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // -----asignación de atributos iniciales-----//
    // Asignarle seguidores a los motores delanteros del chasis
    chasisDerecho2.follow(chasisDerecho1);
    chasisIzquierdo2.follow(chasisIzquierdo1);
    // Declaración de los encoders del chasis
    encoderChasisDerecho = chasisDerecho1.getEncoder();
    encoderChasisIzquierdo = chasisIzquierdo1.getEncoder();
    // Asignar el motor
    intake2.follow(intake1);
    shooter2.follow(shooter1);
    pivoteo2.follow(pivoteo1);
    pivoteo2.setInverted(true);
    encoderPivoteo = pivoteo1.getEncoder();
    chasis.tankDrive(0.5, 0.5);

  }
/**
 * Lo que el robot ejecuta de manera constante tanto en el modo autónomo como en el teleoperado
 */
  @Override
  public void robotPeriodic() {
    encodersChasis[0] = encoderChasisDerecho.getPosition();
    encodersChasis[1] = encoderChasisIzquierdo.getPosition();
    printEncoderIntake();
    printEncodersChasis();
    printIntakeSwitch();
    printPivoteoSwitchAbajo();
    printPivoteoSwitchArriba();
  }
/**
 * Función que se ejecuta una sola vez al iniciar el modo autónomo
 */
  @Override
  public void autonomousInit() {
    tiempo.start();
  };
/**
 * Función que se ejecuta de manera constante durante todo el modo autónomo
 */
  @Override
  public void autonomousPeriodic() {
    if (tiempo.get() <= 2) {
      pivoteoFunc();
    }
    if (tiempo.get() >= 1 && tiempo.get() <= 2) {
      shooterFunc();
    }
    if (tiempo.get() >= 4 && encoderChasisDerecho.getPosition() <= 50) {
      chasisDerecho1.set(0.4);
      chasisIzquierdo1.set(0.4);
    }
    if (tiempo.get() >= 8 && encoderChasisDerecho.getPosition() >= 0) {
      chasisDerecho1.set(-0.4);
      chasisIzquierdo1.set(-0.4);
    }
    if (tiempo.get() >= 10 && encoderChasisDerecho.getPosition() < 5) {
      pivoteoFunc();
    }
    if (tiempo.get() >= 11 && tiempo.get() <= 13) {
      shooterFunc();
    }
  }
/**
 * Función que se ejecuta una sola vez al iniciar el modo teleoperado
 */
  @Override
  public void teleopInit() {
  }
/**
 * Función que se ejecuta de manera periódica en el modo teleoperado
 */
  @Override
  public void teleopPeriodic() {

    // shooter
    while (control.getRawAxis(3) >= 0.2) {
      shooter1.set(0.8);
    }

    // intake
    while (control.getRawAxis(2) >= 0.2 && !switchIntake.get()) {
      intake1.set(0.5);
    }

    // intaken´t
    while (control.getRawButton(5)) {
      intake1.set(-0.5);
    }

    // pivoteo arriba
    if (control.getRawButton(4) && !switchPivoteoArriba.get()) {
      pivoteo = true;
    }
    while (pivoteo) {
      if (encoderPivoteo.getPosition() <= 20) {
        pivoteo1.set(0.3);
      } else {
        pivoteo1.set(0);
      }
    }

    if (control.getRawButton(3)) {
      pivoteo = false;
    }

    while (!pivoteo && !switchPivoteoAbajo.get()) {
      if (encoderPivoteo.getPosition() >= 0) {
        pivoteo1.set(-0.3);
      } else {
        pivoteo1.set(0);
      }
    }

  

  }
/**
 * Función que se ejecuta una vez después de detenerse la ejecución del resto del programa, ya sea por finalización de la partida o por finalización forzada
 */
  @Override
  public void disabledInit() {
  }
/**
 * Código que se ejeuta indefinidamente mientras el robot esté deshabilitado
 */
  @Override
  public void disabledPeriodic() {
  }


 /**
  * Función que al ser llamada acciona los motores del mecanismo especificado a cierta velocidad de acuerdo a los parámetros dados 
  * @param mecanismo Se indica el mecanismo a accionar: pivoteo, chasis, intake, shooter
  * @param position Se indica la posision a la cual el encoder tiene que igualar
  * @param speed Se indica la cantidad de energía que se le mandará al motor
  */
  public void runToPosition(String mecanismo, double position, double speed) {
    switch (mecanismo) {
      case "pivoteo":
        while (encoderPivoteo.getPosition() <= position) {
          pivoteo1.set(speed);
        }
        break;
      case "chasis":
        while (encoderChasisDerecho.getPosition() <= position) {
          chasisDerecho1.set(0.5);
          chasisIzquierdo1.set(0.5);
        }
        break;
    }
  }



/**
 * Función que le asigna 0.5 de velocidad al motor del intake al ser llamada
 */
  public void intakeFunc() {
    intake1.set(0.9);
  }
/**
 * Función que le asigna velocidad negativa el motos del intake, usada para sacar la nota que tenga dentro
 */
  public void notIntakeFunc() {
    intake1.set(-0.9);
  }
/**
 * Función que lleva el pivoteo hasta la posición 20 del encoder manteniendo el motor energizado de manera indefinida
 */
  public void pivoteoFunc() {
    while (encoderPivoteo.getPosition() <= 20) {
      pivoteo1.set(0.3);
    }
    pivoteo1.set(0.05);
  }
/**
 * Función que lleva al pivoteo a su posición original
 */
  public void notPivoteoFunc() {
    while (encoderPivoteo.getPosition() >= 0) {
      pivoteo1.set(-0.3);
    }
  }
/**
 * Función que acciona los motores del shooter con velocidad de 0.8
 */
  public void shooterFunc() {
    shooter1.set(0.8);
  }
//Función que imprime la posición del encoder del intake
  public void printEncoderIntake(){
    SmartDashboard.putNumber("encoderPivoteo", encoderPivoteo.getPosition());
}
//Función que sirve para saber si la nota ya entró al intake 
public void printIntakeSwitch(){
  SmartDashboard.putBoolean("switchIntake", switchIntake.get());
}
//Función que sirve para saber si el pivoteo llegó a su altura máxima 
public void printPivoteoSwitchArriba(){
  SmartDashboard.putBoolean("switchPivoteoArriba", switchPivoteoArriba.get());
}
//Función que sirve para saber si el pivoteo está hasta abajo
public void printPivoteoSwitchAbajo(){
  SmartDashboard.putBoolean("switchPivoteoAbajo", switchPivoteoAbajo.get());
}
/*Función que sirve para saber la posición de los encoders del chasis, en caso de que uno esté disparejo se puede llegar 
    a ver en los valores del encoder. Igual sirve para los autónomos en caso de que no usemos PathPlanner*/ 
public void printEncodersChasis(){
  SmartDashboard.putNumberArray("EncodersChasis", encodersChasis);
}
}


 