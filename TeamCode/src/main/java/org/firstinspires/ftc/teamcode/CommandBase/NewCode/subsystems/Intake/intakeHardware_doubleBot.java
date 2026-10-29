package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

  public class intakeHardware_doubleBot extends SubSystem {

      MotorEx IMotorF = new MotorEx();
      MotorEx IMotorB = new MotorEx();
      ServoDegrees SBServo = new ServoDegrees();
      public intakeHardware_doubleBot(OpModeEX opModeEX) {
          registerSubsystem(opModeEX, returnDefaultCommand());
      }


      public enum intakeState {
          off,
          intaking,

          ejecting,

          transfering

      }

      ElapsedTime ReverseTimer = new ElapsedTime();
      ElapsedTime IntakeTimer = new ElapsedTime();

      public intakeState State = intakeState.off;
      public double Vertical = 0.00;
      public double IntakingDrection = 0; //0=off 1=forward -1=backwards
      boolean intakeAfterReverse = false;
      public boolean blocking = true;
      boolean reverse;
      double reverseTime;
      double intakeTime;

      double ITMFcurrentPower = 0;
      double ITMBcurrentPower = 0;

      @Override
      public void init() {

          IMotorF.initMotor("IMotorF", getOpMode().hardwareMap);
          IMotorB.initMotor("IMotorB", getOpMode().hardwareMap);
          SBServo.initServo("SBServo", getOpMode().hardwareMap);

          SBServo.setRange(255);
      }

      @Override
      public void execute() {
          //controls the motor relative to the state which will be set by a different loop
          switch (State) {
              case off:
                  ITMFcurrentPower = 0;
                  ITMBcurrentPower = 0;

                  IntakingDrection = 0;
                  break;

              case intaking:
                  SBServo.setPosition(30);
                  if (Vertical > 0){
                      //front
                      ITMFcurrentPower = 1;
                      ITMBcurrentPower = 0;

                      IntakingDrection = 1;

                  } else if (Vertical<0) {
                      //back
                      ITMFcurrentPower = 0;
                      ITMBcurrentPower = 1;

                      IntakingDrection = -1;

                  }else {
                      //continue
                      if (IntakingDrection == 1) {
                          ITMFcurrentPower = 1;
                          ITMBcurrentPower = 0;

                      } else if (IntakingDrection == -1) {
                          ITMFcurrentPower = 0;
                          ITMBcurrentPower = 1;

                      } else if (IntakingDrection == 0) {
                          ITMFcurrentPower = 0;
                          ITMBcurrentPower = 0;
                      }


                  }

                  break;

              case ejecting:
                  SBServo.setPosition(30);

                  if (Vertical > 0){
                      //front
                      ITMFcurrentPower = -1;
                      ITMBcurrentPower = 0;

                  } else if (Vertical<0) {
                      //back
                      ITMFcurrentPower = 0;
                      ITMBcurrentPower = -1;

                  }else {
                      //off
                      ITMFcurrentPower = 0;
                      ITMBcurrentPower = 0;
                  }

                  break;


              case transfering:
                  //open
                  //continue
                  //transfer
                  SBServo.setPosition(0);

                  if (Vertical > 0){
                      //front
                      ITMFcurrentPower = 1;
                      ITMBcurrentPower = -1;

                      IntakingDrection = 1;

                  } else if (Vertical<0) {
                      //back
                      ITMFcurrentPower = -1;
                      ITMBcurrentPower = 1;

                      IntakingDrection = -1;

                  }else {
                      //continue
                      if (IntakingDrection == 1) {
                          ITMFcurrentPower = 1;
                          ITMBcurrentPower = -1;

                      } else if (IntakingDrection == -1) {
                          ITMFcurrentPower = -1;
                          ITMBcurrentPower = 1;

                      } else if (IntakingDrection == 0) {
                          ITMFcurrentPower = -1;
                          ITMBcurrentPower = -1;
                      }
                  }



          }


          IMotorF.update(ITMFcurrentPower);
          IMotorB.update(ITMBcurrentPower);


              if (ReverseTimer.milliseconds() > reverseTime && reverse) {
                  ReverseTimer.reset();
                  State = intakeState.ejecting;
                  reverse = false;

              } else if (!reverse && ReverseTimer.milliseconds() > reverseTime && State == intakeState.ejecting) {
                  State = intakeState.intaking;
                  IntakeTimer.reset();
                  intakeAfterReverse = true;

              } else if (intakeAfterReverse && IntakeTimer.milliseconds() > intakeTime) {
                  State = intakeState.off;
                  intakeAfterReverse = false;

              }
      }


  }