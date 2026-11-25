package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
      ServoDegrees FServo = new ServoDegrees();


      private DigitalChannel frontSensor1;
      private DigitalChannel frontSensor2;
      private DigitalChannel frontSensor3;
      private DigitalChannel frontSensor4;
      private DigitalChannel middleSensor1;
      private DigitalChannel middleSensor2;
      private DigitalChannel centerSensor1;






      public intakeHardware_doubleBot(OpModeEX opModeEX) {
          registerSubsystem(opModeEX, returnDefaultCommand());
      }


      public enum intakeState {
          off,
          intaking,

          ejecting,

          transfering

      }
      public enum FollowedIntakeState {
          off,
          intaking,

          ejecting,

          transfering

      }

      ElapsedTime ReverseTimer = new ElapsedTime();
      ElapsedTime IntakeTimer = new ElapsedTime();

      public intakeState State = intakeState.off;
      FollowedIntakeState followedState = FollowedIntakeState.off;

      public double Vertical = 0.00;
      public double IntakingDrection = 0;
      public boolean Flower = false;
      public double Count;

      double ITMFcurrentPower = 0;
      double ITMBcurrentPower = 0;
      public double shooterRPM;
      ElapsedTime minblock = new ElapsedTime();

      @Override
      public void init() {

          IMotorF.initMotor("IMotorF", getOpMode().hardwareMap);
          IMotorB.initMotor("IMotorB", getOpMode().hardwareMap);
          SBServo.initServo("SBServo", getOpMode().hardwareMap);
          FServo.initServo("FServo", getOpMode().hardwareMap);

          frontSensor1 = getOpMode().hardwareMap.get(DigitalChannel.class, "frontSensor1");
          frontSensor2 = getOpMode().hardwareMap.get(DigitalChannel.class, "frontSensor2");
          frontSensor3 = getOpMode().hardwareMap.get(DigitalChannel.class, "frontSensor3");
          frontSensor4 = getOpMode().hardwareMap.get(DigitalChannel.class, "frontSensor4");
          middleSensor1 = getOpMode().hardwareMap.get(DigitalChannel.class, "middleSensor1");
          middleSensor2 = getOpMode().hardwareMap.get(DigitalChannel.class, "middleSensor2");
          centerSensor1 = getOpMode().hardwareMap.get(DigitalChannel.class, "centerSensor1");
          IMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
          frontSensor1.setMode(DigitalChannel.Mode.INPUT);
          frontSensor2.setMode(DigitalChannel.Mode.INPUT);
          frontSensor3.setMode(DigitalChannel.Mode.INPUT);
          frontSensor4.setMode(DigitalChannel.Mode.INPUT);
          middleSensor1.setMode(DigitalChannel.Mode.INPUT);
          middleSensor2.setMode(DigitalChannel.Mode.INPUT);
          centerSensor1.setMode(DigitalChannel.Mode.INPUT);






          SBServo.setRange(255);
          FServo.setRange(255);

      }

      @Override
      public void execute() {
          //controls the motor relative to the state which will be set by a different loop
          if (Vertical> 0){
              IntakingDrection=1;
          } else if (Vertical < 0 ) {
              IntakingDrection= -1;
          }
          double count = (frontSensor1.getState() ? 0 : 1) +(frontSensor2.getState() ? 0 : 1) + (frontSensor3.getState() ? 0 : 1) + (frontSensor4.getState() ? 0 : 1) + (middleSensor1.getState() ? 0 : 1) + (middleSensor2.getState() ? 0 : 1) + (centerSensor1.getState() ? 0 : 1);

          if (count <= 4 && State == intakeState.intaking && !(followedState == FollowedIntakeState.intaking)){
              followedState = FollowedIntakeState.intaking;
              minblock.reset();
          } else if (count > 4 && State == intakeState.intaking && shooterRPM > 100 ) {
              followedState = FollowedIntakeState.transfering;

          }else if (State == intakeState.off){
              followedState = FollowedIntakeState.off;

          }
          if (State == intakeState.transfering){
              followedState = FollowedIntakeState.transfering;
          }
          if (Flower){
              FServo.setPosition(0);
          }else{
              FServo.setPosition(140);
          }
          Count = count;

          switch (followedState) {
              case off:

                  ITMFcurrentPower = 0;
                  ITMBcurrentPower = 0;

                  break;

              case intaking:
                  if (minblock.milliseconds() > 30 ) {
                      SBServo.setPosition(50);
                  }
                  if (IntakingDrection==1){
                      //front
                      ITMFcurrentPower = -1;
                      ITMBcurrentPower = 0;


                  } else if (IntakingDrection==-1) {
                      //back
                      ITMFcurrentPower = 0;
                      ITMBcurrentPower = -1;


                  }else {
                      //continue
                      if (IntakingDrection == 1) {
                          ITMFcurrentPower = -1;
                          ITMBcurrentPower = 0;

                      } else if (IntakingDrection == -1) {
                          ITMFcurrentPower = 0;
                          ITMBcurrentPower = -1;

                      } else if (IntakingDrection == 0) {
                          ITMFcurrentPower = -1;
                          ITMBcurrentPower = -1;
                      }


                  }

                  break;

              case ejecting:
                  SBServo.setPosition(50);

                  if (IntakingDrection==1){
                      //front
                      ITMFcurrentPower = 1;
                      ITMBcurrentPower = 0;

                  } else if (IntakingDrection==-1) {
                      //back
                      ITMFcurrentPower = 0;
                      ITMBcurrentPower = 1;

                  }else {
                      ITMFcurrentPower = 1;
                      ITMBcurrentPower = 1;
                  }

                  break;


              case transfering:
                  //open
                  //continue
                  //transfer
                  SBServo.setPosition(0);

                      //front
                      ITMFcurrentPower = -1;
                      ITMBcurrentPower = -1;

          }


          IMotorF.update(ITMFcurrentPower);
          IMotorB.update(ITMBcurrentPower);



      }


  }