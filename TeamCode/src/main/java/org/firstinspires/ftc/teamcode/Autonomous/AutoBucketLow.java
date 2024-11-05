package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumSubsystem;

@Autonomous
public class AutoBucketLow extends LinearOpMode {

    MecanumSubsystem mecanumSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);

        waitForStart();
        encoderElePos("up",4176,5 );
        encoderDriveToPosition("forward", 1466, 5 );
        armSubsystem.intake(1);
        sleep(2000);
        encoderDriveToPosition("backwards", 1440, 5);
        encoderElePos("down", 4176, 5);
    }

    public void encoderExtPos(String direction, int counts, double timeout){
        armSubsystem.encoderExtend(direction, counts);
        runtime.reset();
        while ((opModeIsActive() && runtime.seconds() < timeout && armSubsystem.extMotorBusy())){}
        armSubsystem.extMotorOff();
    }
    public void encoderElePos(String direction, int counts, double timeout){
        elevatorSubsystem.encoderElevator(direction, counts);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeout && elevatorSubsystem.eleMotorsBusy()){}
        elevatorSubsystem.eleMotorsOff();
    }
    public void encoderDriveToPosition(String direction, int counts, double timeout){
        mecanumSubsystem.encoderDrive(direction, counts);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && mecanumSubsystem.areMotorsBusy()){}
        mecanumSubsystem.driveMotorsOff();
    }


    //low left ele = -4170
    //low right ele = -4176
    //low ext = 0

    //122.2 = 1 inch
}
