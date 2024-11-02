package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "DoNothingAutonomous", group = "Autonomous")
public class DoNothingAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize any components if needed (e.g., motors, sensors)

        // Wait for the start command from the driver station

        waitForStart();

        // No action performed here, program will just end after starting
        while (opModeIsActive()){

        }
    }

}