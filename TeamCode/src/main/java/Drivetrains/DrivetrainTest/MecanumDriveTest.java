package Drivetrains.DrivetrainTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import Drivetrains.MecanumConstants;
import Drivetrains.MecanumDrive;
import Localizers.PinpointLocalizer;
import Util.Pose;

import java.util.Arrays;
import java.util.List;

/**
 * Beta test opmode for MecanumDrive :)
 * @author Sohum Arora - 22985 Paraducks
 */
@TeleOp(name = "MecanumDrive Test", group = "Apex beta test")
public class MecanumDriveTest extends LinearOpMode {

    // Motor hardware map names — update these to match your config
    private static final String lF = "leftFront";
    private static final String rF = "rightFront";
    private static final String lR = "leftRear";
    private static final String rR = "rightRear";

    // Drive mode toggles — Y = field/bot centric, B = brake/float
    private boolean fieldCentric = false;
    private boolean brakeMode = false;

    // Edge-detection booleans to prevent repeated toggle on hold
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastStart = false;

    // Pinpoint localizer setup — update name and offsets to match your config
    PinpointLocalizer localizer;
    String localizerName = "localizer"; //todo change as required
    double localizerXOffset = 0.0;         //todo replace with actual offset
    double localizerYOffset = 0.0;         //todo replace with actual offset

    @Override
    public void runOpMode() {

        // Init localizer
        localizer = new PinpointLocalizer(hardwareMap, localizerName, localizerXOffset, localizerYOffset);
        localizer.init();

        // Build motor list and power-tracking array for MecanumDrive
        List<DcMotorEx> motors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, lF),
                hardwareMap.get(DcMotorEx.class, lR),
                hardwareMap.get(DcMotorEx.class, rF),
                hardwareMap.get(DcMotorEx.class, rR)
        );
        double[] lastMotorPowers = new double[motors.size()];

        // Init drive with constants, motors, and power tracker
        MecanumConstants constants = new MecanumConstants();
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, constants,
                motors, lastMotorPowers
        );

        // Pre-start telemetry instructions
        telemetry.addLine("Mecanum Drivetrain Movement Test");
        telemetry.addLine("Y - toggle field centric and bot centric");
        telemetry.addLine("B - toggle brake mode on and off");
        telemetry.update();

        waitForStart();

        // Apply initial brake mode setting
        drive.startTeleopDrive(brakeMode);

        while (opModeIsActive()) {

            // Update localizer and grab current pose
            localizer.update();
            Pose currentPose = localizer.getPose();

            // Read driver inputs (invert Y so forward stick = positive drive)
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Y: toggle field-centric / bot-centric (rising edge only)
            if (gamepad1.y && !lastY) {
                fieldCentric = !fieldCentric;
                telemetry.addLine("Field-Centric: " + fieldCentric);
            }

            // B: toggle brake mode and reinit drive accordingly (rising edge only)
            if (gamepad1.b && !lastB) {
                brakeMode = !brakeMode;
                drive.startTeleopDrive(brakeMode);
                telemetry.addLine("Brake Mode: " + brakeMode);
            }

            // Start: emergency stop — cuts all motor power
            if (gamepad1.start && !lastStart) {
                drive.breakFollowing();
                telemetry.addLine("Motors stopped (breakFollowing)");
            }

            // Update edge-detection state for next loop
            lastY     = gamepad1.y;
            lastB     = gamepad1.b;
            lastStart = gamepad1.start;

            // Drive only when start isn't held (avoids driving during e-stop)
            if (!gamepad1.start) {
                if (fieldCentric) {
                    // Field-centric: inputs rotated by robot heading
                    drive.fieldCentricDrive(x, y, turn, currentPose.getHeading());
                } else {
                    // Bot-centric: inputs relative to robot front
                    drive.botCentricDrive(x, y, turn);
                }
            }

            // Telemetry output
            telemetry.addData("Bot pose ", currentPose);
            telemetry.addData("x ", currentPose.getX());
            telemetry.addData("y ",currentPose.getY());
            telemetry.addData("heading ", currentPose.getHeading());
            telemetry.addData("Brake mode ", brakeMode);

            if (fieldCentric) {
                telemetry.addLine("Field Centric drive");
            } else {
                telemetry.addLine("Bot centric drive");
            }
            telemetry.update();
        }
    }
}