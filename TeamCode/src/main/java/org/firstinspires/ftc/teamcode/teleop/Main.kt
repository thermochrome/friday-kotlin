package org.firstinspires.ftc.teamcode.teleop

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.Robot
import kotlin.math.sqrt

enum class Activation {
    TOGGLE, PRESS, HELD
}

data class ButtonData(
    val key: GamepadKeys.Button,
    val activated: Command,
    val deactivated: Command? = null,
    val gamepad: GamepadEx? = null
)

@Suppress("unused") @TeleOp
class Main: OpMode() {
    private lateinit var robot: Robot

    private var powers = mutableMapOf(
        "outtake_power" to arrayOf(1.0, 1.0)
    )

    override fun init() {
        robot = Robot(hardwareMap, gamepad1, gamepad2)

        val buttons = mapOf(
            ButtonData(GamepadKeys.Button.X,
                activated = InstantCommand({
                    powers["outtake_power"]!![0] = powers["outtake_power"]!![1]
                }),

                deactivated = InstantCommand({
                    powers["outtake_power"]!![0] = 0.0
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.A,
                activated = InstantCommand({
                    robot.hardware.get<MotorEx>("intake").set(1.0)
                }),

                deactivated = InstantCommand({
                    robot.hardware.get<MotorEx>("intake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.Y,
                activated = InstantCommand({
                    robot.hardware.get<CRServo>("upper_intake").set(-1.0)
                }),

                deactivated = InstantCommand({
                    robot.hardware.get<CRServo>("upper_intake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.DPAD_UP, InstantCommand({
                robot.hardware.get<Servo>("feeder").position = (45.0 / 57.2958)
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_DOWN, InstantCommand({
                robot.hardware.get<Servo>("feeder").position = (22.0 / 57.2958)
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_RIGHT, InstantCommand({
                powers["outtake_power"]!![0] = powers["outtake_power"]!![1] + 0.05
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_LEFT, InstantCommand({
                powers["outtake_power"]!![0] = powers["outtake_power"]!![1] - 0.05
            })) to Activation.PRESS
        )

        buttons.forEach { (buttonData, activation) ->
            val button = GamepadButton(buttonData.gamepad ?: robot.secondGamepad, buttonData.key)

            when (activation) {
                Activation.TOGGLE -> button.toggleWhenPressed(
                    buttonData.activated,
                    buttonData.deactivated
                )

                Activation.PRESS -> button.whenPressed(buttonData.activated)

                Activation.HELD -> button.whenPressed(buttonData.activated).whenReleased(buttonData.deactivated)
            }
        }
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        telemetry.addData("RPM", robot.hardware.get<MotorGroup>("outtake").velocity * 60 / 103.8)
        telemetry.addData("Power", powers["outtake_power"])

        robot.hardware.get<MotorGroup>("outtake").set(powers["outtake_enabled"]!![0])

        val result = robot.hardware.get<Limelight3A>("limelight").latestResult

        if (result.fiducialResults.isNotEmpty()) {
            result.fiducialResults.forEach { tag ->
                val position = listOf(
                    tag.targetPoseCameraSpace.position.x,
                    tag.targetPoseCameraSpace.position.y,
                    tag.targetPoseCameraSpace.position.z
                )

                val distance = sqrt(position.sumOf { d -> d * d })

                telemetry.addData("${tag.fiducialId} Distance", distance)
                telemetry.addData("${tag.fiducialId} Heading", tag.targetPoseCameraSpace.orientation.yaw)
            }
        }

        telemetry.update()
        robot.loop()
    }
}