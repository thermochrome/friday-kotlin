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
        "outtake_power" to 1.0
    )

    override fun init() {
        robot = Robot(hardwareMap, gamepad1, gamepad2)

        val buttons = mapOf(
            ButtonData(GamepadKeys.Button.X,
                InstantCommand({
                    robot.hardware.get<MotorGroup>("outtake").set(powers.getValue("outtake_power"))
                }),

                InstantCommand({
                    robot.hardware.get<MotorGroup>("outtake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.Y,
                InstantCommand({
                    (robot.hardware["intake"] as MotorEx).set(1.0)
                }),

                InstantCommand({
                    (robot.hardware["intake"] as MotorEx).set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.B,
                InstantCommand({
                    (robot.hardware["upper_intake"] as CRServo).set(-1.0)
                }),

                InstantCommand({
                    (robot.hardware["upper_intake"] as CRServo).set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.DPAD_UP, InstantCommand({
                (robot.hardware["feeder"] as Servo).position = (45.0 / 57.2958)
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_DOWN, InstantCommand({
                (robot.hardware["feeder"] as Servo).position = (22.0 / 57.2958)
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_RIGHT, InstantCommand({
                powers["outtake_power"] = powers["outtake_power"]!! + 0.05
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_LEFT, InstantCommand({
                powers["outtake_power"] = powers["outtake_power"]!! - 0.05
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_UP,
                InstantCommand({
                    robot.hardware.get<MotorEx>("up").set(1.0)
                }),

                InstantCommand({
                    robot.hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = robot.gamepads.first
            ) to Activation.HELD,

            ButtonData(GamepadKeys.Button.DPAD_DOWN,
                InstantCommand({
                    robot.hardware.get<MotorEx>("up").set(-1.0)
                }),

                InstantCommand({
                    robot.hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = robot.gamepads.first
            ) to Activation.HELD,
        )

        buttons.forEach { (buttonData, activation) ->
            val button = GamepadButton(buttonData.gamepad ?: robot.gamepads.second, buttonData.key)

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

        val result = robot.hardware.get<Limelight3A>("limelight").latestResult

        if (result.isValid) {
            telemetry.addData("Pose", result.botpose.toString())
        }

        telemetry.update()
        robot.loop()
        robot.drive.drive(robot.gamepads.first)
    }
}