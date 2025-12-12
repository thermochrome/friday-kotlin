package org.firstinspires.ftc.teamcode.teleop

import android.R
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
import kotlin.math.pow
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

    private var powers: MutableMap<String, Any> = mutableMapOf(
        "outtake_power" to 1.0,
        "outtake_enabled" to false
    )

    override fun init() {
        robot = Robot(hardwareMap, gamepad1, gamepad2)

        val buttons = mapOf(
            ButtonData(GamepadKeys.Button.X,
                InstantCommand({
                    powers["outtake_enabled"] = true
//                    robot.hardware.get<MotorGroup>("outtake").set(powers.getValue("outtake_power"))
                }),

                InstantCommand({
                    powers["outtake_enabled"] = false
//                    robot.hardware.get<MotorGroup>("outtake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.A,
                InstantCommand({
                    robot.hardware.get<MotorEx>("intake").set(1.0)
                }),

                InstantCommand({
                    robot.hardware.get<MotorEx>("intake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.Y,
                InstantCommand({
                    robot.hardware.get<CRServo>("upper_intake").set(-1.0)
                }),

                InstantCommand({
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
                powers["outtake_power"] = (powers["outtake_power"]!! as Double) + 0.05
            })) to Activation.PRESS,

            ButtonData(GamepadKeys.Button.DPAD_LEFT, InstantCommand({
                powers["outtake_power"] = (powers["outtake_power"]!! as Double) - 0.05
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

        val power: () -> Double = { if (powers["outtake_enabled"]!! as Boolean) (powers["outtake_power"]!! as Double) else 0.0 }

        robot.hardware.get<MotorGroup>("outtake").set(power.invoke())

        val result = robot.hardware.get<Limelight3A>("limelight").latestResult

        telemetry.addData("TAG", result.botposeTagCount)

        telemetry.addData("Heading", result.botpose.orientation.yaw)
        telemetry.addData("Position", result.botpose.position)

        telemetry.addData("Distance", sqrt(result.botpose.position.x.pow(2) + result.botpose.position.y.pow(2)))


        telemetry.update()
        robot.loop()
    }
}