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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.OpDevices
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
class Main: OpDevices() {
    private var powers = mutableMapOf(
        "outtake_power" to 0.75
    )

    override fun init() {
        super.init()

        val buttons = mapOf(
            ButtonData(GamepadKeys.Button.X,
                activated = InstantCommand({
                    powers["outtake_power"] = 0.75 /* powers["outtake_power"]!![1] */
                }),

                deactivated = InstantCommand({
                    powers["outtake_power"] = 0.0
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.A,
                activated = InstantCommand({
                    hardware.get<MotorEx>("intake").set(1.0)
                }),

                deactivated = InstantCommand({
                    hardware.get<MotorEx>("intake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.Y,
                activated = InstantCommand({
                    hardware.get<CRServo>("upper_intake").set(-1.0)
                }),

                deactivated = InstantCommand({
                    hardware.get<CRServo>("upper_intake").set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.DPAD_UP,
                InstantCommand({
                    hardware.get<MotorEx>("up").set(1.0)
                }),

                InstantCommand({
                    hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = firstGamepad
            ) to Activation.HELD,

            ButtonData(GamepadKeys.Button.DPAD_DOWN,
                InstantCommand({
                    hardware.get<MotorEx>("up").set(-1.0)
                }),

                InstantCommand({
                    hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = firstGamepad
            ) to Activation.HELD,

            ButtonData(GamepadKeys.Button.DPAD_UP, InstantCommand({
                val time = ElapsedTime()

                hardware.get<Servo>("feeder").position = (45.0 / 57.2958)

                if (time.milliseconds() >= 600) {
                    hardware.get<Servo>("feeder").position = (22.0 / 57.2958)
                }
            })) to Activation.PRESS,

//            ButtonData(GamepadKeys.Button.DPAD_DOWN, InstantCommand({
//                robot.hardware.get<Servo>("feeder").position = (22.0 / 57.2958)
//            })) to Activation.PRESS
        )

        buttons.forEach { (buttonData, activation) ->
            val button = GamepadButton(buttonData.gamepad ?: secondGamepad, buttonData.key)

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

    override fun loop() {
        super.loop()

        val limelight: Limelight3A = hardware["limelight"]
        telemetry.addData("RPM", hardware.get<MotorGroup>("outtake").velocity * 60 / 103.8)

        var minimumDistance = Double.MAX_VALUE

        limelight.latestResult.fiducialResults.associate { tag ->
            val position = listOf(
                tag.targetPoseCameraSpace.position.x,
                tag.targetPoseCameraSpace.position.y,
                tag.targetPoseCameraSpace.position.z
            )

            tag.fiducialId to sqrt(position.sumOf { d -> d * d })
        }.forEach { (tagId, distance) ->
            telemetry.addData("$tagId Distance", distance)

            if (distance < minimumDistance) {
                minimumDistance = distance
            }
        }

        if (minimumDistance == Double.MAX_VALUE) {
            minimumDistance = 0.0
        }

        hardware.get<MotorGroup>("outtake").set(0.119219 * minimumDistance + 0.611507)

        if (limelight.latestResult.isValid) {
            telemetry.addData("Heading", limelight.latestResult.fiducialResults[0].targetPoseCameraSpace.position.x)
        }
    }
}