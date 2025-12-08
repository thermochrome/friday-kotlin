package org.firstinspires.ftc.teamcode.teleop

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.Activation
import org.firstinspires.ftc.teamcode.robot.ButtonData
import org.firstinspires.ftc.teamcode.robot.Robot

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
                    val power = powers.getValue("outtake_power")
                    (robot.hardware["outtake_l"] as MotorEx).set(power)
                    (robot.hardware["outtake_r"] as MotorEx).set(-power)
                }),

                InstantCommand({
                    (robot.hardware["outtake_l"] as MotorEx).set(0.0)
                    (robot.hardware["outtake_r"] as MotorEx).set(0.0)
                })
            ) to Activation.TOGGLE,

            ButtonData(GamepadKeys.Button.Y,
                InstantCommand({
                    (robot.hardware["intake"] as MotorEx).set(-1.0)
                }),

                InstantCommand({
                    (robot.hardware["intake"] as MotorEx).set(0.0)
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
        )

        buttons.forEach { (buttonData, activation) ->
            val button = GamepadButton(robot.gamepads.second, buttonData.key)

            when (activation) {
                Activation.TOGGLE -> button.toggleWhenPressed(
                    buttonData.activated,
                    buttonData.deactivated
                )

                Activation.PRESS -> button.whenPressed(buttonData.activated)

                Activation.HELD -> button.whenHeld(buttonData.activated)
            }
        }
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        telemetry.addData("RPM", (robot.hardware["outtake_l"] as MotorEx).correctedVelocity * 60 / 103.8)
        telemetry.addData("Power", powers["outtake_power"])

        val result = (robot.hardware["limelight"] as Limelight3A).latestResult

        if (result.isValid) {
            telemetry.addData("Pose", result.botpose.toString())
        }

        telemetry.update()
        robot.loop()
        robot.drive.drive(robot.gamepads.first, 1.0)
    }
}