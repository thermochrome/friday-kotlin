package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.teleop.Activation

// CommandBase

data class ButtonData(
    val key: GamepadKeys.Button,
    val activation: Activation,
    val activated: Command,
    val deactivated: Command? = null,
    val gamepad: GamepadEx? = null
)

fun registerControls(bindings: Iterable<ButtonData>, defaultGamepad: GamepadEx) {
    bindings.forEach {
        val button = GamepadButton(it.gamepad ?: defaultGamepad, it.key)

        when (it.activation) {
            Activation.TOGGLE ->
                button.toggleWhenPressed(it.activated, it.deactivated)

            Activation.PRESS ->
                button.whenPressed(it.activated)

            Activation.HELD ->
                button.whenPressed(it.activated).whenReleased(it.deactivated)
        }
    }
}