package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower
import org.firstinspires.ftc.teamcode.util.G

@TeleOp
class ExtensionPidfTuner : PidfTuner(Subsystem.EXTENSION) {
    override fun updateOutput(output: Double) = setExtensionPower(output)

    override fun readInput() = G.ehub.extension.currentPosition.toDouble()
}