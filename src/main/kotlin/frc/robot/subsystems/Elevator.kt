package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier

class Elevator : SubsystemBase() {
    private val motor = TalonFX(1)

    fun setPower(voltage: Supplier<Voltage>): Command = Commands.run({ motor.setVoltage(voltage.get().`in`(Volts)) })
}