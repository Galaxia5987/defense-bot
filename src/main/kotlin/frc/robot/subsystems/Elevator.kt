package frc.robot.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier

class Elevator : SubsystemBase() {
    private val motor = TalonFX(11)
    private val motorConfig =
        TalonFXConfiguration().withMotorOutput(
            MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))


    init {
        motor.configurator.apply(motorConfig)
    }


    fun setPower(voltage: Supplier<Voltage>): Command =
        Commands.run({ motor.setVoltage(voltage.get().`in`(Volts)) }, this)
}
