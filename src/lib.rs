#![no_std]

use bit_field::BitField;
use embedded_hal::blocking;
use embedded_hal::digital::v2::InputPin;
use enum_iterator::IntoEnumIterator;

/// The driver for interacting with the LTC2942 battery monitor.
///
/// In order to utilize the charge counting functionality of the device, you must specify the sense
/// resistance using [set_sense_resistance](struct.Ltc2942.html#method.set_sense_resistance).
/// Changing sense resistance will preserve any previously known battery capacity. If the battery
/// capacity was not known, it will be reset to the mid-point.
///
/// The device starts up assuming the charge level of the connected battery is at mid-level. If the
/// charge level is known, it can be provided to the IC using
/// [set_capacity](struct.Ltc2942.html#method.set_capacity).
///
/// ## Supported Measurements
/// The device also supports measurement of the the following values:
/// * SENSE- terminal voltage: [get_voltage](struct.Ltc2942.html#method.get_voltaqge).
/// * Temperature: [temperature_celcius](struct.Ltc2942.html#method.temperature_celcius).
/// * Charge: [get_capacity](struct.Ltc2942.html#method.get_capacity).
///
/// ## Charge capacity and resolution
/// The device supports a configurable coloumb-counting pre-scaler, which is used to set the
/// full-scale range of the capacity measurement. Higher pre-scalers correspond with a higher
/// battery capacity, but lower capacity resolution.
///
/// ## Alarms
/// The monitor supports configurable alarms for temperature-out-of-range, voltage-out-of-range,
/// high charge level, and low charge level. All alarm checks are completed by polling using
/// [check_alarms](struct.Ltc2942.html#method.check_alarms).
///
///  ## Low-power operation
/// The device supports low-power operation by allowing the user to power down the ADC or analog
/// peripherals. When analog peripherals are powered down using
/// [shutdown](struct.Ltc2942.methods#shutdown), all charge information below 1 LSB is lost and
/// analog measurements are no longer updated.
///
/// ## Temperature and Voltage Measurement
/// The ADC (temperature and voltage measurements) may be disabled by using
/// [configure_adc](struct.Ltc2942.methods#configure_adc) to save power. The ADC supports a number
/// of power modes including sleep, where temperature and voltage are not updated, as well as
/// automatic or manual voltage and temperature conversions. The default setting is to measure
/// voltage and temperature once per second.
pub struct Ltc2942<I2C, ALARM> {
    i2c: I2C,
    r_sense: Option<f32>,
    alarm: ALARM,
    m_factor: u8,
}

/// Specifies the charge prescaler, which configures the resolution of the charge counter. Lower
/// prescalers correspond with a higher resolution.
#[derive(Copy, Clone)]
pub enum Prescaler {
    M1 = 0,
    M2 = 0b01,
    M4 = 0b10,
    M8 = 0b11,
    M16 = 0b100,
    M32 = 0b101,
    M64 = 0b110,
    M128 = 0b111,
}

#[derive(Copy, Clone)]
enum Register {
    Status = 0,
    Control = 1,
    Charge = 2,
    ChargeThresholdHigh = 4,
    ChargeThresholdLow = 6,
    Voltage = 8,
    VoltageThresholdHigh = 10,
    VoltageThresholdLow = 11,
    Temperature = 12,
    TemperatureThresholdHigh = 14,
    TemperatureThresholdLow = 15,
}

/// Configures the device ADC for specific operation modes. Note that the ADC will not be active if
/// the device is shut down.
#[derive(Copy, Clone)]
pub enum AdcMode {
    /// Place the ADC into sleep mode, where measurements are not performed.
    Sleep = 0b00,
    /// Measure the temperature once and then transition to sleep mode.
    ManualTemperature = 0b01,
    /// Measure the voltage once and then transition to sleep mode.
    ManualVoltage = 0b10,
    /// Measure temperature and voltage at regular 1 second intervals.
    Automatic = 0b11,
}

/// Specifies a configurable alarm from the chip.
#[derive(Copy, Clone, IntoEnumIterator)]
pub enum Alarm {
    /// Alarm indicating that the chip encountered and under-voltage event. Register contents may no
    /// longer be valid.
    UnderVoltageLockout = 0,
    /// Alarm indicating the measured voltage was outside of the voltage limits.
    Voltage = 1,
    /// Alarm indicating measured charge is lower than the configured lower limit.
    ChargeLow = 2,
    /// Alarm indicating measured charge is higher than the configured upper limit.
    ChargeHigh = 3,
    /// Alarm indicating temperature is out of range (either too high or too low).
    Temperature = 4,
    /// Alarm indicating that the charge register has either under- or over-flowed. The exact
    /// battery capacity may no longer be accurate.
    Charge = 5,
}

/// Specified threshold configurations for alarm notification.
#[derive(IntoEnumIterator, PartialEq)]
pub enum Threshold {
    /// Specifies the thresholds for out-of-range voltage alarms.
    Voltage,
    /// Specifies the thresholds for out-of-range temperature alarms.
    Temperature,
    /// Specifies the thresholds for low and high charge alarms.
    Charge,
}

/// Associated error types indicating invalid operation of the driver.
pub enum Error<I2cE> {
    /// An error occurred with the associated I2C interface.
    Interface(I2cE),
    /// An error occurred while trying to read the alarm pin state.
    Pin,
    /// An invalid size was provided when reading or writing a register OR a value was written to a
    /// register that is too large to fit within the register.
    Size,
    /// The sense resistor connected to the device has not been specified. Sense resistance must be
    /// specified before interacting with charge levels.
    NoSenseResistance,
    /// A register write attempt was made to a read-only register.
    Access,
    /// Invalid threshold levels were supplied (e.g. lower threshold is above upper).
    Thresholds,
    /// The device detected is not the LTC2942, but may be the LTC2941.
    IncorrectDevice,
    /// The provided threshold is out-of-range.
    Range,
}

impl<I2cE> From<I2cE> for Error<I2cE> {
    fn from(i2ce: I2cE) -> Self {
        Error::Interface(i2ce)
    }
}

// TODO: Support configuration of the ALARM pin in CHARGE_COMPLETE mode.
impl<PinE, I2cE, I2C, ALARM> Ltc2942<I2C, ALARM>
where
    I2C: blocking::i2c::Read<Error = I2cE> + blocking::i2c::Write<Error = I2cE>,
    ALARM: InputPin<Error = PinE>,
{
    const I2C_ADDRESS: u8 = 0b1100100;

    /// Construct a driver for the LTC2942 battery monitor.
    ///
    /// Arguments:
    /// * `i2c` - The I2C interface to use for communicating with the device. This should make use of
    /// the embedded-hal traits for I2C blocking operations.
    /// * `alarm` - The input pin to use for detecting alarms specified by the LTC2942.
    ///
    /// Example:
    /// ```
    /// use ltc2942;
    /// let ltc2942 = ltc2942::Ltc2942::new(i2c, alarm_pin).unwrap();
    /// ltc2942.set_sense_resistance(0.100).unwrap();
    /// let capacity_mah = ltc2942.get_capacity().unwrap();
    /// println!("Capacity: {} mAh", capacity_mah);
    /// ```
    pub fn new(i2c: I2C, alarm: ALARM) -> Result<Self, Error<I2cE>> {
        let mut ltc2942 = Ltc2942 {
            i2c: i2c,
            r_sense: None,
            alarm: alarm,
            m_factor: 1,
        };

        // Check that we are talking to the LTC2942.
        let status = ltc2942.read_register(Register::Status)?;
        if status.get_bit(7) {
            return Err(Error::IncorrectDevice);
        }

        // Configure the ADC to perform measurements at 1 second intervals and enable the chip.
        // Configure the prescaler for no prescaling (lowest capacity).
        ltc2942.configure_adc(AdcMode::Automatic)?;
        ltc2942.set_charge_prescaler(Prescaler::M1)?;
        ltc2942.enable()?;

        // Load the prescale factor from the chip.
        let mut config = ltc2942.read_register(Register::Control)? as u8;
        ltc2942.m_factor = 1u8.wrapping_shl(config.get_bits(3..6) as u32);

        // Configure the AL/CC pin to operate in alarm mode.
        config.set_bits(1..3, 0b10u8);
        ltc2942.write_register(Register::Control, config as u16)?;

        // TODO: Initialize default values for thresholds?

        Ok(ltc2942)
    }

    /// Specify the shunt sense resistance of the charge monitor.
    ///
    /// Note: Updating the sense resistance will preserve any previously-known charge state.
    ///
    /// Arguments:
    /// * `r_sense` - The sense resistance in Ohms.
    pub fn set_sense_resistance(&mut self, r_sense: f32) -> Result<(), Error<I2cE>> {
        let previous_charge = self.get_capacity().ok();

        self.r_sense = Some(r_sense);

        if let Some(level) = previous_charge {
            self.set_capacity(level)?;
        } else {
            // Configure the capacity to mid-level now that the sense resistor is updated. We have
            // no context as to what state the battery is at and any other readings may be
            // incorrect.
            self.write_register(Register::Charge, 0x7FFF)?;
        }

        // TODO: Reconfigure the alarm limits for capacity.

        Ok(())
    }

    fn read_register(&mut self, register: Register) -> Result<u16, Error<I2cE>> {
        let mut buffer: [u8; 2] = [0; 2];
        let register_size: usize = match register {
            Register::Status => return Err(Error::Size),
            Register::Control | Register::VoltageThresholdLow | Register::VoltageThresholdHigh => 1,
            _ => 2,
        };

        let register_address = register as u8;
        self.i2c
            .write(Self::I2C_ADDRESS, &register_address.to_be_bytes())?;
        self.i2c
            .read(Self::I2C_ADDRESS, &mut buffer[0..register_size])?;

        match register {
            Register::Status | Register::Control => Ok(buffer[0] as u16),
            Register::VoltageThresholdLow | Register::VoltageThresholdHigh => Ok(buffer[0] as u16),
            _ => Ok(u16::from_be_bytes(buffer)),
        }
    }

    fn write_register(&mut self, register: Register, value: u16) -> Result<(), Error<I2cE>> {
        match register {
            Register::Status | Register::Voltage | Register::Temperature => Err(Error::Access),
            Register::Control | Register::VoltageThresholdLow | Register::VoltageThresholdHigh => {
                if value > u8::max_value() as u16 {
                    return Err(Error::Size);
                }

                let data: [u8; 2] = [register as u8, value as u8];
                self.i2c.write(Self::I2C_ADDRESS, &data)?;

                Ok(())
            }
            _ => {
                let mut data: [u8; 3] = [register as u8, 0, 0];
                data.copy_from_slice(&value.to_be_bytes());

                self.i2c.write(register as u8, &data)?;

                Ok(())
            }
        }
    }

    /// Configure the operation mode of the ADC.
    ///
    /// Note: The ADC is used for measuring temperature and voltage. If disabled, the temperature
    /// and voltage alarms may not trigger.
    ///
    /// Note: The ADC configuration is not used if the device has been shutdown.
    ///
    /// Note: Manually triggered ADC measurements will take 15ms to complete.
    ///
    /// Arguments:
    /// * `mode` - The new operation mode to configure for the ADC.
    pub fn configure_adc(&mut self, mode: AdcMode) -> Result<(), Error<I2cE>> {
        let mut config = self.read_register(Register::Control)? as u8;

        config.set_bits(6..8, mode as u8);

        self.write_register(Register::Control, config as u16)?;

        Ok(())
    }

    /// Shut down the chip, forcing it into a low-power state of less than 1uA.
    ///
    /// Note: Shutting down the chip preserves all register state, but any charge data less than 1
    /// LSB will be lost.
    ///
    /// Note: Analog measurements will not be performed while the chip is shut down.
    pub fn shutdown(&mut self) -> Result<(), Error<I2cE>> {
        let mut config = self.read_register(Register::Control)? as u8;

        config.set_bit(0, true);

        self.write_register(Register::Control, config as u16)?;

        Ok(())
    }

    /// Power up the device and resume normal operation.
    ///
    /// Note: Powering up the chip while it is already on will have no effect.
    pub fn enable(&mut self) -> Result<(), Error<I2cE>> {
        let mut config = self.read_register(Register::Control)? as u8;

        config.set_bit(0, false);

        self.write_register(Register::Control, config as u16)?;

        Ok(())
    }

    // TODO: Implement i2c-sensors traits for voltage / temperature readings.

    /// Get the most recent battery voltage measurement in volts.
    ///
    /// Note: Voltage is updated every 2 seconds when the ADC is configured for `Automatic` mode.
    /// Otherwise, measurements must be triggered manually.
    pub fn get_voltage(&mut self) -> Result<f32, Error<I2cE>> {
        let raw_value = self.read_register(Register::Voltage)?;

        Ok((6.0 as f32 * raw_value as f32) / 0xFFFF as f32)
    }

    /// Get the most recent temperature measurement in degrees Celcius.
    ///
    /// Note: Temperature is updated every 2 seconds when the ADC is configured for `Automatic`
    /// mode. Otherwise, measurements must be triggered manually.
    pub fn tempeature_celsius(&mut self) -> Result<f32, Error<I2cE>> {
        let raw_value = self.read_register(Register::Temperature)?;

        let temperature_kelvin = 600 as f32 * (raw_value as f32 / 0xFFFF as f32);

        Ok(temperature_kelvin - 273.15)
    }

    /// Get the current battery capacity in mAh (milli Amp-hours).
    pub fn get_capacity(&mut self) -> Result<f32, Error<I2cE>> {
        let raw_value = self.read_register(Register::Charge)?;

        let r_sense = match self.r_sense {
            Some(r) => r,
            None => return Err(Error::NoSenseResistance),
        };

        let qlsb_mah = 0.085 * (0.05 / r_sense as f32) * (self.m_factor as f32 / 128 as f32);

        Ok(qlsb_mah * raw_value as f32)
    }

    /// Configure the charge prescaler.
    ///
    /// Note: Higher prescaler values result in a higher total charge capacity, but lower
    /// resolution.
    pub fn set_charge_prescaler(&mut self, prescaler: Prescaler) -> Result<(), Error<I2cE>> {
        let mut status = self.read_register(Register::Status)? as u8;

        status.set_bits(3..6, prescaler as u8);
        self.write_register(Register::Status, status as u16)?;
        self.m_factor = 1u8.wrapping_shl(prescaler as u32);

        Ok(())
    }

    /// Set the current battery capacity.
    ///
    /// Arguments:
    /// * `capacity_mah` - The new battery capacity to set the device to in milli-AmpHours.
    pub fn set_capacity(&mut self, capacity_mah: f32) -> Result<(), Error<I2cE>> {
        let r_sense = match self.r_sense {
            Some(r) => r,
            None => return Err(Error::NoSenseResistance),
        };

        let qlsb_mah = 0.085 * (0.05 / r_sense as f32) * (self.m_factor as f32 / 128 as f32);

        // Check that the provided capacity fits.
        let q_steps = capacity_mah / qlsb_mah;

        if q_steps > u16::max_value() as f32 {
            return Err(Error::Size);
        }

        self.write_register(Register::Charge, q_steps as u16)?;

        Ok(())
    }

    /// Configure an alarm threshold for the device.
    ///
    /// Arguments:
    /// * `threshold` - The threshold to configure.
    /// * `low_threshold` - The low value to configure. Units are volts, degrees C, or mAh depending
    /// on the provided threshold.
    /// * `high_threshold` - The high value to configure. Units are volts, degrees C, or mAh
    /// depending on the provided threshold.
    pub fn set_threshold(
        &mut self,
        threshold: Threshold,
        low_threshold: f32,
        high_threshold: f32,
    ) -> Result<(), Error<I2cE>> {
        if high_threshold < low_threshold {
            return Err(Error::Thresholds);
        }

        match threshold {
            Threshold::Voltage => {
                // Convert the voltages to the LSB units used by the chip.
                let low_volt = (low_threshold / 6.0f32) * 0xFFFF as f32;
                let high_volt = (high_threshold / 6.0f32) * 0xFFFF as f32;

                if (low_volt > u16::max_value() as f32)
                    || (high_volt > u16::max_value() as f32)
                    || (low_volt < u16::min_value() as f32)
                    || (high_volt < u16::min_value() as f32)
                {
                    return Err(Error::Range);
                }

                let v_low = low_volt as u16;
                self.write_register(Register::VoltageThresholdLow, v_low.wrapping_shr(8u32))?;

                let v_high = high_volt as u16;
                self.write_register(Register::VoltageThresholdHigh, v_high.wrapping_shr(8u32))?;
            }
            Threshold::Temperature => {
                // Convert the temperatures to the LSB units used by the chip.
                let low_temperature = (low_threshold + 273.15) / 600f32 * 0xFFFF as f32;
                let high_temperature = (high_threshold + 273.15) / 600f32 * 0xFFFF as f32;

                if (low_temperature > u16::max_value() as f32)
                    || (high_temperature > u16::max_value() as f32)
                    || (low_temperature < u16::min_value() as f32)
                    || (high_temperature < u16::min_value() as f32)
                {
                    return Err(Error::Range);
                }

                let t_low = low_temperature as u16;
                self.write_register(Register::TemperatureThresholdLow, t_low.wrapping_shr(8u32))?;

                let t_high = high_temperature as u16;
                self.write_register(
                    Register::TemperatureThresholdHigh,
                    t_high.wrapping_shr(8u32),
                )?;
            }
            Threshold::Charge => {
                // Convert the charges to the LSB units used by the chip.
                let r_sense = match self.r_sense {
                    Some(r) => r,
                    None => return Err(Error::NoSenseResistance),
                };

                let conversion_factor =
                    1f32 / 0.085 * (r_sense / 0.05) * (128 as f32 / self.m_factor as f32);

                let low_charge = low_threshold * conversion_factor;
                let high_charge = high_threshold * conversion_factor;

                if (low_charge > u16::max_value() as f32)
                    || (high_charge > u16::max_value() as f32)
                    || (low_charge < u16::min_value() as f32)
                    || (high_charge < u16::min_value() as f32)
                {
                    return Err(Error::Range);
                }

                let c_low = low_charge as u16;
                self.write_register(Register::ChargeThresholdLow, c_low)?;

                let c_high = high_charge as u16;
                self.write_register(Register::ChargeThresholdHigh, c_high)?;
            }
        };

        Ok(())
    }

    /// Check any potential alarms sensed by the device.
    pub fn check_alarms(&mut self) -> Result<Option<Alarm>, Error<I2cE>> {
        match self.alarm.is_low() {
            Ok(true) => {
                let status = self.read_register(Register::Status)?;

                // Note that only one alarm is returned, so this cascade tree effectively orders the
                // priorities of the alarms.
                for alarm in Alarm::into_enum_iter() {
                    if status.get_bit(alarm as usize) {
                        return Ok(Some(alarm));
                    }
                }

                // The alarm pin was set, but no alarm is registered, so we treat this as no alarm
                // being present.
                Ok(None)
            }
            Ok(false) => Ok(None),
            Err(_) => return Err(Error::Pin),
        }
    }
}
