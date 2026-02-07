#[cfg(test)]
mod tests {
    use embedded_hal_mock::spi::{Mock, Transaction as SpiTransaction};
    use embedded_hal_mock::pin::{Mock as PinMock, Transaction as PinTransaction, State};

    use ls7366::{Action, Encodable, Target};
    use ls7366::ir::InstructionRegister;
    use ls7366::Ls7366;
    use ls7366::str_register;

    #[test]
    fn test_get_count() {
        let expectations = [
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Cntr,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0xDE, 0xAD, 0xBE, 0xEF]),
            // STR read, will return positive sign
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Str,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0x00, 0x00, 0x00, 0b00001010],
            ),
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Cntr,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0xDE, 0xAD, 0xBE, 0xEF]),
            // STR read, will return negative sign
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Str,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0x00, 0x00, 0x00, 0b00001011],
            )
        ];

        let pin_expectations = [
            PinTransaction::set(State::High),  // new_uninit sets high
            // get_count #1: read_register(Cntr)
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
            // get_count #1: get_status -> read_register(Str)
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
            // get_count #2: read_register(Cntr)
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
            // get_count #2: get_status -> read_register(Str)
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
        ];

        let spi = Mock::new(&expectations);
        let cs = PinMock::new(&pin_expectations);
        let mut driver = Ls7366::new_uninit(spi, cs).unwrap();

        let result = driver.get_count().unwrap();

        assert_eq!(result, 0xDEADBEEF);
        assert_eq!(driver.get_count().unwrap(), -0xDEADBEEF);

        let (mut spi, mut cs) = driver.free();
        spi.done();
        cs.done();
    }

    #[test]
    fn test_status_a() {
        let expectations = [
            // STR read, will return positive sign
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Str,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0x00, 0x00, 0x00, 0b00001010],
            ),
            // STR read, will return negative sign
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Str,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0x00, 0x00, 0x00, 0b11110101],
            )
        ];

        let pin_expectations = [
            PinTransaction::set(State::High),  // new_uninit
            // get_status #1
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
            // get_status #2
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
        ];

        let expected_results = [
            str_register::Str {
                cary: false,
                borrow: false,
                compare: false,
                index: false,
                count_enabled: true,
                power_loss: false,
                count_direction: str_register::Direction::Up,
                sign_bit: str_register::SignBit::Positive,
            },
            str_register::Str {
                cary: true,
                borrow: true,
                compare: true,
                index: true,
                count_enabled: false,
                power_loss: true,
                count_direction: str_register::Direction::Down,
                sign_bit: str_register::SignBit::Negative,
            }
        ];
        let spi = Mock::new(&expectations);
        let cs = PinMock::new(&pin_expectations);
        let mut driver = Ls7366::new_uninit(spi, cs).unwrap();

        for payload in expected_results.iter() {
            let result = driver.get_status().unwrap();
            assert_eq!(&result, payload);
        }

        let (mut spi, mut cs) = driver.free();
        spi.done();
        cs.done();
    }

    #[test]
    fn test_status_b() {
        let expectations = [
            // STR read, will return positive sign
            SpiTransaction::transfer(vec![InstructionRegister {
                target: Target::Str,
                action: Action::Read,
            }.encode(), 0x00, 0x00, 0x00, 0x00], vec![0x00, 0x00, 0x00, 0x00, 0b00000100],
            )];

        let pin_expectations = [
            PinTransaction::set(State::High),  // new_uninit
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
        ];

        let spi = Mock::new(&expectations);
        let cs = PinMock::new(&pin_expectations);
        let mut driver = Ls7366::new_uninit(spi, cs).unwrap();
        let result = driver.get_status().unwrap();
        assert_eq!(result, str_register::Str {
            cary: false,
            borrow: false,
            compare: false,
            index: false,
            count_enabled: false,
            power_loss: true,
            count_direction: str_register::Direction::Down,
            sign_bit: str_register::SignBit::Positive,
        });

        let (mut spi, mut cs) = driver.free();
        spi.done();
        cs.done();
    }

    #[test]
    fn test_write_register() {
        let expectations = [
            // Dtr write
            SpiTransaction::write(vec![InstructionRegister {
                target: Target::Dtr,
                action: Action::Write,
            }.encode(), 0xBA, 0xAD, 0xBE, 0xEF],
            ),

            // mdr0 write
            SpiTransaction::write(vec![InstructionRegister {
                target: Target::Mdr0,
                action: Action::Write,
            }.encode(), 0xFD, 0xFD, 0xFD, 0xFD],
            ),
        ];

        let pin_expectations = [
            PinTransaction::set(State::High),  // new_uninit
            // write #1
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
            // write #2
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
        ];

        let spi = Mock::new(&expectations);
        let cs = PinMock::new(&pin_expectations);
        let mut driver = Ls7366::new_uninit(spi, cs).unwrap();

        driver.write_register(Target::Dtr, &vec![0xBA, 0xAD, 0xBE, 0xEF]).unwrap();
        driver.write_register(Target::Mdr0, &vec![0xFD, 0xFD, 0xFD, 0xFD]).unwrap();

        let (mut spi, mut cs) = driver.free();
        spi.done();
        cs.done();
    }
}
