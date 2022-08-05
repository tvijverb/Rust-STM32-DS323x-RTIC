# Rust RTIC microcontroller for STM32F103 reading DS323x Real-Time Clock with I2C

<!-- ABOUT -->
## About the project
This code will create a USB serial port on the STM32F103. The STM32 will send DS323x sensor data every 60 seconds to the host after connecting. The WFI() instruction will put the STM32F103 in sleep mode in between measurements. I did include the alloc-cortex-m library to have access to the format! macro. Removing this would probably be wise since this crate requires the Rust nightly build. Finaly, a big thank you to the RTIC project for creating an awesome piece of software!

<!-- DOCUMENTATION -->
## Documentation

Formal documentation for Rust RTIC [is available here](https://rtic.rs/1/book/en/).


## Getting Started

1. Install the Rust programming language
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
2. Switch to Rust nightly builds due to alloc-cortex-m crate.
```
rustup default nightly
```
3. Install ARM toolchain
```
rustup target install thumbv7m-none-eabi
```
4. Connect your STM32F103 to USB & attach ST-Link probe to the SWD pins on the STM32F103.
5. Connect the DS323x to pins 6 & 7 (I2C_1) of the STM32
6. Build and run the project
```
cargo run --bin stm32 --release
```
7. Connect to the STM32 using USB serial with 9600 or 115200 baud rate and see the measurements coming in.

## Debugging
The current codebase compiles to 50KiB which is pretty close to the 64k limit of the STM32F103. Including the rtt-target debugging code is therefore not possible. If your device has >64KiB, just uncomment all rtt lines in main.rs and you will see debug messages coming in when running:
```
cargo run --bin stm32 --release
```
The second option is to reduce the current codebase, I would suggest removing the alloc-cortex-m code as a starting point.

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

<!-- LICENSE -->
## License

Distributed under the MIT License.


<!-- CONTACT -->
## Contact

Create an issue on github with your contact information and I'll get back to you ;)



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Rust Book](https://doc.rust-lang.org/book/)
* [Rust RTIC book](https://rtic.rs/1/book/en/)
* [STM32F1xx Hardware Abstraction Layer](https://github.com/stm32-rs/stm32f1xx-hal)
