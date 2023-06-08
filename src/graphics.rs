#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //spawner.spawn(alive_logger()).unwrap();
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut cec0 = OutputOpenDrain::new(p.PIN_0, Level::High);

    info!("set up i2c ");
    let mut config = Config::default();
    config.frequency = 1_000_000;

    let do_graphics = false;

    /*let mut graphics = if do_graphics {
        let scl = p.PIN_15;
        let sda = p.PIN_14;
        let i2c = I2c::new_blocking(p.I2C1, scl, sda, config);

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display.set_display_on(true).unwrap();

        display.clear(BinaryColor::Off).unwrap();
        display.flush().unwrap();

        let raw_image_data = ImageRawLE::<Rgb565>::new(
            include_bytes!("../../embassy/examples/rp/assets/ferris.raw"),
            86,
        );

        let mut raw_binary = [0u8; 16 * 64];

        for x in 0i32..raw_image_data.size().width as i32 {
            for y in 0i32..raw_image_data.size().height as i32 {
                let pixel = raw_image_data.pixel(Point { x, y }).unwrap();
                if pixel != Rgb565::BLACK {
                    let index = (y * 16 + x / 8) as usize;
                    raw_binary[index] |= 1 << (7 - (x % 8));
                }
            }
        }
        let binary_image_data = ImageRawLE::<BinaryColor>::new(&raw_binary, 128);

        let ferris = Image::new(&binary_image_data, Point::new(0, 0));
        // Display the image
        ferris.draw(&mut display).unwrap();
        display.flush().unwrap();
        Some(display)
    } else {
        None
    };*/
}
