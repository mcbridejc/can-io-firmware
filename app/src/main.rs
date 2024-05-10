#![no_std]
#![no_main]
use core::{
    convert::Infallible,
    num::{NonZeroU16, NonZeroU8},
};

use lilos::{exec::Notify, time::Millis};
use pac::RCC;
use stm32_metapac::{self as pac, interrupt};

use cortex_m_rt as _;
use defmt_rtt as _;
use fdcan::{
    config::{DataBitTiming, FdCanConfig},
    frame::TxFrameHeader,
    id::StandardId,
    FdCan, NormalOperationMode,
};
use panic_probe as _;

mod gpio;

use gpio::Pin;

struct FdCan1 {}
struct FdCan2 {}

unsafe impl fdcan::message_ram::Instance for FdCan1 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM1.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan1 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN1.as_ptr() as _;
}
unsafe impl fdcan::message_ram::Instance for FdCan2 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM2.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan2 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN2.as_ptr() as _;
}

#[cortex_m_rt::entry]
fn main() -> ! {
    pac::FLASH
        .acr()
        .modify(|w| w.set_latency(pac::flash::vals::Latency::WS0));

    RCC.cfgr().modify(|w| {
        w.set_sw(pac::rcc::vals::Sw::PLL1_R);
        w.set_hpre(pac::rcc::vals::Hpre::DIV1);
    });

    RCC.apbenr1().modify(|w| {
        w.set_fdcanen(true);
    });

    RCC.gpioenr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpiofen(true);
    });

    // PWM outputs are spread seemingly randomly over 3 timers. Why aren't these on two timers? That
    // is a question for 4-months ago me. I can only imagine I mistakenly re-assigned some pins and
    // got lucky that they all landed on timer outputs?

    let gpios = gpio::gpios();

    let can_rx_pin = gpios.PB8;
    let mut can_tx_pin = gpios.PB9;
    can_tx_pin.set_high();
    can_tx_pin.set_as_output(gpio::Speed::High);

    can_rx_pin.set_as_af(3, gpio::AFType::Input);
    can_tx_pin.set_as_af(3, gpio::AFType::OutputPushPull);

    let mut can = FdCan::new(FdCan1 {}).into_config_mode();
    // Bit timing calculated at http://www.bittiming.can-wiki.info/
    let can_config = FdCanConfig::default()
        .set_automatic_retransmit(false)
        .set_frame_transmit(fdcan::config::FrameTransmissionConfig::ClassicCanOnly)
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: false,
            prescaler: NonZeroU8::new(1).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_nominal_bit_timing(fdcan::config::NominalBitTiming {
            prescaler: NonZeroU16::new(1).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        });

    can.apply_config(can_config);
    let can = can.into_normal();

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_stop(true);
    });

    let mut cp = cortex_m::Peripherals::take().unwrap();
    //p.NVIC.iser[0].modify(|w| w | 1<<)
    // Configure the systick timer for 1kHz ticks at 16MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, 16_000_000);

    configure_adc();

    unsafe { cortex_m::interrupt::enable() };

    lilos::exec::run_tasks(&mut [core::pin::pin!(main_task(can))], lilos::exec::ALL_TASKS);
}

const TRANSMIT_INTERVAL: Millis = Millis(10);

fn configure_adc() {
    pac::RCC.apbenr2().modify(|w| w.set_adcen(true));
    pac::ADC1.cr().modify(|w| {
        w.set_advregen(true);
    });

    // Delay 1/40th of a second for regulator to turn on
    cortex_m::asm::delay(16_000_000 / 40);


    pac::ADC1.cr().modify(|w| w.set_adcal(true));

    // Wait for calibration to complete
    while pac::ADC1.cr().read().adcal() {}

    // Clear ADRDY IRQ
    pac::ADC1.isr().write(|w| w.set_adrdy(true));
    // Enable
    pac::ADC1.cr().modify(|w| w.set_aden(true));

    // Wait for ADRDY signal
    while !pac::ADC1.isr().read().adrdy() {}
    // Clear the flag again
    pac::ADC1.isr().write(|w| w.set_adrdy(true));

    pac::ADC1.cfgr1().modify(|w| {
        w.set_cont(false);
    });

    // Enable oversampling
    pac::ADC1.cfgr2().modify(|w| {
        w.set_ovse(true);
        // 16x oversample
        w.set_ovsr(3);
        // shift by 4 bits
        w.set_ovss(4);
    });

    pac::ADC1.smpr().modify(|w| w.set_smp1(pac::adc::vals::SampleTime::CYCLES39_5));
}

fn read_adc(channel: usize) -> u16 {
    pac::ADC1.chselr().write(|w| w.set_chsel(1<<channel));

    // Clear EOC
    pac::ADC1.isr().write(|w| w.set_eoc(true));

    // Start sampling
    pac::ADC1.cr().modify(|w| w.set_adstart(true));

    // Wait for complete
    while !pac::ADC1.isr().read().eoc() {}

    pac::ADC1.dr().read().regular_data()
}

async fn main_task(mut can: FdCan<FdCan1, NormalOperationMode>) -> Infallible {
    let mut periodic_gate =
        lilos::time::PeriodicGate::new_shift(TRANSMIT_INTERVAL.into(), Millis(0));
    loop {
        periodic_gate.next_time().await;

        let adc_values = [
            read_adc(0),
            read_adc(1),
            read_adc(2),
            read_adc(3),
        ];
        let mut can_buffer = [0u8; 8];
        can_buffer[0..2].copy_from_slice(&adc_values[0].to_le_bytes());
        can_buffer[2..4].copy_from_slice(&adc_values[1].to_le_bytes());
        can_buffer[4..6].copy_from_slice(&adc_values[2].to_le_bytes());
        can_buffer[6..8].copy_from_slice(&adc_values[3].to_le_bytes());

        can.transmit(
            TxFrameHeader {
                len: 8,
                frame_format: fdcan::frame::FrameFormat::Standard,
                id: fdcan::id::Id::Standard(StandardId::new(0x130).unwrap()),
                bit_rate_switching: false,
                marker: None,
            },
            &can_buffer,
        ).ok();
    }
}
