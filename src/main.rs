//! Firmware for CAN-IO board
//!
//! The CAN-IO board has 4 analog inputs, and 6 controllable GPIOs, and is designed to make these
//! values available to other nodes on a CAN bus. Currently, the firmware only supports the 4 analog
//! inputs.
#![no_std]
#![no_main]

use core::{
    convert::Infallible,
    num::{NonZeroU8, NonZeroU16},
    pin::pin,
    time::Duration,
};

use lilos::{exec::Notify, time::Millis};
use persist::SectionUpdate;
use stm32_metapac::{self as pac, RCC, interrupt};

use fdcan::{
    FdCan, FdCanControl, Fifo0, NormalOperationMode, Rx,
    config::{DataBitTiming, FdCanConfig, GlobalFilter},
    filter::{StandardFilter, StandardFilterSlot},
};

use cortex_m_rt as _;
//use defmt_rtt as _;
use panic_probe as _;
use rtt_target::{self as _, rtt_init, set_defmt_channel};
use zencan::{OBJECT2000, OBJECT2001, OBJECT2002, OBJECT2101};
use zencan_node::{
    Node,
    common::{NodeId, objects::ObjectRawAccess},
    restore_stored_objects,
};

fn get_serial() -> u32 {
    let mut ctx = md5::Context::new();
    ctx.consume(pac::UID.uid(0).read().to_le_bytes());
    ctx.consume(pac::UID.uid(1).read().to_le_bytes());
    ctx.consume(pac::UID.uid(2).read().to_le_bytes());
    let digest = ctx.compute();
    u32::from_le_bytes(digest.0[0..4].try_into().unwrap())
}

mod flash;
mod persist;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

mod gpio;
use gpio::Pin;

use crate::flash::Stm32g0Flash;

struct FdCan1 {}
#[allow(dead_code)]
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

//static mut CAN_RX: Option<&mut dyn Receive> = None;
static mut CAN_RX: Option<Rx<FdCan1, NormalOperationMode, Fifo0>> = None;
static mut CAN_CTRL: Option<FdCanControl<FdCan1, NormalOperationMode>> = None;
static CAN_NOTIFY: Notify = Notify::new();
static mut FLASH: Option<Stm32g0Flash> = None;

enum FlashSections {
    NodeConfig = 1,
    Objects = 2,
    Unknown = 256,
}

impl From<u8> for FlashSections {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::NodeConfig,
            2 => Self::Objects,
            _ => Self::Unknown,
        }
    }
}

#[allow(static_mut_refs)]
fn store_objects(reader: &mut dyn embedded_io::Read<Error = Infallible>, size: usize) {
    // Safety: No other threads (i.e. IRQs) will use flash)
    let mut flash = unsafe { FLASH.as_mut().unwrap().unlock() };
    if persist::update_sections(
        &mut flash,
        &mut [SectionUpdate {
            section_id: FlashSections::Objects as u8,
            data: persist::UpdateSource::Reader((reader, size)),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing objects to flash");
    }
}

#[allow(static_mut_refs)]
fn store_node_config(id: &NodeId) {
    let mut flash = unsafe { FLASH.as_mut().unwrap().unlock() };
    let data = [id.raw()];
    if persist::update_sections(
        &mut flash,
        &mut [SectionUpdate {
            section_id: FlashSections::NodeConfig as u8,
            data: persist::UpdateSource::Slice(&data),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing node config to flash");
    }
}

fn notify_can_task() {
    CAN_NOTIFY.notify();
}

/// Setup the ADC for reading the analog channels
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

    pac::ADC1
        .smpr()
        .modify(|w| w.set_smp1(pac::adc::vals::SampleTime::CYCLES39_5));
}

fn read_adc(channel: usize) -> u16 {
    pac::ADC1.chselr().write(|w| w.set_chsel(1 << channel));

    // Clear EOC
    pac::ADC1.isr().write(|w| w.set_eoc(true));

    // Start sampling
    pac::ADC1.cr().modify(|w| w.set_adstart(true));

    // Wait for complete
    while !pac::ADC1.isr().read().eoc() {}

    pac::ADC1.dr().read().regular_data()
}

fn read_saved_node_id(flash: &mut Stm32g0Flash) -> NodeId {
    if let Some(sections) = persist::load_sections(&flash.unlock()) {
        for s in sections {
            let section_type = FlashSections::from(s.section_id);
            match section_type {
                FlashSections::NodeConfig => {
                    if s.data.len() > 0 {
                        match NodeId::try_from(s.data[0]) {
                            Ok(node_id) => return node_id,
                            Err(_) => {
                                defmt::error!("Read invalid node_id {} from flash", s.data[0]);
                                break;
                            }
                        }
                    } else {
                        defmt::error!("Found zero length NodeConfig section");
                    }
                }
                _ => continue,
            }
        }
    }

    NodeId::Unconfigured
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // Check out peripherals from the runtime.
    let mut cp = cortex_m::Peripherals::take().unwrap();

    pac::FLASH
        .acr()
        .modify(|w| w.set_latency(pac::flash::vals::Latency::WS0));

    RCC.cfgr().modify(|w| {
        w.set_sw(pac::rcc::vals::Sw::HSI);
        w.set_hpre(pac::rcc::vals::Hpre::DIV1);
    });

    RCC.apbenr1().modify(|w| {
        w.set_fdcanen(true);
    });

    // Enable clock to all the GPIO ports
    RCC.gpioenr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpiofen(true);
    });

    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                name: "defmt",
            }
        }
    };

    set_defmt_channel(channels.up.0);

    // The last two pages of flash are set aside for non-volatile storage
    // Each page is 2kB
    const FLASH_PAGE_A: usize = 62;
    const FLASH_PAGE_B: usize = 63;
    let mut flash = flash::Stm32g0Flash::new(pac::FLASH, FLASH_PAGE_A, FLASH_PAGE_B);

    let gpios = crate::gpio::gpios();

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
        })
        .set_global_filter(GlobalFilter {
            handle_standard_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            handle_extended_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            reject_remote_standard_frames: false,
            reject_remote_extended_frames: false,
        });

    can.apply_config(can_config);
    let mut can = can.into_normal();

    can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
    can.enable_interrupt_line(fdcan::config::InterruptLine::_1, true);
    can.set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );

    let (can_ctrl, can_tx, can_rx0, _can_rx1) = can.split();

    unsafe {
        CAN_RX = Some(can_rx0);
        CAN_CTRL = Some(can_ctrl);
    }

    configure_adc();

    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);

    let node_id = read_saved_node_id(&mut flash);

    let initnode = Node::init(
        node_id,
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );

    // Use the UID register to set a unique serial number
    zencan::OBJECT1018.set_serial(get_serial());

    // Load persistent data from flash It's important to do this after calling Node::init,, because
    // some of the restored objects (e.g. PDO config) require that callbacks be registered before
    // they can be accessed.
    if let Some(sections) = persist::load_sections(&flash.unlock()) {
        for s in sections {
            let section_type = FlashSections::from(s.section_id);
            match section_type {
                FlashSections::NodeConfig => (), // Ignore
                FlashSections::Objects => {
                    defmt::info!("Loaded objects from flash");
                    restore_stored_objects(&zencan::OD_TABLE, s.data);
                }
                FlashSections::Unknown => {
                    defmt::warn!("Found unrecognized flash section {}", s.section_id);
                }
            }
        }
    } else {
        defmt::info!("No data found in flash");
    }

    // Store the flash object in static so it can be access by callbacks
    unsafe {
        FLASH = Some(flash);
    }

    let mut node = initnode.finalize();

    // Register handlers for saving node data to flash
    node.register_store_objects(&store_objects);
    node.register_store_node_config(&store_node_config);

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_stop(true);
    });
    pac::RCC.ahbenr().modify(|w| w.set_dma1en(true));

    // Set up the OS timer. This can be done before or after starting the
    // scheduler, but must be done before using any timer features.
    lilos::time::initialize_sys_tick(&mut cp.SYST, 16_000_000);

    unsafe { cortex_m::interrupt::enable() };
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM16_FDCAN_IT0) };

    // Run our four tasks in parallel. The final parameter specifies which tasks
    // to poll on the first iteration.
    lilos::exec::run_tasks(
        &mut [pin!(can_task(node, can_tx)), pin!(main_task())],
        lilos::exec::ALL_TASKS,
    )
}

fn zencan_to_fdcan_header(msg: &zencan_node::common::CanMessage) -> fdcan::frame::TxFrameHeader {
    let id: fdcan::id::Id = match msg.id() {
        zencan_node::common::messages::CanId::Extended(id) => {
            fdcan::id::ExtendedId::new(id).unwrap().into()
        }
        zencan_node::common::messages::CanId::Std(id) => {
            fdcan::id::StandardId::new(id).unwrap().into()
        }
    };
    fdcan::frame::TxFrameHeader {
        len: msg.dlc,
        frame_format: fdcan::frame::FrameFormat::Standard,
        id,
        bit_rate_switching: false,
        marker: None,
    }
}

/// A task for running the CAN node processing periodically, or when triggered by the CAN receive
/// interrupt to run immediately
async fn can_task(
    mut node: Node,
    mut can_tx: fdcan::Tx<FdCan1, NormalOperationMode>,
) -> Infallible {
    let epoch = lilos::time::TickTime::now();
    loop {
        lilos::time::with_timeout(Duration::from_millis(10), CAN_NOTIFY.until_next()).await;
        let time_us = epoch.elapsed().0 * 1000;
        node.process(time_us, &mut |msg| {
            let header = zencan_to_fdcan_header(&msg);
            if let Err(_) = can_tx.transmit(header, msg.data()) {
                defmt::error!("Error transmitting CAN message");
            }
        });
    }
}

/// Task for periodically reading the sensors
async fn main_task() -> Infallible {
    const MAX_PERIOD: u32 = 5000;
    let mut read_interval = zencan::OBJECT2100.get_value().max(MAX_PERIOD);
    let mut periodic_gate =
        lilos::time::PeriodicGate::new_shift(Millis(read_interval as u64), Millis(0));

    loop {
        periodic_gate.next_time().await;

        let adc_values = [read_adc(0), read_adc(1), read_adc(2), read_adc(3)];

        for i in 0..4 {
            let raw_value = adc_values[i];
            OBJECT2000.set(i, adc_values[i]).unwrap();
            let scale_num = OBJECT2101.get_scale_numerator() as i32;
            let scale_den = OBJECT2101.get_scale_denominator() as i32;
            let offset = OBJECT2101.get_offset() as i32;
            let scaled_value = ((raw_value as i32 + offset).saturating_mul(scale_num)) / scale_den;

            OBJECT2001.set(i, scaled_value as i32).unwrap();
            OBJECT2002
                .set(
                    i,
                    scaled_value.clamp(i16::MIN as i32, i16::MAX as i32) as i16,
                )
                .unwrap();

            OBJECT2000.set_event_flag(i as u8 + 1).unwrap();
            OBJECT2001.set_event_flag(i as u8 + 1).unwrap();
            OBJECT2002.set_event_flag(i as u8 + 1).unwrap();
        }

        // Notify can task that there is something new to process
        CAN_NOTIFY.notify();

        let new_interval = zencan::OBJECT2100.get_value();
        if new_interval != read_interval {
            read_interval = new_interval;
            periodic_gate = lilos::time::PeriodicGate::new_shift(
                Millis(read_interval as u64),
                Millis(read_interval as u64),
            );
        }
    }
}

#[allow(static_mut_refs)]
#[interrupt]
fn TIM16_FDCAN_IT0() {
    // safety: Accept for during boot-up when we set it, we only access in this interrupt
    let ctrl = unsafe { CAN_CTRL.as_mut().unwrap() };
    let rx = unsafe { CAN_RX.as_mut().unwrap() };

    ctrl.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

    let mut buffer = [0u8; 8];

    while let Ok(msg) = rx.receive(&mut buffer) {
        // ReceiveOverrun::unwrap() cannot fail
        let msg = msg.unwrap();

        let id = match msg.id {
            fdcan::id::Id::Standard(standard_id) => {
                zencan_node::common::messages::CanId::std(standard_id.as_raw())
            }
            fdcan::id::Id::Extended(extended_id) => {
                zencan_node::common::messages::CanId::extended(extended_id.as_raw())
            }
        };
        let msg = zencan_node::common::messages::CanMessage::new(id, &buffer[..msg.len as usize]);
        // Ignore error -- as an Err is returned for messages that are not consumed by the node
        // stack
        zencan::NODE_MBOX.store_message(msg).ok();
    }
}
