//! Provides mock hardware for development platforms lacking hardware.
//! Additionally, provides common setup and initialization if the platform doesn't need anything special.
//!
//! This allows for easy testing of host to EC comms.
pub mod battery;
pub mod thermal;
pub mod time_alarm;

crate::impl_relay_handler!(
    MockOdpRelayHandler,
    battery_service::Service<'static, 1>,
    crate::mock::thermal::ThermalService
);

/// Initialize mock embedded services.
pub async fn init(
    spawner: embassy_executor::Spawner,
) -> MockOdpRelayHandler<impl embedded_services::event::Receiver<u8>> {
    embedded_services::info!("Initializing mock services...");
    embedded_services::init().await;

    let thermal = thermal::init(spawner).await;
    let battery = battery::init(spawner).await;
    let tas = time_alarm::init(spawner).await;

    // Build a thermal event receiver by muxing sensor + fan event channels
    let thermal_rx = embedded_services::event::MuxReceiver::new()
        .with(thermal::SENSOR_EVENT_CHANNEL.receiver(), |e| {
            thermal_service_interface::Event::Sensor(0, e)
        })
        .with(thermal::FAN_EVENT_CHANNEL.receiver(), |e| {
            thermal_service_interface::Event::Fan(0, e)
        });

    MockOdpRelayHandler::new(
        battery_service_relay::BatteryServiceRelayHandler::new(battery),
        thermal_service_relay::ThermalServiceRelayHandler::new(thermal),
        time_alarm_service_relay::TimeAlarmServiceRelayHandler::new(tas),
        embedded_services::event::NeverReceiver::new(),
        thermal_rx,
        embedded_services::event::NeverReceiver::new(),
    )
}
