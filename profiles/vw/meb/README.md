# VW MEB platform

This profile was built and tested on a 2025 **VW ID.5** (MEB platform) with a vLinker MC+ (ELM327 v2.2)
adapter, connected through an `AT CRA`-filtered 29-bit CAN session for the vehicle's internal
components, plus 11-bit requests for the body/comfort units. It should work on other MEB-based
cars (ID.3, ID.4, ID.7, Enyaq, etc.) with the same ECUs, but addresses and scaling may need minor
adjustments.

Use at your own risk - the scales, biases and expressions are reverse-engineered from CAN traffic,
not from official documentation.

## Important: the alarm

**Any diagnostic request sent while the car is locked and the alarm is armed will trigger the
alarm.** This is confirmed, reproducible behavior on this platform. Do not poll the car while it is
locked and unattended. See [Safely pausing/resuming polling](#safely-pausingresuming-polling)
below for how this profile/firmware combination avoids that automatically.

Passive CAN listening (no requests sent) does not trigger the alarm, but the OBD port on this
platform is gateway-filtered - passive monitoring picks up no traffic at all, so it isn't a usable
alternative to polling.

## What this profile covers

* **Doors and windows** - open/closed state and window position (0-100%) for all four doors
  (driver, passenger, rear-left, rear-right), read from the individual door control units.
* **Central locking** - lock state.
* **12V battery** - voltage.
* **HV (traction) battery**
  * State of charge, both a calibrated "display" estimate and a gross BMS value
  * Max usable energy capacity (kWh)
  * Voltage, current and power (+ charge / - discharge)
  * Battery temperature
  * An estimated range (km), calculated from SOC x max energy / an assumed efficiency, since no
    direct "range remaining" DID could be found on this platform (see
    [Known limitations](#known-limitations))
* **Climate** - outside/inside temperature, cabin CO2, air recirculation state
* **Charging** - operation mode (used to detect AC charging), auxiliary power draw
* **Diagnostics** - standard DTC reporting

### Known limitations

* **Tailgate and bonnet status are present in the profile but disabled.** The DIDs originally
  believed to report these were found, through repeated open/closed comparisons, to actually be
  slow-changing internal counters unrelated to the physical latch state - not a reliable signal.
  No working DID was found for these on the body control module; they may live on a different,
  not-yet-identified ECU.
* **No charge-limit DID was found.** Extensive diffing of the BMS and charging-unit DIDs while
  changing the charge limit (80% <-> 90%) did not turn up a value that tracked the setting
  reliably.
* The range estimate is a calculation, not a value read from the car, and assumes a fixed
  efficiency (0.18 kWh/km by default) - adjust this constant in the profile's `rangeEstimate`
  expression to match your own driving efficiency.

## Safely pausing/resuming polling

Because any request while locked+armed triggers the alarm, this profile relies on the firmware's
`obdResume`/`obdPause` MQTT buttons and the `obdPolling` binary sensor, combined with a few Home
Assistant automations, to make sure the car is never polled while parked and locked:

* The firmware itself auto-pauses as soon as it sees the driver's door open (a local, 
  connectivity-independent safeguard - it works even if the ESP has no internet access at all).
* Home Assistant automations handle resuming polling again once it's actually safe to do so.

This combination was refined over many real-world test drives; the scenarios below are the ones
that came up in testing.

### 1. Resume when you get in to drive

Uses the Home Assistant Companion App's Bluetooth connectivity sensor on your phone(s) to detect
when you're actually in the car (paired with the car's infotainment/hands-free Bluetooth, not
keyless-entry proximity - this only connects once you're seated with the cabin electronics
powered up, so it won't false-trigger from just walking past the car).

```yaml
template:
  - binary_sensor:
      - name: "Car Driver Seat Occupied (BT)"
        unique_id: car_seat_occupied_bt
        state: >
          {{ '<CAR_BLUETOOTH_MAC>' in states('sensor.<your_phone>_bluetooth_connection')
             or '<CAR_BLUETOOTH_MAC>' in states('sensor.<other_phone>_bluetooth_connection') }}

automation:
  - alias: "OBD - resume polling when getting in"
    trigger:
      - platform: state
        entity_id: binary_sensor.car_seat_occupied_bt
        to: "on"
    action:
      - service: mqtt.publish
        data:
          topic: "obd2mqtt/<device_id>_obdResume/ctrl"
          payload: "PRESS"
```

### 2. Pause when you get out

```yaml
automation:
  - alias: "OBD - pause polling when leaving seat"
    trigger:
      - platform: state
        entity_id: binary_sensor.car_seat_occupied_bt
        to: "off"
    action:
      - service: mqtt.publish
        data:
          topic: "obd2mqtt/<device_id>_obdPause/ctrl"
          payload: "PRESS"
```

### 3. Resume after a brief stop without actually leaving the seat

The firmware's door-open auto-pause also fires if you open the door mid-drive without getting out
(reaching over, a drive-through window, etc.). Since polling is then paused, the door sensor itself
stops updating too (no CAN requests go out at all while paused) - so you can't trigger a "resume"
off the door *closing* again. Instead, trigger off the door *opening* (which was captured right
before the pause kicked in), wait a few seconds, and only resume if you're still confirmed seated:

```yaml
automation:
  - alias: "OBD - resume after a brief door-open while still seated"
    trigger:
      - platform: state
        entity_id: binary_sensor.<driver_door_open_entity_id>
        to: "on"
    action:
      - delay: "00:00:08"
      - condition: state
        entity_id: binary_sensor.car_seat_occupied_bt
        state: "on"
      - service: mqtt.publish
        data:
          topic: "obd2mqtt/<device_id>_obdResume/ctrl"
          payload: "PRESS"
```

### 4. Recover a missed resume after a connectivity gap

If the ESP loses its MQTT/mobile connection and reconnects later (e.g. mid-drive through a dead
zone), a one-shot "resume" trigger from scenario 1 can be missed entirely (it isn't queued, and
once home it did not just magically resume: the automation already fired once in the past). The
`obdResume` button entity goes `unavailable` while the ESP is disconnected, so re-firing on it
becoming available again - gated by still being seated and not already polling - closes that gap:

```yaml
automation:
  - alias: "OBD - resume once ESP reconnects (missed trigger recovery)"
    trigger:
      - platform: state
        entity_id: button.<obd_resume_entity_id>
        from: "unavailable"
    condition:
      - condition: state
        entity_id: binary_sensor.car_seat_occupied_bt
        state: "on"
      - condition: state
        entity_id: binary_sensor.<obd_polling_entity_id>
        state: "off"
    action:
      - service: button.press
        target:
          entity_id: button.<obd_resume_entity_id>
```

Don't use the `obdPolling` sensor's state as a condition on its own without also checking the
button's availability first - if the connection was lost while polling happened to be "on", that
stale value sits cached in Home Assistant for the whole outage and can't be trusted until a fresh
state update has actually arrived.

## Estimating SOC while charging and unreachable

Once the car is locked, asleep and (dis)charging, it stops answering diagnostic requests, so the
live SOC value stops updating. If you have a smart charger/wallbox with its own cumulative energy
meter (e.g. a [Monta](https://monta.com/) charge point), you can estimate the SOC in between real
readings:

1. When polling pauses (car goes to sleep while still plugged in), store the last known SOC and the
   charger's current meter reading:

   ```yaml
   automation:
     - alias: "Car - snapshot SOC and meter reading on pause"
       trigger:
         - platform: state
           entity_id: binary_sensor.<obd_polling_entity_id>
           to: "off"
       action:
         - service: input_number.set_value
           target:
             entity_id: input_number.car_soc_at_pause
           data:
             value: "{{ states('sensor.<your_soc_entity>') | float(0) }}"
         - service: input_number.set_value
           target:
             entity_id: input_number.car_kwh_at_pause
           data:
             value: "{{ states('sensor.<charger_meter_entity>') | float(0) }}"
   ```

2. Calculate an estimated SOC from the delta since then:

   ```yaml
   template:
     - sensor:
         - name: "Car Estimated SOC"
           unit_of_measurement: "%"
           state: >
             {% set kwh_added = states('sensor.<charger_meter_entity>') | float(0)
                - states('input_number.car_kwh_at_pause') | float(0) %}
             {% set battery_kwh = 77.35 %}
             {% set efficiency = 0.92 %}
             {{ (states('input_number.car_soc_at_pause') | float(0)
                + (kwh_added * efficiency / battery_kwh * 100)) | round(1) }}
   ```

   Adjust `battery_kwh` to your pack's usable capacity (from this profile's `hvMaxEnergy` sensor)
   and `efficiency` to your charger/cable losses.

3. **If you share the charger with another car**, any energy it draws will otherwise be wrongly
   counted as going into this car. Fix it by continuously re-baselining the stored meter reading
   whenever the *other* car's charge cable is connected, so none of its energy ever counts:

   ```yaml
   automation:
     - alias: "Car - ignore other car's charging on the shared meter"
       trigger:
         - trigger: state
           entity_id:
             - sensor.<charger_meter_entity>
       condition:
         - condition: state
           entity_id: binary_sensor.<other_car_charge_cable_entity>
           state: "on"
       action:
         - target:
             entity_id: input_number.car_kwh_at_pause
           data:
             value: "{{ states('sensor.<charger_meter_entity>') | float(0) }}"
           action: input_number.set_value
   ```

   This only works cleanly if the charger can only serve one car at a time (true for most single
   home wallboxes) - if both cars could draw from the same meter simultaneously, this car's real
   charging progress during an overlap would also be missed.
