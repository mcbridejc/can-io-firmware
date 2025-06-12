# can-io-firmware

Software for a simple CAN IO board, with 4 analog inputs and a few GPIOs, which is based on an STM32G0B1CB. It's a work-in-progress.

Also serves as a demo for [zencan](https://github.com/mcbridejc/zencan).

The objects used for communicating on the CAN bus are defined in
[zencan_config.toml](zencan_config.toml).

# Description

## Analog inputs

There are four 12-bit analog input channels, each mapping 0-3.3V to 0-4096 counts.

The analog channels are read at a configurable frequency. Object 0x2100sub0 sets the sample period
in milliseconds. Write this object to change the frequency.

The raw values are written to an array in object 0x2000. You can read this object to get the raw ADC values.

It is also possible to scale the raw analog values into either an i32, or an i16 value. This can be
used to match the units on the inputs to those required by another device. The scaling is configured
via object 0x2101, which is a record defined as:

| Sub Index | Type | Description       |
| --------- | ---- | ----------------- |
| 0         | u8   | Max sub index     |
| 1         | i16  | Scale Numerator   |
| 2         | i16  | Scale Denominator |
| 3         | i16  | Offset            |

The scaled value is computed as scaled = ((raw + offset) * numerator) / denominator. The calculation
is done as an i32, so it should not be possible to overflow.

The scaled value is then written as an i32 to object 0x2001. It is converted to an i16 (with
saturation, so e.g. -100_000 -> -32768, or 100_000 -> 32767) and stored in 0x2002. Either of these
can be read, or mapped to a PDO for periodic transmission.

If any of the output objects are mapped to a TPDO with the transmission type set to 254
(asynchronous/event driven), the PDO will be sent every time the analog channels are read.

# Setup

## Setting a node ID

By default, a freshly programmed device will come up in an unconfigured state (node ID == 255). In
order to activate the device, it has to be assigned a node ID. This can be done using
[`zencan-cli`](https://github.com/mcbridejc/zencan/zencan-cli), and a USB CAN adapter. Currently,
zencan-cli only supports socketcan, so you need a CAN adapter that works with socketcan.

Assuming you have a socketcan interface called 'can0', launch `zencan-cli`:

```shell
zencan-cli can0
```

Then you can run the `lss fastscan` command to search for unconfigured nodes on the bus. If it
works, you should get a report like this, showing the identity (vendor, product, revision, serial)
of the device. The first three numbers are configured in [zencan_config.toml](zencan_config.toml),
and will always be the same. The last one -- the serial -- is generated from the STM32 UID register,
so it will be "random".

```shell
can0>lss fastscan
Found 1 unconfigured nodes
0xcafe 0x408 0x1 0xbab9b0fc
```

Then, you can copy the identifier and use it to assign a node ID to your device:

```shell
can0>lss set-node-id 1 0xcafe 0x408 0x1 0xbab9b0fc
Success!
```

Now, the device should be active and start sending heartbeat messages, which you can view with the
`zencandump` utility. Also, you should be able to read some basic data from the node, using the
`scan` command:

```shell
can0>scan
Node 10: PreOperational
    Identity vendor: CAFE, product: 408, revision: 1, serial: BAB9B0FC
    Device Name: 'can-io'
    Versions: 'v0.0.1' SW, 'rev1' HW
    Last Seen: 0s ago
```

To make the node ID change permanent, you need to issue one more command:

```shell
can0>lss store-config 0xcafe 0x408 0x1 0xbab9b0fc
```

## Configuring and PDO mapping

To configure the device, you must write to the appropriate objects, and then save those object
values to flash so they will be retained through a power cycle.

The easiest way to do this, is by creating a node config file, and loading it with the `load-config`
command in `zencan-cli`.

Here's an example config which sets up the scale, and maps the raw values to TPDO0, which will be
sent on CAN ID 0x200, while mapping the scaled values to TPDO1, which will be sent on CAN ID 0x201.

```toml
# Setup scale factors to offset analog 0 and analog 1 so that 0-4096 becomes -100 to 100
# Scale numerator
[[store]]
index = 0x2101
sub = 1
type = "i16"
value = 100
# Scale denominator
[[store]]
index = 0x2101
sub = 2
type = "i16"
value = 2048
# Offset
[[store]]
index = 0x2101
sub = 3
type = "i16"
value = -2048

# Send all four raw analog values on 0x200
[tpdo.0]
enabled = true
cob = 0x200
transmission_type = 254
mappings = [
    { index=0x2000, sub=1, size=16 },
    { index=0x2000, sub=2, size=16 },
    { index=0x2000, sub=3, size=16 },
    { index=0x2000, sub=4, size=16 },
]

# Send all four scaled i16 analog values on 0x200
[tpdo.1]
enabled = true
cob = 0x201
transmission_type = 254
mappings = [
    { index=0x2002, sub=1, size=16 },
    { index=0x2002, sub=2, size=16 },
    { index=0x2002, sub=3, size=16 },
    { index=0x2002, sub=4, size=16 },
]
```

Then, in `zencan-cli`, write the config parameters to the device:

```shell
can0>load-config 10 example_node_config.toml
```

Start the device operating, by issuing an NMT command:

```shell
can0>nmt start all
```

Now, the device should be sending CAN messages on ID 0x200 an 0x201 with the raw and scaled values,
respectively.

To make the config changes permanent, save the object dictionary to flash:

```shell
can0>save-objects 10
```

## Notes

Requires flip-link: `cargo install flip-link`