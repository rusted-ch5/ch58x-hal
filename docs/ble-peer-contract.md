# Independent peer acceptance contract

Implement the nRF52840 test peer locally with an open BLE stack. Its firmware
source and binaries are intentionally not included. This is a test behavior
specification, not a claim that this Rust snapshot passed RF acceptance.

1. Record exact CH585/nRF52840 board and probe serials, fresh USB paths, power,
   common ground, both source revisions/ELF hashes, TX power, PHY, interval,
   timeout, channel map, box geometry and capture placement. Lease each exact
   target/probe pair. Keep probes detached throughout each RF assertion window.
2. Fully program/verify/reset CH585 before allowing the Central to connect.
   Downloading Nordic firmware starts it; explicitly hold/reset it when needed
   to prevent a connection to the old CH image. Do not halt during an RF run.
3. Discover legacy ADV_IND on 37/38/39; actively request and validate SCAN_RSP,
   including address type and contents. Check malformed/wrong-address requests
   do not cause a response or connection. Central scanning by CH585 is outside
   this snapshot's backend scope.
4. Establish an unencrypted LE 1M link at 30 ms, latency zero, timeout 2 seconds.
   Use an independently supplied ATT/GATT Host on CH585 to test discovery, long
   reads, CCCD enable/disable and fragmented notifications. Test payloads must
   carry sequence/member identifiers and a byte oracle; count complete delivered
   application values separately from HCI fragments and on-air retransmissions.
5. Require at least 30 seconds of monotonically sequenced notifications. During
   the connection request 30 -> 15 -> 30 ms intervals and full map ->
   `00 ff ff 00 00` -> full map. After each instant require both peers to report
   matching parameters and at least four further intact notification pairs.
6. Inject duplicate/retried PDUs, invalid CRC, stale/passed instants and control
   collisions. Check that an unacknowledged owner is retained, new RX is delivered
   once, malformed PDUs do not advance state and response deadlines cannot be
   restarted by unrelated traffic. Encryption requests must be rejected and the
   LE feature response must not advertise encryption.
7. Disable CCCD, cleanly disconnect, advertise again without resetting CH585,
   reconnect and repeat the application integrity check. Record all close reasons.
8. End the independent peer assertions before the single final CH snapshot.
   Retain raw counters, trace/PCAP where used, source/firmware hashes and exact
   startup order. A debugger interruption invalidates continuity evidence.

Separate gates: host tests; cross-build/link/size; verified flash; boot/clock/IRQ;
RF discovery/connection; ATT/GATT; integrity/reconnect/endurance. No security or
HIDS claim follows from this plaintext test. Metal-box observations apply only
to that recorded placement; they are not open-space range or sensitivity results.
