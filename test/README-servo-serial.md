# ServoController serial regression tests

These tests use Linux pseudo terminals, not a robot or physical serial port.
They never run the interactive `testServoSerial` hardware tool.

## Standalone transport tests

Requires Python 3, g++, pthreads and libutil. Run from the repository root:

```sh
python3 test/run-servo-receive.py \
  --header-dir rtc/ServoController --output /tmp/servo-tests --sanitize
```

Omit `--sanitize` if ASan/UBSan is unavailable. `--suite receive`, `transfers`,
`framing` or `initialize` selects a subset. Use a separate output directory for
each candidate. The runner records the header hash, commands and results in
`results.json`; each subprocess has a three-second watchdog.

The production header is compiled as GNU C++98. Tests cover fragmented echo
and return packets, missing responses, invalid framing/identity/checksum,
device error flags, coalesced frames, delayed different-ID replies, bounded
garbage/trickle handling, concurrent GET/GET and GET/OFF, partial writes,
EINTR/EAGAIN, failed input cleanup, failure propagation and initialization.
The initialization test checks 115200 baud in the PTY termios settings; it does
not measure a physical line rate.

Packet dumps are disabled by default: synchronous stdout/stderr output can block
the caller even after serial I/O has completed. Error diagnostics remain enabled.
Configure CMake with `-DSERVO_SERIAL_DEBUG=ON` to restore packet dumps for
diagnosis; direct header consumers can define `SERVO_SERIAL_DEBUG`.
Run the same PTY suite with `--debug-logging` to test that configuration.
The coalesced-frame case checks that packet logging follows the selected mode.
This option does not change protocol validation, baudrate or service signatures.

Known-limit scenarios are reported **separately**, not as fixed regressions:

- After a timeout, an old response with the same ID/address/length can be
  accepted by a later request. There is no request sequence number in these
  packets. Framing checks do not prove freshness.
- After a partial write, transmitting and receiving an OFF echo does not prove
  remote-parser recovery or physical torque-off. This test is a hypothetical
  remote-parser counterexample, not measured servo firmware behavior.

## Controller return-value tests

```sh
python3 test/test-servo-controller-results.py \
  --source rtc/ServoController/ServoController.cpp --output /tmp/servo-results
```

This compiles the actual selected method bodies with a fake bus. It checks
failure returns, scalar angle conversion, full-group calls, all-ID OFF attempts
and ON's stop-on-first-failure behavior. The input-validation follow-up also
checks short/empty/oversized input, subset counts and unknown group IDs with
ASan/UBSan. It is not a CORBA integration test.

## Optional existing-service integration

With an existing OpenRTM/OpenHRP installation and generated hrpsys IDL build:

```sh
bash test/run-servo-legacy.sh "$PWD" /path/to/ros-prefix \
  /path/to/hrpsys-build /tmp/servo-legacy
```

The output directory must not already exist. This compiles the actual Controller
as GNU C++98, links the unchanged servant, and exercises 16 existing services
plus a failed bulk-GET output case. The peer is a PTY; the ORB binds loopback.
No ROS graph, robot nameserver, calibration or physical servo is accessed.
The interactive hardware tool is compiled but **not executed**. On the existing
HIRO backport branch, add `hiro` as the final argument to test its pre-existing
subset-group behavior. The host fixture uses C++11 threads; production code does
not require C++11.

## Scope and compatibility

Public ROS/IDL signatures, normal return conventions and angle conversions are
unchanged. SET transport errors now propagate instead of reporting success.
OFF attempts every configured ID even if one fails, and returns aggregate failure.
The mutex protects a complete high-level getter, not an entire multi-ID service
batch. Separate external calls to the low-level `sendPacket`/`receivePacket`
methods still need caller serialization; direct concurrent use of `fd` is not
protected. Ownership of the mutex makes the class noncopyable; rebuild consumers
rather than mixing old and new C++ objects.

The patch does not add automatic retransmission of whole packets, output flush,
port reopen, `tcdrain`, an OFF-priority/cancellation policy or a permanent fault
latch. Input cleanup only discards queued input; it does not reset remote state.
The 200 ms bounds cover individual I/O stages, not lock acquisition or potentially
blocking diagnostic output. These host tests are not QNX runtime qualification,
a maximum polling-rate benchmark, or a physical OFF safety test.
