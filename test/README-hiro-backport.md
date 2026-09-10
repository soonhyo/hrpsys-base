# HIRO backport of the common ServoController transport fix

Base: pazeshun/hrpsys-base `hironxjsk-devel`,
`279161b238913a23d3d12de33fcd5900b61e332e`.

Common transport commit: `8efaea8d02ef752994468699677044eb32b06fc6`.

This branch preserves the existing `setID`, `setReverse`, `getROMData` and
interactive test-tool commands. Their changes are limited to transaction
locking and error propagation. The existing subset-group count fix (`len`
rather than the entire configured ID count) is also preserved.

The shared transport implementation matches the upstream PR branch
`fix-servo-serial-transactions`; it does not include the earlier experimental
r6 OFF-priority/fault-latch policy or timing instrumentation.

Run the common host tests described in README-servo-serial.md. For the actual
RTC/servant fixture, pass `hiro` as the final argument. The three existing
extensions additionally have six PTY-only cases (normal and failed echo):

```sh
g++ -std=gnu++98 -pthread -Irtc/ServoController \
  test/test-servo-hiro-extensions.cpp -lutil -o /tmp/test-servo-hiro-extensions
timeout 5s /tmp/test-servo-hiro-extensions
```

This is a Linux host test, not an instruction to execute the interactive
hardware tool or write servo flash. No robot-library installation, QNX rebuild,
RTC restart, motion or physical torque operation is part of these tests.
Native QNX build and controlled hardware qualification are still required
before replacing an installed controller. Keep the existing binary/source
backup and restoration procedure; do not install directly from this document.
