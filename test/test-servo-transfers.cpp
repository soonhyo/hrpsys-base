// Host-only PTY tests for transaction exclusion and short writes.
#include <assert.h>
#include <pty.h>
#include <pthread.h>
#include <stdlib.h>
#include "ServoSerial.h"

static int port = -1, mode = 0, writes = 0;
static bool flush_fault = false;
extern "C" int __real_tcflush(int, int);
extern "C" int __wrap_tcflush(int fd, int queue) {
  if (fd == port) {
    assert(queue == TCIFLUSH); // Recovery must never discard queued output.
    if (flush_fault) { errno = EIO; return -1; }
  }
  return __real_tcflush(fd, queue);
}
extern "C" ssize_t __real_write(int, const void *, size_t);
extern "C" ssize_t __wrap_write(int fd, const void *data, size_t size) {
  if (fd == port) {
    ++writes;
    if (mode == 1) {
      if (writes == 1) { errno = EINTR; return -1; }
      if (writes == 2) { errno = EAGAIN; return -1; }
      if (size > 3) size = 3;
    }
    if (mode == 2) return 0;
    if (mode == 3) { errno = EIO; return -1; }
    if (mode == 4) { errno = EAGAIN; return -1; }
    if (mode == 5) {
      if (writes > 1) { errno = EIO; return -1; }
      size = 2;
    }
    if (mode == 6) {
      if (writes > 1) { errno = EIO; return -1; }
      size = 6;
    }
  }
  return __real_write(fd, data, size);
}

static void readAll(int fd, unsigned char *data, int size) {
  while (size) {
    int n = read(fd, data, size);
    assert(n > 0);
    data += n;
    size -= n;
  }
}

static void writeAll(int fd, const unsigned char *data, int size) {
  while (size) {
    int n = write(fd, data, size);
    assert(n > 0);
    data += n;
    size -= n;
  }
}

static void response(int fd, int id) {
  unsigned char packet[26] = {0xFD, 0xDF, 0, 0, 0x2A, 18, 1};
  packet[2] = id;
  packet[7] = id * 10;
  for (int i = 2; i < 25; ++i) packet[25] ^= packet[i];
  writeAll(fd, packet, 26);
}

struct Call { ServoSerial *serial; int id; int result; double value; };
static void *get(void *value) {
  Call &call = *static_cast<Call *>(value);
  call.result = call.serial->getPosition(call.id, &call.value);
  return NULL;
}
static void *off(void *value) {
  Call &call = *static_cast<Call *>(value);
  call.result = call.serial->setTorqueOff(call.id);
  return NULL;
}

int main(int argc, char **argv) {
  assert(argc == 2);
  int master, slave;
  char name[128];
  assert(openpty(&master, &slave, name, NULL, NULL) == 0);
  ServoSerial serial(name);
  close(slave);
  port = serial.fd;
  assert(port >= 0);
  if (!strcmp(argv[1], "concurrent") || !strcmp(argv[1], "get-off")) {
    Call a = {&serial, 3, -99, -99}, b = {&serial, 4, -99, -99};
    pthread_t first, second;
    assert(pthread_create(&first, NULL, get, &a) == 0);
    unsigned char request[9];
    readAll(master, request, 8);
    assert(request[2] == 3);
    writeAll(master, request, 8);
    bool is_off = !strcmp(argv[1], "get-off");
    assert(pthread_create(&second, NULL, is_off ? off : get, &b) == 0);
    fd_set set;
    FD_ZERO(&set);
    FD_SET(master, &set);
    struct timeval wait = {0, 30000};
    // Another command must not be sent while the first GET awaits its return.
    assert(select(master + 1, &set, NULL, NULL, &wait) == 0);
    response(master, 3);
    readAll(master, request, is_off ? 9 : 8);
    assert(request[2] == 4);
    writeAll(master, request, is_off ? 9 : 8);
    if (!is_off) response(master, 4);
    assert(pthread_join(first, NULL) == 0);
    assert(pthread_join(second, NULL) == 0);
    assert(a.result == 0 && a.value == 3);
    assert(b.result == 0 && (is_off || b.value == 4));
  } else if (!strcmp(argv[1], "flush-error") || !strcmp(argv[1], "rx-error-off")) {
    double angle = -99;
    flush_fault = !strcmp(argv[1], "flush-error");
    assert(serial.getPosition(3, &angle) == -1);
    assert(errno == ETIMEDOUT && angle == -99);
    unsigned char request[9];
    readAll(master, request, 8);
    if (!flush_fault) {
      Call call = {&serial, 3, -99, -99};
      pthread_t thread;
      assert(pthread_create(&thread, NULL, off, &call) == 0);
      readAll(master, request, 9);
      assert(request[2] == 3 && request[4] == 0x24 && request[7] == 0);
      writeAll(master, request, 9);
      assert(pthread_join(thread, NULL) == 0 && call.result == 0);
    }
  } else if (!strcmp(argv[1], "partial-off-limit")) {
    mode = 6;
    assert(serial.setPosition(3, 0, 1) == -1);
    assert(errno == EIO && writes == 2);
    unsigned char wire[15];
    readAll(master, wire, 6);
    mode = 0;
    Call call = {&serial, 3, -99, -99};
    pthread_t thread;
    assert(pthread_create(&thread, NULL, off, &call) == 0);
    readAll(master, wire + 6, 9);
    assert(wire[6] == 0xFA && wire[7] == 0xAF && wire[10] == 0x24 && wire[13] == 0);
    writeAll(master, wire + 6, 9); // Adapter echo does not imply servo acceptance.
    assert(pthread_join(thread, NULL) == 0 && call.result == 0);
    // A hypothetical length-driven remote parser consumes the OFF header as
    // the incomplete first command's count. It still waits for more bytes.
    // This model is NOT a measurement of RS301CR firmware behavior.
    assert(wire[0] == 0xFA && wire[1] == 0xAF && wire[5] == 4);
    assert(8 + wire[5] * wire[6] > (int)sizeof(wire));
    puts("KNOWN LIMIT: OFF bytes and echo do not prove remote parser recovery or torque OFF");
  } else if (!strcmp(argv[1], "short-write")) {
    mode = 1;
    Call call = {&serial, 3, -99, -99};
    pthread_t thread;
    assert(pthread_create(&thread, NULL, get, &call) == 0);
    unsigned char request[8];
    readAll(master, request, 8);
    unsigned char expected[8] = {0xFA, 0xAF, 3, 9, 0, 0, 1, 0x0B};
    assert(!memcmp(request, expected, 8));
    writeAll(master, request, 8);
    response(master, 3);
    assert(pthread_join(thread, NULL) == 0);
    assert(call.result == 0 && call.value == 3 && writes == 5);
  } else {
    if (!strcmp(argv[1], "zero-write")) mode = 2;
    else if (!strcmp(argv[1], "write-error")) mode = 3;
    else if (!strcmp(argv[1], "write-timeout")) mode = 4;
    else { assert(!strcmp(argv[1], "partial-error")); mode = 5; }
    assert(serial.setTorqueOff(3) == -1);
    if (mode == 4) assert(errno == ETIMEDOUT);
    if (mode == 5) {
      unsigned char prefix[2];
      readAll(master, prefix, 2);
      assert(prefix[0] == 0xFA && prefix[1] == 0xAF && writes == 2);
    }
    // All convenience setters must preserve a transport failure.
    mode = 3;
    int ids[1] = {3};
    double rad[1] = {0}, sec[1] = {1};
    assert(serial.setReset(3) == -1);
    assert(serial.setPosition(3, 0) == -1);
    assert(serial.setPosition(3, 0, 1) == -1);
    assert(serial.setPositions(1, ids, rad) == -1);
    assert(serial.setPositions(1, ids, rad, sec) == -1);
    assert(serial.setMaxTorque(3, 10) == -1);
    assert(serial.setTorqueOn(3) == -1);
    assert(serial.setTorqueBreak(3) == -1);
  }
  close(master);
  return 0;
}
