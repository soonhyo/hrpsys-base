// Host-only PTY regression tests. No robot, ROS or hardware serial access.
#include <assert.h>
#include <pty.h>
#include <pthread.h>
#include <stdlib.h>
#include "ServoSerial.h"

struct Peer {
  int fd;
  int cut;
  unsigned char packet[26];
  bool echo;
};

static void writeAll(int fd, const unsigned char *data, int size) {
  while (size > 0) {
    int n = write(fd, data, size);
    if (n < 0 && errno == EINTR) continue;
    assert(n > 0);
    data += n;
    size -= n;
  }
}

static void *reply(void *value) {
  Peer &peer = *static_cast<Peer *>(value);
  if (peer.echo) {
    unsigned char request[8];
    int done = 0;
    while (done < 8) {
      int n = read(peer.fd, request + done, 8 - done);
      assert(n > 0);
      done += n;
    }
    writeAll(peer.fd, request, 3);
    usleep(10000);
    writeAll(peer.fd, request + 3, 5);
  }
  if (peer.cut > 0) writeAll(peer.fd, peer.packet, peer.cut);
  usleep(10000);
  writeAll(peer.fd, peer.packet + peer.cut, 26 - peer.cut);
  return NULL;
}

static double now() {
  struct timespec value;
  assert(clock_gettime(CLOCK_MONOTONIC, &value) == 0);
  return value.tv_sec + value.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
  assert(argc >= 2);
  int master, slave;
  char name[128];
  assert(openpty(&master, &slave, name, NULL, NULL) == 0);
  ServoSerial serial(name);
  assert(serial.fd >= 0);
  close(slave);
  Peer peer = {master, 0, {0xFD, 0xDF, 3, 0, 0x2A, 18, 1}, false};
  peer.packet[7] = 123;
  bool valid = true;
  bool silent = false;
  if (!strcmp(argv[1], "fragment")) {
    assert(argc == 3);
    peer.cut = atoi(argv[2]);
    assert(peer.cut >= 0 && peer.cut <= 26);
  } else if (!strcmp(argv[1], "echo")) {
    peer.echo = true;
    peer.cut = 10;
  } else if (!strcmp(argv[1], "missing-return") || !strcmp(argv[1], "missing-echo")) {
    silent = true;
    valid = false;
  } else {
    valid = false;
    if (!strcmp(argv[1], "id")) peer.packet[2] = 4;
    else if (!strcmp(argv[1], "header")) peer.packet[0] = 0;
    else if (!strcmp(argv[1], "count")) peer.packet[6] = 2;
    else if (!strcmp(argv[1], "length")) peer.packet[5] = 17;
    else if (!strcmp(argv[1], "address")) peer.packet[4] = 0;
    else if (!strcmp(argv[1], "flags")) peer.packet[3] = 0x20;
    else assert(!strcmp(argv[1], "checksum"));
  }
  for (int i = 2; i < 25; ++i) peer.packet[25] ^= peer.packet[i];
  if (!strcmp(argv[1], "checksum")) peer.packet[25] ^= 1;
  pthread_t thread;
  if (!silent) assert(pthread_create(&thread, NULL, reply, &peer) == 0);
  unsigned char output[18];
  memset(output, 0xA5, sizeof(output));
  double start = now();
  if (peer.echo) assert(serial.sendPacket(0xFAAF, 3, 9, 0, 0, 1, NULL) == 8);
  int result;
  if (!strcmp(argv[1], "missing-echo"))
    result = serial.sendPacket(0xFAAF, 3, 9, 0, 0, 1, NULL);
  else result = serial.receivePacket(3, 0x2A, 18, output);
  double elapsed = now() - start;
  assert((result > 0) == valid);
  if (valid) assert(output[0] == 123);
  else for (int i = 0; i < 18; ++i) assert(output[i] == 0xA5);
  if (silent) assert(elapsed >= 0.15 && elapsed < 1.0);
  if (!silent) assert(pthread_join(thread, NULL) == 0);
  close(master);
  return 0;
}
