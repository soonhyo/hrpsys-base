// Host-only stream framing tests. A PTY peer emulates echo and return packets.
#include <assert.h>
#include <pty.h>
#include <pthread.h>
#include <stdlib.h>
#include <vector>
#include "ServoSerial.h"

typedef std::vector<unsigned char> Bytes;

static void checksum(Bytes &packet) {
  packet.back() = 0;
  for (size_t i = 2; i + 1 < packet.size(); ++i) packet.back() ^= packet[i];
}

static Bytes response(int id, int value, int length = 18) {
  Bytes packet(8 + length, 0);
  packet[0] = 0xFD; packet[1] = 0xDF; packet[2] = id;
  packet[4] = 0x2A; packet[5] = length; packet[6] = 1;
  packet[7] = value & 255; packet[8] = (value >> 8) & 255;
  checksum(packet);
  return packet;
}

static void append(Bytes &stream, const Bytes &packet) {
  stream.insert(stream.end(), packet.begin(), packet.end());
}

static void writeAll(int fd, const Bytes &data) {
  size_t done = 0;
  while (done < data.size()) {
    int count = write(fd, &data[done], data.size() - done);
    if (count < 0 && errno == EINTR) continue;
    assert(count > 0);
    done += count;
  }
}

static Bytes request(int fd) {
  Bytes data(8);
  size_t done = 0;
  while (done < data.size()) {
    int count = read(fd, &data[done], data.size() - done);
    if (count < 0 && errno == EINTR) continue;
    assert(count > 0);
    done += count;
  }
  return data;
}

struct Peer { int fd; const char *mode; int cut; };

static void *serve(void *value) {
  Peer &peer = *static_cast<Peer *>(value);
  Bytes echo = request(peer.fd);
  Bytes good = response(echo[2], 123);
  Bytes stream;
  if (!strcmp(peer.mode, "late-other") || !strcmp(peer.mode, "same-id-limit")) {
    writeAll(peer.fd, echo);
    // The first GET times out before this old response is delivered.
    usleep(230000);
    Bytes second = request(peer.fd);
    if (!strcmp(peer.mode, "late-other")) {
      append(stream, response(echo[2], 777));
      append(stream, echo);
      append(stream, second);
      append(stream, response(second[2], 123));
    } else {
      append(stream, second);
      append(stream, response(echo[2], 777));
      append(stream, response(second[2], 123));
    }
    writeAll(peer.fd, stream);
    return NULL;
  }
  if (!strcmp(peer.mode, "trickle")) {
    echo[2] = 8;
    checksum(echo);
    for (int i = 0; i < 20; ++i) {
      writeAll(peer.fd, echo);
      usleep(20000);
    }
    return NULL;
  }
  if (!strcmp(peer.mode, "noise-limit")) {
    writeAll(peer.fd, Bytes(2048, 0x55));
    return NULL;
  }
  if (!strcmp(peer.mode, "coalesced") || !strcmp(peer.mode, "flags")) {
    append(stream, echo);
    if (!strcmp(peer.mode, "flags")) {
      Bytes error = good;
      error[3] = 0x20;
      checksum(error);
      append(stream, error);
    }
  } else {
    // Old echo and old return are both valid, but belong to another ID.
    stream.push_back(0x09); stream.push_back(0); stream.push_back(0xFD);
    Bytes old_echo = echo;
    old_echo[2] = 8;
    checksum(old_echo);
    append(stream, old_echo);
    append(stream, response(8, 456));
    Bytes broken = echo;
    broken.back() ^= 1;
    append(stream, broken);
    append(stream, echo);
    Bytes wrong_address = good;
    wrong_address[4] = 0;
    checksum(wrong_address);
    append(stream, wrong_address);
    append(stream, response(echo[2], 999, 12));
    broken = good;
    broken.back() ^= 1;
    append(stream, broken);
    // A plausible but incomplete long header must not hide the following frame.
    Bytes incomplete = response(echo[2], 0, 253);
    incomplete.resize(7);
    append(stream, incomplete);
  }
  append(stream, Bytes(good.begin(), good.begin() + peer.cut));
  writeAll(peer.fd, stream);
  usleep(10000);
  writeAll(peer.fd, Bytes(good.begin() + peer.cut, good.end()));
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
  if (!strcmp(argv[1], "buffered") || !strcmp(argv[1], "maximum")) {
    int length = !strcmp(argv[1], "maximum") ? 255 : 18;
    Bytes stream = response(3, 123, length);
    append(stream, response(4, 456, length));
    writeAll(master, stream);
    unsigned char output[255];
    assert(serial.receivePacket(3, 0x2A, length, output) > 0);
    assert(output[0] == 123);
    assert(serial.receivePacket(4, 0x2A, length, output) > 0);
    assert(output[0] == (456 & 255));
    close(master);
    return 0;
  }
  Peer peer = {master, argv[1], argc == 3 ? atoi(argv[2]) : 26};
  assert(peer.cut >= 0 && peer.cut <= 26);
  pthread_t thread;
  assert(pthread_create(&thread, NULL, serve, &peer) == 0);
  double angle = -99;
  double start = now();
  int result = serial.getPosition(3, &angle);
  double elapsed = now() - start;
  if (!strcmp(argv[1], "late-other") || !strcmp(argv[1], "same-id-limit")) {
    assert(result == -1 && angle == -99);
    int id = !strcmp(argv[1], "late-other") ? 4 : 3;
    assert(serial.getPosition(id, &angle) == 0);
    if (id == 4) assert(angle == 12.3);
    else {
      // Known protocol limitation: old/new responses with identical identity
      // are indistinguishable. This is NOT a successful freshness test.
      assert(angle == 77.7);
      puts("KNOWN LIMIT: late same-ID response accepted; freshness unproven");
    }
  } else if (!strcmp(argv[1], "trickle") || !strcmp(argv[1], "noise-limit") ||
             !strcmp(argv[1], "flags")) {
    assert(result == -1 && angle == -99);
    assert(elapsed < 0.8);
    if (!strcmp(argv[1], "trickle")) assert(elapsed >= 0.15);
    if (!strcmp(argv[1], "noise-limit")) assert(errno == EOVERFLOW);
  } else assert(result == 0 && angle == 12.3);
  assert(pthread_join(thread, NULL) == 0);
  close(master);
  return 0;
}
