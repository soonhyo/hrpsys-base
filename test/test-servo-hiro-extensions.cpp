// PTY-only verification of the three existing HIRO extensions. No hardware.
#include <assert.h>
#include <pty.h>
#include <pthread.h>
#include "ServoSerial.h"

struct Call { ServoSerial *serial; int operation; int result; unsigned char rom[30]; };
static void *invoke(void *value) {
  Call &c = *static_cast<Call *>(value);
  if (c.operation == 0) c.result = c.serial->setID(3, 4);
  else if (c.operation == 1) c.result = c.serial->setReverse(3, 1);
  else c.result = c.serial->getROMData(3, c.rom);
  return NULL;
}
static void readAll(int fd, unsigned char *data, int size) {
  while (size) {
    int n = read(fd, data, size); assert(n > 0); data += n; size -= n;
  }
}
int main() {
  for (int operation = 0; operation < 3; ++operation) {
    for (int fail = 0; fail < 2; ++fail) {
      int master, slave; char name[128];
      assert(openpty(&master, &slave, name, NULL, NULL) == 0);
      ServoSerial serial(name); close(slave);
      Call call = {&serial, operation, -99, {0}};
      memset(call.rom, 0xA5, 30);
      pthread_t thread;
      assert(pthread_create(&thread, NULL, invoke, &call) == 0);
      unsigned char packet[9];
      int size = operation == 2 ? 8 : 9;
      readAll(master, packet, size);
      assert(packet[0] == 0xFA && packet[1] == 0xAF && packet[2] == 3);
      if (operation == 0) assert(packet[3] == 0 && packet[4] == 4 && packet[7] == 4);
      if (operation == 1) assert(packet[3] == 0 && packet[4] == 5 && packet[7] == 1);
      if (operation == 2) assert(packet[3] == 3 && packet[4] == 0);
      if (!fail) {
        assert(write(master, packet, size) == size);
        if (operation < 2) {
          readAll(master, packet, 8);
          assert(packet[2] == (operation == 0 ? 4 : 3));
          assert(packet[3] == 0x40 && packet[4] == 0xFF);
          assert(write(master, packet, 8) == 8);
        } else {
          unsigned char reply[38] = {0xFD, 0xDF, 3, 0, 0, 30, 1, 0x10, 0x30};
          for (int i = 2; i < 37; ++i) reply[37] ^= reply[i];
          assert(write(master, reply, 38) == 38);
        }
      }
      assert(pthread_join(thread, NULL) == 0);
      assert(call.result == (fail ? -1 : 0));
      if (operation == 2) {
        if (!fail) assert(call.rom[0] == 0x10 && call.rom[1] == 0x30);
        else for (int i = 0; i < 30; ++i) assert(call.rom[i] == 0xA5);
      }
      fd_set set; FD_ZERO(&set); FD_SET(master, &set);
      struct timeval wait = {0, 10000};
      assert(select(master + 1, &set, NULL, NULL, &wait) == 0); // no retry/extra flash
      close(master);
    }
  }
  puts("PASS: 6 HIRO extension cases (normal + failed echo), PTY only");
}
