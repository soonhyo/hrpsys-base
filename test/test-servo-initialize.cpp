// Host-only checks of baud setup and initialization failure cleanup.
#include <assert.h>
#include <pty.h>
#include <stdlib.h>
#include "ServoSerial.h"

static int failure = 0, opened_fd = -1, step = 0;
extern "C" int __real_tcgetattr(int, struct termios *);
extern "C" int __real_cfsetospeed(struct termios *, speed_t);
extern "C" int __real_cfsetispeed(struct termios *, speed_t);
extern "C" int __real_tcsetattr(int, int, const struct termios *);
static bool failed(int current) {
  ++step;
  if (current == failure) { errno = EIO; return true; }
  return false;
}
extern "C" int __wrap_tcgetattr(int fd, struct termios *term) {
  opened_fd = fd;
  return failed(1) ? -1 : __real_tcgetattr(fd, term);
}
extern "C" int __wrap_cfsetospeed(struct termios *term, speed_t speed) {
  return failed(2) ? -1 : __real_cfsetospeed(term, speed);
}
extern "C" int __wrap_cfsetispeed(struct termios *term, speed_t speed) {
  return failed(3) ? -1 : __real_cfsetispeed(term, speed);
}
extern "C" int __wrap_tcsetattr(int fd, int action, const struct termios *term) {
  return failed(4) ? -1 : __real_tcsetattr(fd, action, term);
}

int main(int argc, char **argv) {
  assert(argc == 2);
  int master, slave;
  char name[128];
  assert(openpty(&master, &slave, name, NULL, NULL) == 0);
  if (!strcmp(argv[1], "open")) {
    // A child of a terminal device cannot exist; no fixed temporary pathname.
    char missing[160];
    snprintf(missing, sizeof(missing), "%s/missing", name);
    ServoSerial serial(missing);
    assert(serial.fd == -1 && step == 0);
  } else {
    failure = atoi(argv[1]);
    assert(failure >= 0 && failure <= 4);
    ServoSerial serial(name);
    if (failure) {
      assert(serial.fd == -1 && step == failure);
      assert(opened_fd >= 0 && fcntl(opened_fd, F_GETFD) == -1 && errno == EBADF);
    } else {
      assert(serial.fd >= 0 && step == 4);
      struct termios term;
      assert(__real_tcgetattr(serial.fd, &term) == 0);
      assert(cfgetispeed(&term) == B115200 && cfgetospeed(&term) == B115200);
      assert((fcntl(serial.fd, F_GETFL) & O_NONBLOCK) != 0);
    }
  }
  close(slave);
  close(master);
  return 0;
}
