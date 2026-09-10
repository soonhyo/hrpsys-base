// Host integration test: actual RTC + unchanged legacy servant + PTY driver.
// No ROS graph, robot, nameserver or new stop/ack service is used by the client.
// Link the candidate Controller and unchanged servant directly; do not define NDEBUG.
#include "ServoController.h"
#include <pty.h>
#include <poll.h>
#include <cassert>
#include <cmath>
#include <atomic>
#include <mutex>
#include <thread>
#include <iostream>
#include <cerrno>

static thread_local bool reject_bus_lock = false;
extern "C" int __real_pthread_mutex_lock(pthread_mutex_t *);
extern "C" int __wrap_pthread_mutex_lock(pthread_mutex_t *mutex) {
  if (reject_bus_lock) {
    reject_bus_lock = false;
    return EDEADLK;
  }
  return __real_pthread_mutex_lock(mutex);
}
static void exact(int fd, unsigned char *p, size_t n) {
  while (n) {
    pollfd event = {fd, POLLIN, 0}; assert(poll(&event, 1, 2000) > 0);
    int r = read(fd, p, n); assert(r > 0); p += r; n -= r;
  }
}
class LegacyComponent : public ServoController {
public:
  LegacyComponent(RTC::Manager *m) : ServoController(m) {}
  ServoControllerService_impl &legacy() { return m_service0; }
  void detachLegacyPort() { removePort(m_ServoControllerServicePort); }
};
struct Peer {
  int master, slave;
  char path[128];
  std::atomic<bool> quit;
  std::mutex mutex;
  std::vector<std::vector<unsigned char> > packets;
  std::thread thread;
  Peer() : quit(false) {
    assert(!openpty(&master, &slave, path, NULL, NULL));
  }
  void start() { thread = std::thread([this]() { serve(); }); }
  ~Peer() { quit = true; thread.join(); close(slave); close(master); }
  std::vector<unsigned char> last() {
    std::lock_guard<std::mutex> lock(mutex); assert(!packets.empty()); return packets.back();
  }
  void serve() {
    while (!quit) {
      pollfd event = {master, POLLIN, 0}; int r = poll(&event, 1, 10);
      assert(r >= 0); if (!r) continue;
      unsigned char h[7]; exact(master, h, 7);
      std::vector<unsigned char> p(8 + h[5]*h[6]);
      memcpy(&p[0], h, 7); exact(master, &p[7], p.size()-7);
      { std::lock_guard<std::mutex> lock(mutex); packets.push_back(p); }
      assert(write(master, &p[0], 3) == 3); // transport must accumulate short reads
      usleep(1000);
      assert(write(master, &p[3], p.size()-3) == (int)p.size()-3);
      if (h[3] == 0x09 || h[3] == 0x0B) {
        const bool limit = h[3] == 0x0B;
        std::vector<unsigned char> a(limit ? 20 : 26, 0);
        a[0]=0xFD; a[1]=0xDF; a[2]=h[2]; a[4]=limit?0x1E:0x2A;
        a[5]=a.size()-8; a[6]=1;
        if (limit) a[12]=50;
        else { a[7]=123; a[9]=2; a[11]=3; a[13]=4; a[15]=35; a[17]=0xBC; a[18]=2; }
        for (size_t i=2; i<a.size()-1; ++i) a.back() ^= a[i];
        assert(write(master, &a[0], a.size()) == (int)a.size());
      }
    }
  }
};
static int word(const std::vector<unsigned char> &p, size_t offset) {
  return (short)(p[offset] | (p[offset+1] << 8));
}
int main() {
  const char *options[] = {"testServoLegacyUsage", "-f", "/dev/null", "-o", "naming.enable:NO",
    "-o", "manager.corba_servant:NO", "-o", "logger.enable:NO",
    "-o", "manager.shutdown_on_nortcs:NO",
    "-o", "corba.endpoints:127.0.0.1:0"};
  int argc=sizeof(options)/sizeof(options[0]);
  RTC::Manager *manager=RTC::Manager::init(argc, const_cast<char **>(options));
  assert(manager->activateManager());
  manager->runManager(true);
  {
    Peer peer;
    LegacyComponent component(manager);
    component.getProperties()["servo.devname"]=peer.path;
    component.getProperties()["servo.id"]="2,3,4,5,6,7,8,9";
    component.getProperties()["servo.offset"]="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8";
    component.getProperties()["servo.dir"]="1,1,-1,-1,1,1,-1,-1";
    assert(component.onInitialize()==RTC::RTC_OK);
    peer.start();
    ServoControllerService_impl &api=component.legacy();
    double v=0;
    for (int id=2; id<=9; ++id) {
      assert(api.getJointAngle(id,v));
      assert(fabs(v-(12.3-(id-1)*0.1*180/M_PI))<1e-8);
      assert(api.setMaxTorque(id,50));
    }
    assert(api.servoOn());
    assert(api.setJointAngle(4,10,1.5));
    auto p=peer.last();
    assert(p[2]==4 && p[4]==0x1E && word(p,9)==150);
    // Scalar degrees + offset; group radians * dir + offset (legacy semantics).
    assert(word(p,7)==(short)((10*M_PI/180+0.3)*180/M_PI*10));
    OpenHRP::ServoControllerService::dSequence all;
    all.length(8); for (unsigned int i=0;i<8;++i) all[i]=0.1*(i+1);
    assert(api.setJointAngles(all,1));
    p=peer.last(); assert(p[2]==0 && p[5]==5 && p[6]==8);
    for (int i=0;i<8;++i) assert(p[7+i*5]==i+2);
    OpenHRP::ServoControllerService::iSequence group;
    const int group_ids[] = {9,6,8,7,2,3,4,5};
#ifdef SERVO_HIRO_BACKPORT
    const int group_size = 4;
#else
    // Upstream has a separate pre-existing subset group count defect.
    const int group_size = 8;
#endif
    group.length(group_size);
    for(int i=0;i<group_size;++i) group[i]=group_ids[i];
    assert(api.addJointGroup("legacy_hand",group));
    OpenHRP::ServoControllerService::dSequence angles;
    angles.length(group_size); for(int i=0;i<group_size;++i) angles[i]=0.2;
    assert(api.setJointAnglesOfGroup("legacy_hand",angles,2));
    p=peer.last(); assert(p[2]==0 && p[6]==group_size);
    for(int i=0;i<group_size;++i) {
      const int id=group[i], dir=(id==4 || id==5 || id==8 || id==9)?-1:1;
      assert(p[7+i*5]==id && word(p,10+i*5)==200);
      assert(word(p,8+i*5)==(short)((0.2*dir+(id-1)*0.1)*180/M_PI*10));
    }
    assert(api.getDuration(3,v) && v==20);
    assert(api.getSpeed(3,v) && v==3);
    short percentage=0; assert(api.getMaxTorque(3,percentage) && percentage==50);
    assert(api.getTorque(3,v) && v==4);
    assert(api.getTemperature(3,v) && v==35);
    assert(api.getVoltage(3,v) && v==7); // retain existing integer division
    assert(api.servoOff());
    // No new acknowledgement between OFF and any of these legacy operations.
    OpenHRP::ServoControllerService::dSequence_var measured;
    assert(api.getJointAngles(measured.out()) && measured->length()==8);
    for(unsigned int i=0;i<8;++i) assert(measured[i]==12.3); // existing bulk units
    // Admission can reject a call before any bus I/O. The servant still needs
    // a valid output sequence to marshal its false result.
    size_t before_rejection;
    { std::lock_guard<std::mutex> lock(peer.mutex); before_rejection = peer.packets.size(); }
    OpenHRP::ServoControllerService::dSequence_var rejected_values;
    reject_bus_lock = true;
    bool rejected_ok = api.getJointAngles(rejected_values.out());
    reject_bus_lock = false;
    assert(!rejected_ok);
    assert(rejected_values.operator->() != NULL);
    assert(rejected_values->length() == 8); // original bulk allocation precedes per-ID reads
    { std::lock_guard<std::mutex> lock(peer.mutex); assert(peer.packets.size() == before_rejection); }
    assert(api.setJointAnglesOfGroup("legacy_hand",angles,1));
    assert(api.setReset(3));
    // removeJointGroup has an unrelated missing-return defect; not exercised here.
    assert(api.servoOn());
    assert(api.servoOff());
    component.detachLegacyPort(); // normally performed by the RTC lifecycle
    assert(component.onFinalize()==RTC::RTC_OK);
    PortableServer::ObjectId_var objectId=manager->getPOA()->servant_to_id(&component);
    manager->getPOA()->deactivate_object(objectId);
  }
  manager->shutdown();
  std::cout << "PASS: actual RTC/legacy servant/PTY, 16 existing services + failed GET output; no ACK/rearm, scalar and group units/order preserved\n";
}
