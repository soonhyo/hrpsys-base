#!/usr/bin/env python3
"""Compile unchanged method bodies from the selected Controller.cpp with a fake bus.

Exercises return propagation, OFF iteration and existing conversions, not CORBA dispatch.
No robot, ROS node or nameserver is started.
"""
import argparse
from pathlib import Path
import re
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", required=True, type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
source = args.source.read_text()
names = ["setJointAngle", "setJointAngles", "setJointAnglesOfGroup", "getJointAngle",
         "setMaxTorque", "setReset", "servoOn", "servoOff"]
bodies, declarations = [], []
for name in names:
    match = re.search(r"bool ServoController::" + name + r"\([^\n]*\)\n\{", source)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    bodies.append(source[match.start():pos])
    declarations.append(source[match.start():match.end()-2].replace("ServoController::", "") + ";")

prefix = r"""
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace std;
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAILED line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
namespace OpenHRP {
struct ServoControllerService {
  struct dSequence {
    vector<double> data;
    unsigned length() const { return data.size(); }
    const double *get_buffer() const { return &data[0]; }
  };
};
}
struct FakeSerial {
  vector<int> calls;
  int fail_id;
  bool group_fail;
  double rad;
  FakeSerial() : fail_id(-2), group_fail(false), rad(0) {}
  int write(int id) { calls.push_back(id); return (id==fail_id || fail_id==-1) ? -1 : 0; }
  int setPosition(int id,double angle,double) { rad=angle; return write(id); }
  int setPositions(int n,int *ids,double *,double *) {
    calls.assign(ids,ids+n); return group_fail?-1:0;
  }
  int setMaxTorque(int id,short) { return write(id); }
  int setReset(int id) { return write(id); }
  int setTorqueOn(int id) { return write(id); }
  int setTorqueOff(int id) { return write(id); }
  int getPosition(int id,double *angle) {
    if (write(id)<0) return -1; *angle=30; return 0;
  }
};
struct ServoController {
  FakeSerial *serial;
  vector<int> servo_id;
  vector<double> servo_offset,servo_dir;
  map<string,vector<int> > joint_groups;
  struct { string instance_name; } m_profile;
"""
main = r"""
int main() {
  FakeSerial bus;
  ServoController c;
  c.serial=&bus;
  for(int id=2;id<=9;++id) {
    c.servo_id.push_back(id);
    c.servo_offset.push_back(0.1);
    c.servo_dir.push_back(id%2 ? -1 : 1);
  }
  double angle=77;
  CHECK(c.getJointAngle(4,angle));
  CHECK(fabs(angle-(30-0.1*180/M_PI))<1e-9);
  CHECK(c.setJointAngle(4,10,1));
  CHECK(fabs(bus.rad-(10*M_PI/180+0.1))<1e-9);
  bus.fail_id=4;
  CHECK(!c.setJointAngle(4,10,1));
  CHECK(!c.setMaxTorque(4,50));
  CHECK(!c.setReset(4));
  angle=77;
  CHECK(!c.getJointAngle(4,angle) && angle==77);
  bus.fail_id=-2;
  OpenHRP::ServoControllerService::dSequence all;
  all.data.assign(8,0.2);
  c.joint_groups["all"]=c.servo_id;
  CHECK(c.setJointAngles(all,1));
  CHECK(bus.calls==c.servo_id);
  CHECK(c.setJointAnglesOfGroup("all",all,1));
  CHECK(bus.calls==c.servo_id);
  // Reject invalid lengths before reading any element or sending a command.
  OpenHRP::ServoControllerService::dSequence short_angles;
  short_angles.data.assign(1,0.2);
  bus.calls.clear();
  CHECK(!c.setJointAngles(short_angles,1) && bus.calls.empty());
  short_angles.data.clear();
  CHECK(!c.setJointAngles(short_angles,1) && bus.calls.empty());
  short_angles.data.assign(9,0.2);
  CHECK(!c.setJointAngles(short_angles,1) && bus.calls.empty());
  // A subset must not read past its ID/angle arrays.
  c.joint_groups["subset"].push_back(4);
  short_angles.data.assign(1,0.2);
  CHECK(c.setJointAnglesOfGroup("subset",short_angles,1));
  CHECK(bus.calls.size()==1 && bus.calls[0]==4);
  c.joint_groups["unknown"].push_back(127);
  bus.calls.clear();
  CHECK(!c.setJointAnglesOfGroup("unknown",short_angles,1) && bus.calls.empty());
  bus.group_fail=true;
  CHECK(!c.setJointAngles(all,1));
  CHECK(!c.setJointAnglesOfGroup("all",all,1));
  bus.group_fail=false;
  // Every configured ID is attempted on OFF, even after the first failure.
  for(int failed=-2;failed<=9;++failed) {
    bus.fail_id=failed; bus.calls.clear();
    bool success=c.servoOff();
    CHECK(bus.calls==c.servo_id);
    CHECK(success==(failed!=-1 && (failed<2 || failed>9)));
  }
  // ON retains the original stop-on-first-failure behavior.
  bus.fail_id=4; bus.calls.clear();
  CHECK(!c.servoOn());
  CHECK(bus.calls.size()==3 && bus.calls[2]==4);
  bus.fail_id=-2; bus.calls.clear();
  CHECK(c.servoOn() && bus.calls==c.servo_id);
  puts("PASS: actual method bodies; scalar/group failure returns, output guard, all-ID OFF, ON early stop");
}
"""
args.output.mkdir(parents=True, exist_ok=True)
cpp = args.output / "controller-results.cpp"
binary = args.output.resolve() / "controller-results"
cpp.write_text(prefix + "\n".join(declarations) + "\n};\n" + "\n".join(bodies) + main)
subprocess.run(["g++", "-std=gnu++98", "-fsanitize=address,undefined",
                "-fno-omit-frame-pointer", "-no-pie", str(cpp), "-o", str(binary)], check=True)
subprocess.run([str(binary)], check=True, timeout=5)
