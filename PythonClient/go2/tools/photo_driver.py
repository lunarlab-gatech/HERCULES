#!/usr/bin/env python3
"""Run the Go2 and G1 bridges and execute commands appended to per-robot command files (one per line):
   vel VX VY VYAW | stop | sit | stand | forward M | side M | turn DEG | pivot DEG | wait S | quit
usage: photo_driver.py DIR   (reads DIR/cmd_go2.txt and DIR/cmd_g1.txt, writes DIR/driver.log)"""
import os, sys, time, threading, math
PC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PC); sys.path.insert(0, os.path.join(PC, "go2"))
from go2_commander import RobotCommander

D = sys.argv[1]
robots = sys.argv[2:] or ["go2", "g1"]
logf = open(os.path.join(D, "driver.log"), "a", buffering=1)
def log(*a):
    s = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a)
    print(s, flush=True); logf.write(s + "\n")

def run(name):
    bot = RobotCommander(robot=name, verbose=False)
    path = os.path.join(D, "cmd_%s.txt" % name)
    seen = 0
    log(name, "ready")
    while bot.bridge.running:
        lines = open(path).read().splitlines() if os.path.exists(path) else []
        for line in lines[seen:]:
            seen += 1
            p = line.split()
            if not p: continue
            try:
                if p[0] == "vel": bot.set_velocity(*[float(x) for x in p[1:4]])
                elif p[0] == "stop": bot.stop()
                elif p[0] == "sit": bot.sit()
                elif p[0] == "stand": bot.stand()
                elif p[0] == "forward": bot.forward(float(p[1]))
                elif p[0] == "side": bot.sidestep(float(p[1]))
                elif p[0] == "turn": bot.turn(float(p[1]))
                elif p[0] == "pivot": bot.turn(float(p[1]), rate=1.0, mode="pivot")
                elif p[0] == "heading":  # closed-loop turn to an absolute MuJoCo yaw (deg): fast leg with a stop margin, then a slow trim
                    tgt = math.radians(float(p[1]))
                    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
                    for rate, lag, tol in ((0.8, math.radians(20), math.radians(22)), (0.3, math.radians(5), math.radians(6)), (0.3, math.radians(5), math.radians(6))):
                        err = wrap(tgt - bot.sim.yaw())
                        if abs(err) < tol: continue
                        sgn = 1 if err > 0 else -1
                        bot.set_velocity(0.0, 0.0, sgn * rate)
                        bot._wait_until(lambda: sgn * wrap(tgt - bot.sim.yaw()) < lag, abs(err) / rate * 3 + 3)
                        bot.stop(); bot.wait(0.8)
                    log(name, "heading err %.1f deg" % math.degrees(wrap(tgt - bot.sim.yaw())))
                elif p[0] == "wait": bot.wait(float(p[1]))
                elif p[0] == "quit": bot.close(); log(name, "quit"); return
            except Exception as e:
                log(name, "ERROR", line, repr(e))
            x, y, z = bot.pose()[:3]
            log(name, "done:", line, "pose %.2f %.2f %.2f yaw %.1f" % (x, y, z, math.degrees(bot.sim.yaw())))
        time.sleep(0.2)
    log(name, "bridge stopped")

ths = [threading.Thread(target=run, args=(r,), daemon=True) for r in robots]
for t in ths: t.start()
for t in ths: t.join()
