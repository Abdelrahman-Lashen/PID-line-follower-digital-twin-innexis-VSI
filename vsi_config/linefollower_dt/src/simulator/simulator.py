#!/usr/bin/env python3
from __future__ import print_function
import struct, sys, argparse, math, random

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

class StraightLinePath:
    def __init__(self):
        self.length = 100.0
    def nearest_point(self, x, y):
        s = max(0.0, min(x, self.length))
        return s, 0.0, 1.0, 0.0, s

class CurvedPath:
    N_SAMPLES = 400
    def __init__(self):
        self._pts = self._sample()
    @staticmethod
    def _cubic_bezier(p0, p1, p2, p3, t):
        mt = 1.0 - t
        x = mt**3*p0[0] + 3*mt**2*t*p1[0] + 3*mt*t**2*p2[0] + t**3*p3[0]
        y = mt**3*p0[1] + 3*mt**2*t*p1[1] + 3*mt*t**2*p2[1] + t**3*p3[1]
        return x, y
    @staticmethod
    def _cubic_bezier_tangent(p0, p1, p2, p3, t):
        mt = 1.0 - t
        dx = 3*(mt**2*(p1[0]-p0[0]) + 2*mt*t*(p2[0]-p1[0]) + t**2*(p3[0]-p2[0]))
        dy = 3*(mt**2*(p1[1]-p0[1]) + 2*mt*t*(p2[1]-p1[1]) + t**2*(p3[1]-p2[1]))
        norm = math.sqrt(dx*dx + dy*dy) + 1e-9
        return dx/norm, dy/norm
    def _sample(self):
        segs = [((0,0),(5,0),(10,4),(15,4)),((15,4),(20,4),(25,0),(30,0))]
        pts = []
        n = self.N_SAMPLES // 2
        for seg in segs:
            for i in range(n):
                t = i / n
                pts.append((self._cubic_bezier(*seg, t), self._cubic_bezier_tangent(*seg, t)))
        return pts
    def nearest_point(self, rx, ry):
        best_dist = float('inf'); best_i = 0
        for i, (p, _) in enumerate(self._pts):
            d = (p[0]-rx)**2 + (p[1]-ry)**2
            if d < best_dist:
                best_dist = d; best_i = i
        p, tang = self._pts[best_i]
        return p[0], p[1], tang[0], tang[1], best_i / len(self._pts)

class DifferentialRobot:
    WHEEL_BASE = 0.3
    V_BASE = 1.0
    def __init__(self, x0=0.0, y0=0.0, theta0=0.0):
        self.x = x0; self.y = y0; self.theta = theta0
    def step(self, omega_cmd, dt):
        L = self.WHEEL_BASE
        vR = self.V_BASE + omega_cmd * L / 2.0
        vL = self.V_BASE - omega_cmd * L / 2.0
        v = (vR + vL) / 2.0
        w = (vR - vL) / L
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = (self.theta + w*dt + math.pi) % (2*math.pi) - math.pi

class SensorNoise:
    def __init__(self, sigma_pos=0.0, p_disturbance=0.0):
        self.sigma_pos = sigma_pos
        self.p_disturbance = p_disturbance
    def corrupt(self, x, y, theta):
        nx = x + random.gauss(0, self.sigma_pos) if self.sigma_pos > 0 else x
        ny = y + random.gauss(0, self.sigma_pos) if self.sigma_pos > 0 else y
        if random.random() < self.p_disturbance:
            nx += random.uniform(-0.5, 0.5)
            ny += random.uniform(-0.5, 0.5)
        return nx, ny, theta

class Simulator:
    def __init__(self, args):
        self.componentId = 0
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50101
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.receivedNumberOfBytes = 0
        self.receivedPayload = []
        self.path = StraightLinePath()
        self.robot = DifferentialRobot(0.0, random.uniform(-2.0, 2.0), random.uniform(-0.4, 0.4))
        self.noise = SensorNoise(0.0, 0.0)
        self.omega_cmd = 0.0
        self.dt = 0.01

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        try:
            vsiCommonPythonApi.waitForReset()
            self.updateInternalVariables()
            if vsiCommonPythonApi.isStopRequested():
                raise Exception("stopRequested")
            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
            while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                self.robot.step(self.omega_cmd, self.dt)
                rx, ry, rtheta = self.noise.corrupt(self.robot.x, self.robot.y, self.robot.theta)
                px, py, tx, ty, _ = self.path.nearest_point(rx, ry)
                for can_id, value in [(10,rx),(11,ry),(12,rtheta),(13,tx),(14,ty),(15,px),(16,py)]:
                    vsiCanPythonGateway.setCanId(can_id)
                    vsiCanPythonGateway.setCanPayloadBits(struct.pack("=d", value), 0, 64)
                    vsiCanPythonGateway.setDataLengthInBits(64)
                    vsiCanPythonGateway.sendCanPacket()
                try:
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, 20)
                    self.omega_cmd = struct.unpack("=d", d[0:8])[0]
                except:
                    pass
                self.updateInternalVariables()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception("stopRequested")
                nextExpectedTime += self.simulationStep
                if vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime:
                    continue
                if nextExpectedTime > self.totalSimulationTime:
                    vsiCommonPythonApi.advanceSimulation(self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs())
                    break
                vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())
        except Exception as e:
            if str(e) == "stopRequested":
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
            else:
                print(f"Simulator error: {str(e)}")
        except:
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.stopRequested = vsiCommonPythonApi.isStopRequested()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()

def main():
    inputArgs = argparse.ArgumentParser(" ")
    inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX')
    inputArgs.add_argument('--server-url', metavar='CO', default='localhost')
    args = inputArgs.parse_args()
    simulator = Simulator(args)
    simulator.mainThread()

if __name__ == '__main__':
    main()