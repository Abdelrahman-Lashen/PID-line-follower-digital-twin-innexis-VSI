#!/usr/bin/env python3
from __future__ import print_function
import struct, sys, argparse, math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

class PIDController:
    MAX_OMEGA = 5.0
    def __init__(self, Kp=2.0, Ki=0.1, Kd=0.5, alpha=0.5, dt=0.01):
        self.Kp=Kp; self.Ki=Ki; self.Kd=Kd; self.alpha=alpha; self.dt=dt
        self._integral=0.0; self._prev_err=0.0
    def _wrap(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    def compute(self, rx, ry, rtheta, px, py, tx, ty):
        e_lat = (rx-px)*ty - (ry-py)*tx
        e_heading = self._wrap(math.atan2(ty, tx) - rtheta)
        e = e_lat + self.alpha * e_heading
        self._integral += e * self.dt
        derivative = (e - self._prev_err) / self.dt
        self._prev_err = e
        u = self.Kp*e + self.Ki*self._integral + self.Kd*derivative
        return max(-self.MAX_OMEGA, min(self.MAX_OMEGA, u))

class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50102
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.receivedNumberOfBytes = 0
        self.receivedPayload = []
        self.pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.5, alpha=0.5, dt=0.01)
        self.rx=0.0; self.ry=0.0; self.rtheta=0.0
        self.px=0.0; self.py=0.0; self.tx=1.0; self.ty=0.0
        self.omega_cmd = 0.0

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
                try:
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,10)
                    self.rx = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,11)
                    self.ry = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,12)
                    self.rtheta = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,13)
                    self.tx = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,14)
                    self.ty = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,15)
                    self.px = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,16)
                    self.py = struct.unpack("=d", d[0:8])[0]
                except:
                    pass
                self.omega_cmd = self.pid.compute(self.rx, self.ry, self.rtheta, self.px, self.py, self.tx, self.ty)
                vsiCanPythonGateway.setCanId(20)
                vsiCanPythonGateway.setCanPayloadBits(struct.pack("=d", self.omega_cmd), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()
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
                print(f"Controller error: {str(e)}")
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
    controller = Controller(args)
    controller.mainThread()

if __name__ == '__main__':
    main()