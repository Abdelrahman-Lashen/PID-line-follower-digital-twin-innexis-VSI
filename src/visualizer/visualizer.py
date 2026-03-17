#!/usr/bin/env python3
from __future__ import print_function
import struct, sys, argparse, math, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50103
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.receivedNumberOfBytes = 0
        self.receivedPayload = []
        self.robot_xs = []; self.robot_ys = []
        self.path_xs  = []; self.path_ys  = []
        self.lat_errs = []; self.omegas   = []
        self.times    = []; self.t        = 0.0

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
                    rx = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,11)
                    ry = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,15)
                    px = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,16)
                    py = struct.unpack("=d", d[0:8])[0]
                    d = vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,20)
                    omega = struct.unpack("=d", d[0:8])[0]
                    lat_err = math.sqrt((rx-px)**2 + (ry-py)**2)
                    self.robot_xs.append(rx); self.robot_ys.append(ry)
                    self.path_xs.append(px);  self.path_ys.append(py)
                    self.lat_errs.append(lat_err)
                    self.omegas.append(omega)
                    self.times.append(self.t)
                    self.t += 0.01
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
                print(f"Visualizer error: {str(e)}")
        except:
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
        self.save_results()

    def save_results(self):
        if not self.robot_xs:
            print("No data recorded."); return
        os.makedirs("outputs", exist_ok=True)
        overshoot = max(self.lat_errs)
        n_ss = max(1, len(self.lat_errs) // 5)
        ss_error = sum(self.lat_errs[-n_ss:]) / n_ss
        settling_time = self.times[-1]
        for i in range(len(self.lat_errs)):
            if all(e < 0.05 for e in self.lat_errs[i:]):
                settling_time = self.times[i]; break
        print(f"\n=== KPIs ===")
        print(f"  Overshoot:          {overshoot:.4f} m")
        print(f"  Settling time:      {settling_time:.4f} s")
        print(f"  Steady-state error: {ss_error:.4f} m")
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(self.path_xs, self.path_ys, 'k--', linewidth=2, label='Reference Path')
        ax.plot(self.robot_xs, self.robot_ys, 'b-', linewidth=1.5, label='Robot Trajectory')
        ax.plot(self.robot_xs[0], self.robot_ys[0], 'go', markersize=8, label='Start')
        ax.plot(self.robot_xs[-1], self.robot_ys[-1], 'r^', markersize=8, label='End')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title('VSI Robot Trajectory')
        ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
        plt.tight_layout()
        plt.savefig("outputs/vsi_trajectory.png", dpi=120); plt.close()
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(self.times, self.lat_errs, 'r-', linewidth=1.5)
        ax.axhline(0.05, color='gray', linestyle='--', label='Threshold (0.05m)')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('|Lateral Error| (m)')
        ax.set_title('VSI Lateral Error over Time')
        ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("outputs/vsi_error.png", dpi=120); plt.close()
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(self.times, self.omegas, 'g-', linewidth=1.5)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('omega_cmd (rad/s)')
        ax.set_title('VSI Angular Velocity Command')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("outputs/vsi_omega.png", dpi=120); plt.close()
        print("Plots saved to outputs/")

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.stopRequested = vsiCommonPythonApi.isStopRequested()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()

def main():
    inputArgs = argparse.ArgumentParser(" ")
    inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX')
    inputArgs.add_argument('--server-url', metavar='CO', default='localhost')
    args = inputArgs.parse_args()
    visualizer = Visualizer(args)
    visualizer.mainThread()

if __name__ == '__main__':
    main()