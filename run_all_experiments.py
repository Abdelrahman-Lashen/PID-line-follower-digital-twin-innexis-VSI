import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiments.E1_gain_sweep import run_e1
from experiments.E2_E3_E4 import run_e2, run_e3, run_e4

def main():
    print("=== Robot Line Follower — All Experiments ===")
    t0 = time.time()
    run_e1(); run_e2(); run_e3(); run_e4()
    print(f"\nAll done in {time.time()-t0:.1f}s")
    print("Results in: outputs/E1/ E2/ E3/ E4/")

if __name__ == "__main__":
    main()
