"""Convert the CSV files written by matlab/reference_run.m into results/matlab/*.npz.

    python3 model/matlab_csv_to_npz.py <directory with the CSV files>

reference_states.csv and reference_outputs.csv become reference_states.npz (keys states, outputs).
reference_sim_<tag>.csv becomes reference_sim_<tag>.npz with t = column 0, x = columns 1:25 and
out = columns 25:36.
"""
import os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.abspath(os.path.join(HERE, '..', 'results', 'matlab'))


def main(src):
    os.makedirs(OUT, exist_ok=True)
    Z = np.loadtxt(os.path.join(src, 'reference_states.csv'), delimiter=','); O = np.loadtxt(os.path.join(src, 'reference_outputs.csv'), delimiter=',')
    np.savez_compressed(os.path.join(OUT, 'reference_states.npz'), states=Z, outputs=O)
    for tag in ('default', 'tight'):
        f = os.path.join(src, f'reference_sim_{tag}.csv')
        if os.path.exists(f):
            M = np.loadtxt(f, delimiter=',')
            np.savez_compressed(os.path.join(OUT, f'reference_sim_{tag}.npz'), t=M[:, 0], x=M[:, 1:25], out=M[:, 25:])
    print('wrote', OUT)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, '..', '..', 'matlab')))
