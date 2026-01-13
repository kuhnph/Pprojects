import numpy as np
import pandas as pd
import os
from sim.params import params
P = params()

class Logger:
    def __init__(self, N=int(P.N), n_state=12, n_u=4):
        self.k = 0
        self.t = np.empty((N,), dtype=np.float32)
        self.x = np.empty((N, n_state), dtype=np.float32)
        self.u = np.empty((N,n_u), dtype=np.float32)

    def log(self, t, x, u):
        self.t[self.k] = t
        self.x[self.k, :] = x.reshape(-1) #makes the array go from vertical to horizontal 1D
        self.u[self.k, :] = u.reshape(-1)
        self.k += 1
    
    def export(self):
        """
        t: (N,) time 
        x: (N,12) state history
        u: (N,4) control history
        """
        t = self.t
        x = self.x
        state_cols = ["pn","pe","pd","u","v","w","phi","theta","psi","p","q","r"]
        u_cols = ["del_e","del_t","del_a","del_r"]
        df = pd.DataFrame(x, columns=state_cols)
        df.insert(0, 't', t)
        df.to_parquet(os.path.join('out',"output.parquet"), index=False)
        np.savez_compressed(os.path.join('out',"output"), t=t, x=x)


    def trim(self):
        return self.t[:self.k], self.x[:self.k], self.u[:self.k]