# app.py 2/15/2026
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)
import numpy as np
from sim.dynamics import dynamics
from sim.params import params
from viewer.mesh import aircraft_model_mesh
from viewer.renderer import renderer
from viewer.window import SimWindow
from plotting.dataLogging import Logger

def main():
    dyn = dynamics()
    P = params()
    L = Logger()
    log = Logger(N=int(P.N))

    # Static mesh once
    V, idx = aircraft_model_mesh(scale=5.0)

    def sim_step(dt):
        u = dyn_u()
        log.log(P.T,dyn.state,u)
        dyn.update(u)
        P.T += dt

    def dyn_u():
        # simplest: hold trim input from params (replace with controller later)
        return win.u

    def get_pose():
        s = dyn.state
        return (
            float(s[0,0]), float(s[1,0]), float(s[2,0]),
            float(s[6,0]), float(s[7,0]), float(s[8,0]),
        )

    def renderer_factory(ctx):
        return renderer(ctx, V, idx)

    def get_sim_time():
        return float(P.T)

    win = SimWindow(
        sim_step_func=sim_step,
        get_pose_func=get_pose,
        get_sim_time_func=get_sim_time,
        renderer_factory=renderer_factory,
        width=1000,
        height=800,
        render_hz=60,
        sim_hz=200,
        record=False,
    )

    import pyglet
    try:
        pyglet.app.run()
    finally:
        # force finalize ffmpeg even if the app exits without closing the window
        print('Exporting')
        log.export()
        try:
            if getattr(win, "writer", None) is not None:
                win.writer.close()
                win.writer = None
        except Exception as e:
            print("Error closing writer:", e)

if __name__ == "__main__":
    main()
