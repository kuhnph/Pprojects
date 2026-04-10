# app.py
import sys
import os

src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)

from sim.dynamics import dynamics
from sim.params import params
from viewer.mesh import aircraft_model_mesh
from viewer.renderer import renderer
from viewer.window import SimWindow
from plotting.dataLogging import Logger


def main():
    P = params()
    dyn = dynamics(P)
    log = Logger(N=int(P.N))

    # Static mesh once
    V, idx = aircraft_model_mesh(scale=5.0)

    def sim_step(dt):
        u = dyn_u()

        n_substeps = max(1, int(round(P.speed_scale)))
        for _ in range(n_substeps):
            log.log(dyn.t, dyn.state, u)
            dyn.update(u)
            
        P.t = dyn.t

    def dyn_u():
        # Expected order: [delta_e, delta_t, delta_a, delta_r]^T
        return win.u

    def get_pose():
        s = dyn.state
        return (
            float(s[0, 0]),  # pn
            float(s[1, 0]),  # pe
            float(s[2, 0]),  # pd
            float(s[6, 0]),  # phi
            float(s[7, 0]),  # theta
            float(s[8, 0]),  # psi
        )

    def renderer_factory(ctx):
        return renderer(ctx, V, idx)

    def get_sim_time():
        return float(dyn.t)

    win = SimWindow(
        sim_step_func=sim_step,
        get_pose_func=get_pose,
        get_sim_time_func=get_sim_time,
        renderer_factory=renderer_factory,
        width=1000,
        height=800,
        render_hz=P.render_hz,
        sim_hz=int(round(1.0 / P.Ts)),
        record=False,
    )

    import pyglet
    try:
        pyglet.app.run()
    finally:
        print("Exporting")
        log.export()
        try:
            if getattr(win, "writer", None) is not None:
                win.writer.close()
                win.writer = None
        except Exception as e:
            print("Error closing writer:", e)


if __name__ == "__main__":
    main()