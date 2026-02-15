# src/viewer/window.py - 2/15/2026
"""viewer.window

Pyglet window + ModernGL context that runs the simulation loop and renders frames.

- Advances sim at fixed sim_hz using an accumulator
- Renders at render_hz
- Optional offscreen capture via FBO + FFmpegVideoWriter

"""  
import pyglet
import moderngl
import time
import numpy as np
from sim.rotations import model_matrix_from_pose
from viewer.videoCapture import FFmpegVideoWriter


class SimWindow(pyglet.window.Window):
    def __init__(self, sim_step_func, get_pose_func, renderer_factory,
                 width=1000, height=800, render_hz=60, sim_hz=200,
                 record=False, record_path="results/mav_view.mp4"):
        super().__init__(width=width, height=height, caption="MAV Viewer (GPU)", resizable=True)

        self.ctx = moderngl.create_context()
        self.renderer = renderer_factory(self.ctx)

        # Fullscreen present quad (to display cap_tex on the window)
        self.present_prog = self.ctx.program(
            vertex_shader="""
                #version 330
                in vec2 in_pos;
                in vec2 in_uv;
                out vec2 v_uv;
                void main() {
                    v_uv = in_uv;
                    gl_Position = vec4(in_pos, 0.0, 1.0);
                }
            """,
            fragment_shader="""
                #version 330
                uniform sampler2D u_tex;
                in vec2 v_uv;
                out vec4 f_color;
                void main() {
                    f_color = texture(u_tex, v_uv);
                }
            """,
        )

        quad = np.array([
            #  x,   y,   u,  v
            -1.0, -1.0, 0.0, 0.0,
             1.0, -1.0, 1.0, 0.0,
            -1.0,  1.0, 0.0, 1.0,
             1.0,  1.0, 1.0, 1.0,
        ], dtype=np.float32)

        self.present_vbo = self.ctx.buffer(quad.tobytes())
        self.present_vao = self.ctx.vertex_array(
            self.present_prog,
            [(self.present_vbo, "2f 2f", "in_pos", "in_uv")]
        )


        self.sim_step = sim_step_func
        self.get_pose = get_pose_func

        self.sim_dt = 1.0 / sim_hz
        self.accum = 0.0
        self.last = time.perf_counter()

        # ----- Recording (Option C) -----
        self.record = record
        self.record_path = record_path
        self.record_fps = int(render_hz)
        self.writer = None


        # One callback that advances sim time (variable dt -> fixed sim steps)
        pyglet.clock.schedule(self._tick)

        # Separate callback that forces redraw at a steady rate
        pyglet.clock.schedule_interval(self._render, 1.0 / render_hz)

        # Offscreen capture targets (FBO + texture)
        self.cap_tex = None
        self.cap_depth = None
        self.cap_fbo = None
        self._cap_size = None


    def _ensure_writer(self):
        if not self.record:
            return

        fb_w, fb_h = self.ctx.screen.size

        if self.writer is None:
            self.writer = FFmpegVideoWriter(
                width=fb_w,
                height=fb_h,
                fps=self.record_fps,
                outfile=self.record_path,
                log_path="results/ffmpeg_capture.log",
            )
            self._last_size = (fb_w, fb_h)
            return

        if (fb_w, fb_h) != self._last_size:
            print("[record] Framebuffer resized; closing writer to avoid corrupt output.")
            self.writer.close()
            self.writer = None
            self.record = False

    def _tick(self, _dt):
        now = time.perf_counter()
        frame_dt = now - self.last
        self.last = now

        self.accum += frame_dt
        # prevent spiral of death if you pause/debug
        if self.accum > 0.25:
            self.accum = 0.25

        while self.accum >= self.sim_dt:
            self.sim_step(self.sim_dt)
            self.accum -= self.sim_dt

    def _render(self, _dt):
        # If recording, make sure capture FBO and writer exist and sizes match
        if self.record:
            self._ensure_capture_fbo()
            self._ensure_writer()

        # Draw (on_draw will render into FBO if recording)
        self.dispatch_event("on_draw")

        # If recording, read from the texture (NOT the window framebuffer)
        if self.record and self.writer is not None:
            frame = self.cap_tex.read(alignment=1)  # rgb bytes
            self.writer.write(frame)

        # Present to screen
        self.flip()


    def on_close(self):
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        super().on_close()


    def on_draw(self):
        pn, pe, pd, phi, theta, psi = self.get_pose()
        model = model_matrix_from_pose(pn, pe, pd, phi, theta, psi)

        if self.record and self.cap_fbo is not None:
            # Render into offscreen FBO only
            self.cap_fbo.use()
            self.cap_fbo.clear(0.0, 0.0, 0.0, 1.0)  # bright magenta
            self.renderer.draw(self._cap_size[0], self._cap_size[1], model)
            return

        # Normal live render direct to screen
        self.clear()
        self.renderer.draw(self.width, self.height, model)



    @staticmethod
    def _model_matrix(pn, pe, pd, phi, theta, psi):
        """Backward-compatible wrapper for model matrix."""
        return model_matrix_from_pose(pn, pe, pd, phi, theta, psi)

    def _ensure_capture_fbo(self):
        # Use actual framebuffer pixel size, not logical window size
        fb_w, fb_h = self.ctx.screen.size
        size = (fb_w, fb_h)

        if self._cap_size == size and self.cap_fbo is not None:
            return

        # (Re)create capture targets
        self._cap_size = size

        # RGB8 texture + depth buffer
        self.cap_tex = self.ctx.texture(size, components=3, dtype="u1")
        self.cap_depth = self.ctx.depth_renderbuffer(size)
        self.cap_fbo = self.ctx.framebuffer(color_attachments=[self.cap_tex], depth_attachment=self.cap_depth)

        # Optional: good defaults
        self.cap_fbo.clear(0.05, 0.06, 0.08, 1.0)

    def _present_to_screen(self):
        self.ctx.screen.use()
        fb_w, fb_h = self.ctx.screen.size
        self.ctx.viewport = (0, 0, int(fb_w), int(fb_h))

        # Clear the window framebuffer so we KNOW we touched it
        self.ctx.clear(0.0, 0.2, 0.0, 1.0)  # dark green background

        self.ctx.disable(moderngl.DEPTH_TEST)

        self.present_prog["u_tex"].value = 0
        self.cap_tex.use(location=0)

        # IMPORTANT: force 4 verts
        self.present_vao.render(mode=moderngl.TRIANGLE_STRIP, vertices=4)

        self.ctx.enable(moderngl.DEPTH_TEST)


