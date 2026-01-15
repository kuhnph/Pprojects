# viewer/renderer.py
import numpy as np
import moderngl

def euler_to_R(phi, theta, psi):
    c, s = np.cos, np.sin
    R = np.array([
        [c(theta)*c(psi), s(phi)*s(theta)*c(psi)-c(phi)*s(psi), c(phi)*s(theta)*c(psi)+s(phi)*s(psi)],
        [c(theta)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi)],
        [-s(theta),       s(phi)*c(theta),                      c(phi)*c(theta)]
    ], dtype=np.float32)
    return R

def model_matrix_from_state(pn, pe, pd, phi, theta, psi):
    # NED: x=north, y=east, z=down
    R = euler_to_R(phi, theta, psi)
    t = np.array([pn, pe, pd], dtype=np.float32)

    M = np.eye(4, dtype=np.float32)
    M[:3, :3] = R
    M[:3, 3] = t
    return M

def perspective(fovy_deg, aspect, znear, zfar):
    f = 1.0 / np.tan(np.deg2rad(fovy_deg) / 2.0)
    M = np.zeros((4, 4), dtype=np.float32)
    M[0, 0] = f / aspect
    M[1, 1] = f
    M[2, 2] = (zfar + znear) / (znear - zfar)
    M[2, 3] = (2 * zfar * znear) / (znear - zfar)
    M[3, 2] = -1.0
    return M

def look_at(eye, target, up):
    eye = np.array(eye, dtype=np.float32)
    target = np.array(target, dtype=np.float32)
    up = np.array(up, dtype=np.float32)

    f = target - eye
    f /= (np.linalg.norm(f) + 1e-9)
    r = np.cross(f, up)
    r /= (np.linalg.norm(r) + 1e-9)
    u = np.cross(r, f)

    M = np.eye(4, dtype=np.float32)
    M[0, :3] = r
    M[1, :3] = u
    M[2, :3] = -f
    M[:3, 3] = -M[:3, :3] @ eye
    return M

class Renderer:
    def __init__(self, ctx: moderngl.Context, vertices, indices):
        """
        GPU-side state rendering

        Uploads static aircraft geometry to the GPU
        shaders
        camera & projection parameters
        Draw the aircraft each frame using a model-view-projection matrix

        Parameters
        ----------
        ctx : moderngl.Context
            Active OpenGL context created by pyglet
        vertices : (N,3) float32 array
            Aircraft vertices in *model/body coordinates*
        indices : (M,) uint32 array
            Triangle indices defining mesh connectivity
        """

        # Store OpenGL context
        self.ctx = ctx
        # Makes triangles obstruct the view of other triangles
        self.ctx.enable(moderngl.DEPTH_TEST)

        # ------------------------------------------------------------
        # Shader program
        # ------------------------------------------------------------
        # Vertex shader:
        #   - Takes a vertex position in model space
        #   - Applies Model-View-Projection transform
    
        # Fragment shader:
        #   - Outputs a constant color (flat shading)
        #
        # Note: this is intentionally minimal for performance and clarity
        self.prog = self.ctx.program(
            vertex_shader="""
                #version 330
                uniform mat4 u_mvp;   // Combined model-view-projection matrix
                in vec3 in_pos;       // Vertex position (model space)

                void main() {
                    gl_Position = u_mvp * vec4(in_pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330
                out vec4 f_color;

                void main() {
                    // Light blue aircraft color
                    f_color = vec4(0.6, 0.75, 0.95, 1.0);
                }
            """,
        )

        # ------------------------------------------------------------
        # GPU buffers
        # ------------------------------------------------------------

        # Vertex Buffer Object (VBO): Stores static aircraft vertices
        vbo = self.ctx.buffer(vertices.tobytes())
        # Index Buffer Object (IBO): Stores triangle connections
        ibo = self.ctx.buffer(indices.tobytes())

        # Vertex Array Object (VAO):
        #   Binds vertex layout ("3f") to shader input "in_pos"
        #   and associates the index buffer
        self.vao = self.ctx.vertex_array(self.prog, [(vbo, "3f", "in_pos")],index_buffer=ibo)
        # Reference to the uniform used to upload MVP matrix each frame
        self.u_mvp = self.prog["u_mvp"]

        # ------------------------------------------------------------
        # Camera parameters (world coordinates, NED convention)
        # ------------------------------------------------------------

# ---------- Camera ----------
        self.up = np.array([0, 0, -1], dtype=np.float32)
        self.fovy = 60.0
        self.znear = 0.1
        self.zfar = 5000.0
        self.camera_mode = "chase"
        # Offsets / fixed camera points
        self.chase_offset = np.array([-250, -250, -120], dtype=np.float32)
        self.fixed_eye = np.array([0, -600, 0], dtype=np.float32)
        self.fixed_target = np.array([0, 0, 0], dtype=np.float32)

        # ---------- Trail ----------
        self.maxTrail = 1200
        self.trail = np.zeros((self.maxTrail, 3), dtype=np.float32)
        self.trailCount = 0
        self.trail_vbo = self.ctx.buffer(reserve=self.trail.nbytes) #virtual buffer object for the trail
        self.trailProgress = self.ctx.program(
            vertex_shader="""
            #version 330
            uniform mat4 u_mvp;
            in vec3 in_pos;
            void main(){
                gl_Position = u_mvp * vec4(in_pos, 1.0);
                }
        """,
        fragment_shader="""
            #version 330
            out vec4 f_color;
            void main() {
            f_color = vec4(1.0, 1.0, 0.3, 1.0); //yellow
                }
        """,
        )
        self.trail_u_mvp = self.trailProgress["u_mvp"]
        self.trail_vao = self.ctx.vertex_array(self.trailProgress, [(self.trail_vbo, "3f", "in_pos")])
        self.LINE_STRIP = getattr(moderngl, "LINE_STRIP", 3)

    def _appendTrail(self, pos):
        self.trail[self.trailCount % self.maxTrail, :] = pos
        self.trailCount += 1

        n = min(self.trailCount, self.maxTrail)

        if self.trailCount < self.maxTrail:
            data = self.trail[:n]
        else:
            start = self.trailCount % self.maxTrail
            data = np.vstack((self.trail[start:], self.trail[:start]))
        
        self.trail_vbo.write(data.tobytes())
        return n

    def _camera_matrices(self, pos, aspect):
        #----------------Build view and projection matrices based on current camera mode------------------
        if self.camera_mode == 'chase':
            eye = pos + self.chase_offset
            target = pos
        elif self.camera_mode == "fixed":
            eye = self.fixed_eye
            target = self.fixed_target
        else:
            eye = self.fixed_eye
            target = pos
        
        V = look_at(eye, target, self.up)
        P = perspective(self.fovy, aspect, self.znear, self.zfar)
        return P, V

    def draw(self, width, height, model_mat):
        """
        Draw the aircraft for the current frame.

        Parameters
        ----------
        width : int
            Window width (pixels)
        height : int
            Window height (pixels)
        model_mat : (4,4) float32 array
            Model transform built from MAV state:
            - rotation from (phi, theta, psi)
            - translation from (pn, pe, pd)
        """

        # Clear color + depth buffers
        # This wipes the previous frame
        self.ctx.clear(0.05, 0.06, 0.08)

        # Aspect ratio needed for camera projection
        aspect = width / max(height, 1)

        # ------------------------------------------------------------
        # Camera setup (chase camera)
        # ------------------------------------------------------------

        # Aircraft world posistion based on (last column of homogeneous transform)
        pos = model_mat[:3, 3]

        #update trail and number of points to render
        n = self._appendTrail(pos)

        #update camera matrices
        P, V = self._camera_matrices(pos, aspect)

        # Combined transform:
        #   model → world → camera → clip
        mvp = P @ V @ model_mat

        # Draw trail in world space (model = Identity because trail points already world coords)
        mvp_world = P @ V @ np.eye(4, dtype=np.float32)
        self.trail_u_mvp.write(mvp_world.T.tobytes())
        self.trail_vao.render(mode=moderngl.LINE_STRIP, vertices=n)

        # Draw aircraft mesh (model = aircraft pose)
        self.u_mvp.write(mvp.T.tobytes())
        self.vao.render()
