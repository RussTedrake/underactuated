import plotly.graph_objects as go
import numpy as np

q = np.linspace(-10, 10, 51)
qdot = np.linspace(-3, 3, 41)
V = np.zeros((len(q), len(qdot)))
Vdot = np.zeros((len(q), len(qdot)))

for i in range(len(q)):
    for j in range(len(qdot)):
        E = .5*qdot[j]**2 - np.cos(q[i])
        Etilde = E - 1
        V[i,j] = 0.5*Etilde**2
        Vdot[i,j] = -(qdot[j]*np.cos(q[i])*Etilde)**2

fig = go.Figure(
    go.Surface(
        x=q,
        y=qdot,
        z=V.T,  # I'm surprised by the need for transpose here.
        colorscale="jet",
        cmin=0.0,
        cmax=2.5,
        contours={
            "x": {
                "start": q[0],
                "end": q[-1],
                "size": 0.5,
                "show": True
            },
            "y": {
                "start": qdot[0],
                "end": qdot[-1],
                "size": 0.5,
                "show": True
            },
        },
        lighting={
            "ambient": 1.0,
            "diffuse": 1.0,
            "specular": 0.0,
        },
        showscale=False,
    ))

fig.update_layout(scene={
                            "xaxis_title": 'q',
                            "yaxis_title": 'q̇',
                            "zaxis_title": 'V',
                            "zaxis" : {"range": [0., 2.5]},
                        },
                  scene_aspectmode='manual',
                  scene_aspectratio=dict(x=1, y=1, z=.5),
                  scene_camera={"eye": {
                      "x": 1,
                      "y": -1,
                      "z": 1.5
                  }},
                  margin={
                      "l": 0,
                      "r": 0,
                      "t": 0,
                      "b": 0
                  })
fig.write_html("data/cartpole_swingup_V.html")
fig.show()

fig2 = go.Figure(
    go.Surface(
        x=q,
        y=qdot,
        z=Vdot.T,  # I'm surprised by the need for transpose here.
        colorscale="jet",
        cmin=-1.5,
        cmax=0.0,
        contours={
            "x": {
                "start": q[0],
                "end": q[-1],
                "size": 0.5,
                "show": True
            },
            "y": {
                "start": qdot[0],
                "end": qdot[-1],
                "size": 0.5,
                "show": True
            },
        },
        lighting={
            "ambient": 1.0,
            "diffuse": 1.0,
            "specular": 0.0,
        },
        showscale=False,
    ))

fig2.update_layout(scene={
                            "xaxis_title": 'q',
                            "yaxis_title": 'q̇',
                            "zaxis_title": 'V̇',
                            "zaxis" : {"range": [-1.5, 0.0]},
                        },
                  scene_aspectmode='manual',
                  scene_aspectratio=dict(x=1, y=1, z=.5),
                  scene_camera={"eye": {
                      "x": 1,
                      "y": -1,
                      "z": 1.5
                  }},
                  margin={
                      "l": 0,
                      "r": 0,
                      "t": 0,
                      "b": 0
                  })
fig2.write_html("data/cartpole_swingup_Vdot.html")
fig2.show()