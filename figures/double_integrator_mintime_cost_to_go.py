import plotly.graph_objects as go
import numpy as np

q = np.linspace(-10, 10, 51)
qdot = np.linspace(-4, 4, 31)
J = np.zeros((len(q), len(qdot)))

for i in range(len(q)):
    for j in range(len(qdot)):
        if (qdot[j] < 0 and
                q[i] < .5 * qdot[j]**2) or (qdot[j] >= 0 and
                                            q[i] < -.5 * qdot[j]**2):
            J[i, j] = 2 * np.sqrt(.5 * qdot[j]**2 - q[i]) - qdot[j]
        else:
            J[i, j] = qdot[j] + 2 * np.sqrt(.5 * qdot[j]**2 + q[i])

fig = go.Figure(
    go.Surface(
        x=q,
        y=qdot,
        z=J.T,  # I'm surprised by the need for transpose here.
        colorscale="jet",
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
    "zaxis_title": 'J'
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
fig.write_html("data/double_integrator_mintime_cost_to_go.html")
fig.show()
