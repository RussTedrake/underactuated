import numpy as np
import plotly.express as px

x = np.linspace(-np.pi, np.pi, 101)
f = x**2 + 3*np.sin(x)**2
df = 2*x + 6*np.sin(x)*np.cos(x)

fig = px.line(
  x=x, y=f, 
  labels={'x':'x', 'y':'f(x)'},
  color_discrete_sequence=['#993333']
)
fig.update_layout(
  margin=dict(l=20, r=20, t=20, b=20),
  plot_bgcolor='rgba(0,0,0,0)',
  paper_bgcolor='rgba(0,0,0,0)'
)

fig.write_html("data/pl_inequality.html")
fig.show()

fig2 = px.line(
  x=f, y=df**2, 
  labels={'x':'f(x)-f*', 'y':'|df/dx|²'},
  color_discrete_sequence=['#993333']
)
fig2.update_layout(
  margin=dict(l=20, r=20, t=20, b=20),
  plot_bgcolor='rgba(0,0,0,0)',
  paper_bgcolor='rgba(0,0,0,0)'
)

fig2.write_html("data/pl_inequality_grad.html")
fig2.show()

