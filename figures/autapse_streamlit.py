import streamlit as st
import pandas as pd
import numpy as np
import altair as alt

# pip install streamlit
# streamlit run autapse_streamlit.py


def autapse(x, w=1, u=0):
    """Args:
    w is feedback weight
    u is input
    """
    return -x + np.tanh(w * x + u)


Autapse = np.vectorize(autapse)
xmax = 2.
ymax = 1.
x = np.arange(-xmax, xmax, 0.01)

w = st.slider('w', min_value=0.0, max_value=3.0, value=2.0)
u = st.slider('u', min_value=-1.5, max_value=1.5, value=0.0)

df = pd.DataFrame({'x': x, 'xdot': Autapse(x, w, u)})

c = alt.Chart(df).mark_line().encode(x='x', y='xdot')

st.altair_chart(c, use_container_width=True)
