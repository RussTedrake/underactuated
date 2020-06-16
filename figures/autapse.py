# -*- coding: utf-8 -*-
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import numpy as np

app = dash.Dash(__name__, external_stylesheets=["../htmlbook/book.css"])

app.layout = html.Div([
    dcc.Graph(id='graph'),
    html.Div(['Set ', html.I(['w']), ': ']),
    dcc.Slider(
        id='w',
        min=0,
        max=3.,
        value=2.,
        marks={w: str(w) for w in np.arange(0, 3.1, 0.5)},
        step=0.1,
        updatemode='drag',
        included=False,
        # tooltip = { 'always_visible': True }
    ),
    html.Div(['Set ', html.I(['u']), ': ']),
    dcc.Slider(
        id='u',
        min=-1.5,
        max=1.5,
        value=0,
        marks={u: str(u) for u in np.arange(-1.5, 1.6, 0.5)},
        step=0.1,
        updatemode='drag',
        included=False,
        # tooltip = { 'always_visible': True, 'placement': 'bottom' }
    ),
])


def autapse(x, w=1, u=0):
    return -x + np.tanh(w * x + u)


Autapse = np.vectorize(autapse)
x = np.arange(-2., 2., 0.01)


@app.callback(Output('graph', 'figure'),
              [Input('w', 'value'), Input('u', 'value')])
def update_figure(w, u):
    return {
        'data': [
            {
                'x': x,
                'y': Autapse(x, w=w, u=u),
                'type': 'line'
            },
            {
                'x': [-1, 1],
                'y': [1, -1],
                'mode': 'lines',
                'line': {
                    'color': 'black',
                    'width': 1
                }
            },
        ],
        'layout': {
            'title': 'Autapse',
            'xaxis': {
                'title': 'x'
            },
            'yaxis': {
                'title': 'xdot'
            },
        },
    }


if __name__ == '__main__':
    app.run_server(debug=True)
