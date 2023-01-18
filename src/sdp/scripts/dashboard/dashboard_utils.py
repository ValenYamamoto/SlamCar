import numpy as np  
import pandas as pd
import streamlit as st
import plotly.graph_objects as go
import yaml

def read_yaml_file():
    with open("../params_filename.txt") as f:
        filename = next(f).strip()
    with open(filename) as f:
        params_dict = yaml.safe_load(f)
        
    return params_dict

def graph_particles(x, y):
    fig = go.Scatter(x=x, y=y, mode='markers', name='particles', marker=dict(size=7, color='red'))
    return fig

def graph_estimate(x, y):
    fig = go.Scatter(x=[x], y=[y], mode='markers', name='estimate', marker=dict(size=15, color='blue'))
    return fig

def graph_walls(d):
    print(d["WALLS_X"])
    walls_x = d['WALLS_X']
    walls_y = d['WALLS_Y']

    fig = go.Scatter(x=walls_x, y=walls_y, mode='lines', name='walls')
    return fig

def create_figure(particles, estimate, walls):
    """
    particles: (x_array, y_array)
    estimate: ([x_pos], [y_pos])
    walls: (x_array, y_array)

    returns: figure

    """
    d = read_yaml_file()
    if len(d["WALLS_X"]) == 2:
        x_min, x_max = -1, np.max(d["WALLS_X"]) + 1
        y_min, y_max = np.min(d["WALLS_Y"]) - 1, np.max(d["WALLS_Y"]) + 1
    else:
        x_min, x_max = np.min(d["WALLS_X"]), np.max(d["WALLS_X"]) + 1
        y_min, y_max = np.min(d["WALLS_Y"]) - 1, np.max(d["WALLS_Y"]) + 1
    data = (graph_particles(*particles), graph_estimate(*estimate), graph_walls(d))
    figure = go.Figure(
            data=data,
            layout=dict(
                autosize=False,
                width=500,
                height=500,
                margin=dict(
                    l=50,
                    r=50,
                    b=50,
                    t=50,
                    pad=4
                )
            )
        )
    figure.update_xaxes(range=[x_min, x_max])
    figure.update_yaxes(range=[y_min, y_max])
    return figure
