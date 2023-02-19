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
    walls_x = d['WALLS_X']
    walls_y = d['WALLS_Y']

    fig = go.Scatter(x=walls_x, y=walls_y, mode='lines', name='walls')
    return fig

def graph_landmarks(d):
    results = []
    for i in range(len(d['LANDMARK_X'])):
        walls_x, walls_y = generate_landmark_walls(d, d['LANDMARK_X'][i], d['LANDMARK_Y'][i])
        fig = go.Scatter(x=walls_x, y=walls_y, mode='lines', name='landmark')
        results.append(fig)
    return results 

def generate_landmark_walls(d, x, y):
    x_bot = x - d['LANDMARK_R']
    y_bot = y - d['LANDMARK_R']
    r = d['LANDMARK_R'] * 2 # actually the diameter but im lazy
    return [x_bot, x_bot, x_bot+r, x_bot+r, x_bot], [y_bot, y_bot+r, y_bot+r, y_bot, y_bot]

def graph_landmark_estimate(landmarks):
    fig = go.Scatter(x=landmarks[:, 0], y=landmarks[:, 1], mode='markers', name='landmark est', marker=dict(size=15, color='green'))
    return fig

def create_figure(d, particles, estimate, landmarks):
    """
    particles: (x_array, y_array)
    estimate: ([x_pos], [y_pos])
    walls: (x_array, y_array)

    returns: figure

    """
    if len(d["WALLS_X"]) == 2:
        x_min, x_max = -1, np.max(d["WALLS_X"]) + 1
        y_min, y_max = np.min(d["WALLS_Y"]) - 1, np.max(d["WALLS_Y"]) + 1
    else:
        x_min, x_max = np.min(d["WALLS_X"]), np.max(d["WALLS_X"]) + 1
        y_min, y_max = np.min(d["WALLS_Y"]) - 1, np.max(d["WALLS_Y"]) + 1
    data = [graph_particles(*particles), graph_estimate(*estimate), graph_walls(d)]
    if d["N_LANDMARKS"] > 0:
        landmark_walls = graph_landmarks(d)
        data += landmark_walls
        data.append(graph_landmark_estimate(landmarks))
        print(landmarks)

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
