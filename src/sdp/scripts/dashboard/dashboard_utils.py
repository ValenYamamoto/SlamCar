import numpy as np  
import pandas as pd
import streamlit as st
import plotly.graph_objects as go

def graph_particles(x, y):
    fig = go.Scatter(x=x, y=y, mode='markers', name='particles', marker=dict(size=7, color='red'))
    return fig

def graph_estimate(x, y):
    fig = go.Scatter(x=[x], y=[y], mode='markers', name='estimate', marker=dict(size=15, color='blue'))
    return fig

def graph_walls(wall_x, wall_y):
    walls_x = [0, wall_x, wall_x, 0, 0]
    walls_y = [0, 0, wall_y, wall_y, 0]
    fig = go.Scatter(x=walls_x, y=walls_y, mode='lines', name='walls')
    return fig

def create_figure(particles, estimate, walls):
    """
    particles: (x_array, y_array)
    estimate: ([x_pos], [y_pos])
    walls: (x_array, y_array)

    returns: figure

    """
    data = (graph_particles(*particles), graph_estimate(*estimate), graph_walls(*walls))
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
    return figure
