import time 

import numpy as np
import pandas as pd 
import plotly.express as px 
import streamlit as st 

from dashboard_socket import DashboardSocket, str_to_particles
from dashboard_utils import create_figure, read_yaml_file

HOST = '127.0.0.1'
PORT = 65432

# Setup dashboard
st.set_page_config(
    page_title="SLAM Dashboard",
    page_icon="✅",
    layout="wide",
)

st.title("SLAM Dashboard")

# Connect Socket
socket = DashboardSocket(False, HOST, PORT)


try:
    # creating a single-element container
    placeholder = st.empty()
    with placeholder.container():

        kpi1, kpi2 = st.columns(2)

        kpi1.metric(
            label="Monte Carlo Estimate",
            value=f"N/A",
        )
        
        kpi2.metric(
            label="Importance Weight Estimate",
            value=f"N/A",
        )
        
            
        st.markdown("### Detailed Data View")
        st.markdown("No Data Yet")

    socket.connect()


    while True:

        msg = socket.receive()
        if msg:
            d = read_yaml_file()
            x, y, w, landmarks = str_to_particles(msg, d['N_LANDMARKS'])
            """
            x_pos, y_pos, iw = [], [], []
            mc_x, mc_y, iw_x, iw_y, iw_total = 0, 0, 0, 0, 0
            for x, y, w in particles:
                x_pos.append(x)
                y_pos.append(y)
                iw.append(w)
                mc_x += x
                mc_y += y
                iw_x += x * w
                iw_y += y * w
                iw_total += w
            mc_x /= len(x_pos)
            mc_y /= len(x_pos)
            iw_x /= iw_total
            iw_y /= iw_total
            """
            mc_x = np.mean(x)
            mc_y = np.mean(y)
            iw_x = np.sum(x * w) / np.sum(w)
            iw_y = np.sum(y * w) / np.sum(w)
            landmark_ests = np.mean(landmarks, axis=0)


            df = pd.DataFrame({'x': x, "y": y, "w": w})

            with placeholder.container():

                # create three columns
                kpi1, kpi2 = st.columns(2)

                # fill in those three columns with respective metrics or KPIs
                kpi1.metric(
                    label="Monte Carlo Estimate",
                    value=f"({mc_x:.2f}, {mc_y:.2f})",
                )
                
                kpi2.metric(
                    label="Importance Weight Estimate",
                    value=f"({iw_x:.2f}, {iw_y:.2f})",
                )
                
                # fig = px.scatter(x=x_pos, y=y_pos)
                # fig.update_xaxes(range=[-2, 22])
                # fig.update_yaxes(range=[-2, 22])
                # st.write(fig
                fig = create_figure(d, (x, y), (mc_x, mc_y), landmark_ests)
                st.write(fig)
                    
                st.markdown("### Detailed Data View")
                st.dataframe(df)
        else:
            socket.connect()
finally:
    socket.close()
