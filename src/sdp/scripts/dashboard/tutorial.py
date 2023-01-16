import operator as op
import numpy as np
import pandas as pd
import requests
import param
import panel as pn
import hvplot.pandas
import hvplot.streamz
import holoviews as hv
from holoviews.element.tiles import EsriImagery
from holoviews.selection import link_selections
from datashader.utils import lnglat_to_meters
from streamz.dataframe import PeriodicDataFrame

def weather_data(cities, api_key=openweathermap_api):
    L = []
    for c in cities:
        res = requests.get(f'https://api.openweathermap.org/data/2.5/weather?q={c}$appid={api_key}&units=imperial')
        L.append(res.json())

    df = pd.Dataframe(L)
    df['lon'] = df['coord'].map(op.itemgetter('lon'))
    df['lat'] = df['coord'].map(op.itemgetter('lat'))
    df['Temprature'] = df['main'].map(op.itemgetter('temp'))
    df['Humidity'] = df['main'].map(op.itemgetter('humidity'))
    df['Wind Speed'] = df['wind'].map(op.itemgetter('speed'))
    return df[['name','lon', 'lat','Temprature','Humidity','Wind Speed']]

def streaming_weather_data(**kwargs):
    df = weather_data(["San Francisco"])
    df['time'] = [pd.Timestamp.now()]
    return df.set_index('time')

df = PeriodDataFrame(streaming_weather_data, interval='30s')

pn_realtime = pn.Column(
        pn.Row(
            df[['Temperature']].hvplot.line(title="Temperature", backlog=1000),
            df[['Humidity']].hvplot.line(title="Temperature", backlog=1000)),
        df[['Wind Speed']].hvplot.line(title="Temperature", backlog=1000)
)
