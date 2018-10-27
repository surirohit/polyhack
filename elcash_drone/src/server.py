#!/usr/bin/env python
import requests

SERVER = "http://10.4.14.37:5000/api"

def getArena():
    url = SERVER + "/arena"
    params = {}
    r = requests.get(url=url, params=params)
    data = r.json()
    print data

if __name__ == '__main__':
    getArena()
