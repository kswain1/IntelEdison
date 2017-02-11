import requests

payload = {"accelx":[0,0], "accely":[0,0],
                       "accelz":[0,0]}

r=requests.post('https://obscure-headland-45385.herokuapp.com/swings',json=payload)
