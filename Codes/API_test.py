import datetime
from allthingstalk import Client, Device, StringAsset, AssetState
import csv
import time
# Parameters used to authorize and identify your device
# Get them on maker.allthingstalk.com
DEVICE_TOKEN = 'maker:4UnamcAIy29im0lqFxaBU3ZSeyPz2NV8P261raB0'
DEVICE_ID = 'AqfnjTLsXCMuf3u0VCLak63f'

##class WeatherStation(Device):
##    temperature = talk.NumberAsset(unit='°C')
##    humidity = talk.NumberAsset(unit='%')


client = Client(DEVICE_TOKEN)

#weather = WeatherStation(client=client, id='AqfnjTLsXCMuf3u0VCLak63f')

#timestamp = datetime.datetime.utcnow() + datetime.timedelta(days=1)

#Node1temperature = client.get_asset_state(DEVICE_ID, 'Temperature')
#Node1humidity = client.get_asset_state(DEVICE_ID, 'Humidity')

##print(Node1humidity.value)
##print(Node1temperature.value)
##print(Node1humidity.at)

fields=[]

while True:
    timestamp = datetime.datetime.utcnow() + datetime.timedelta(days=1)
    Node1temperature = client.get_asset_state(DEVICE_ID, 'Temperature')
    Node1humidity = client.get_asset_state(DEVICE_ID, 'Humidity')
    Soil_Moisture_Percentage = client.get_asset_state(DEVICE_ID, 'Soil_Moisture_Percentage')
    print(Node1humidity.value)
    print(Node1temperature.value)
    print(Soil_Moisture_Percentage)
    print(Node1humidity.at)
    with open('LocalTest.csv', 'a') as f:
        fields = [timestamp, Node1temperature, Node1humidity, Soil_Moisture_Percentage]
        writer = csv.writer(f)
        writer.writerow(fields)
        print('done writing')
    time.sleep(1800)
#############################################################


