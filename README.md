# pysmarttrak
python script to interact with a Sierra Instruments Smart Trak 50 mass flow controller

	with SmartTrak50.open('/dev/ttyUSB0') as s:
		print(s.GetFlow())

Usage is pretty straight forward, see function names for available queries.

https://www.sierrainstruments.com/products/50series.html


Also added is the PolyScience CA10 refrigerating recirculating chiller.

	with PolyScienceCA10.open('/dev/ttyUSB0') as s:
		print(s.GetSetPoint())

Usage is pretty straight forward, see function names for available queries.

https://www.digivac.com/product/polyscience-durachill-ca10-chiller-10c-to-70c/

