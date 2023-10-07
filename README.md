# pysmarttrak
python script to interact with a Sierra Instruments Smart Trak 50 mass flow controller

	with SmartTrak50('/dev/ttyUSB0') as s:
		print(s.GetFlow())

Usage is pretty straight forward, see function names for available queries.
