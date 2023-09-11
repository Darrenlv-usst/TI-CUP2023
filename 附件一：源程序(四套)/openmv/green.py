import time, sensor, lcd
from pyb import UART
THRESHOLD_R = [(0, 255, 15, 255, -8, 61)]
THRESHOLD_R = [(0, 255, 15, 255, -8, 61)]
CENTER_X = 160
CENTER_Y = 167
def sending_data(dx, dy):
	global uart
	flx = 0
	fly = 0
	if dx < 0:
		dx = -dx
		flx = 1
	if dy < 0:
		dy = -dy
		fly = 1
	FH = bytearray([0xA1, 0x02, flx, dx, fly, dy, 0x1A])
	uart.write(FH)
uart = UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_contrast(3)
sensor.set_auto_exposure(False, 1200)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.set_hmirror(False)
sensor.set_vflip(True)
err_dx = 0
err_dy = 0
once = 1
blob = []
task = 1
while True:
	img = sensor.snapshot()
	img.lens_corr(1.4)
	img.draw_cross(CENTER_X, CENTER_Y)
	if uart.any():
		task = uart.read(1)
		if task == b'1':
			once = 1
	if task == b'1':
		if once == 1:
			blobs = img.find_blobs(THRESHOLD_R, pixels_threshold = 2, area_threshold = 15)
		else:
			blobs = img.find_blobs(THRESHOLD_R, roi = (CENTER_X - 35, CENTER_Y - 35, 70, 70), pixels_threshold = 2, area_threshold = 3)
		blob = []
		dst = 150
		for b in blobs:
			x = b[0] + int(b[2] / 2)
			y = b[1] + int(b[3] / 2)
			dx = abs(x - CENTER_X)
			dy = abs(y - CENTER_Y)
			if dx + dy < dst:
				blob = b
				dst = dx + dy
		if blob:
			cx = blob[0] + int(blob[2] / 2)
			cy = blob[1] + int(blob[3] / 2)
			if once == 1:
				err_dx = cx - CENTER_X
				err_dy = cy - CENTER_Y
				sending_data(-err_dx, -err_dy)
				img.draw_line(CENTER_X, CENTER_Y, cx, cy)
				if abs(err_dx) <= 5 and abs(err_dy) <= 5:
					past = blob
					once = 0
			else:
				cx_past = past[0] + int(past[2] / 2)
				cy_past = past[1] + int(past[3] / 2)
				if abs(cx - cx_past) < 35 and abs(cy - cy_past) < 35:
					err_dx = cx - CENTER_X
					err_dy = cy - CENTER_Y
					past = blob
					sending_data(-err_dx, -err_dy)
					img.draw_line(CENTER_X, CENTER_Y, cx, cy)
	elif task == b'2':
		blobs = img.find_blobs(THRESHOLD_G, pixels_threshold = 2, area_threshold = 15)
		blob.clear()
		dst = 150
		for b in blobs:
			x = b[0] + int(b[2] / 2)
			y = b[1] + int(b[3] / 2)
			dx = abs(x - CENTER_X)
			dy = abs(y - CENTER_Y)
			if dx + dy < dst:
				blob = b
				dst = dx + dy
		if blob:
			CENTER_X = blob[0] + int(blob[2] / 2)
			CENTER_Y = blob[1] + int(blob[3] / 2)
			task = b'0'