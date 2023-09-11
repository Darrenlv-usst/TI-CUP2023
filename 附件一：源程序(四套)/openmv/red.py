import time, sensor, lcd
from pyb import UART
THRESHOLD_R = [(0, 100, 25, 127, -33, 30)]
THRESHOLD_G = [(57, 100, -45, 18, -45, 43)]
THRESHOLD = 125
DIV_TASK_2 = 10
DIV_TASK_3 = 6
ROI = (10, 10, 90, 90)
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
sensor.set_framesize(sensor.QQVGA)
sensor.set_windowing((120, 120))
sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 20)
sensor.set_hmirror(False)
sensor.set_vflip(True)
sensor.set_auto_exposure(False,1000)
lcd.init()
clock = time.clock()
cnt_find_rect = 0
p_array = []
flgstart = 0
flgend = 0
flgend3 = 0
cnt = 0
err_dx = 0
err_dy = 0
task = b'0'
goal_task2 = 0
goal_task3 = 0
goal_task4 = 0
cx_screen = 60
cy_screen = 60
p0 = (97, 19)
p1 = (19, 22)
p2 = (20, 98)
p3 = (98, 98)
d0 = (93, 21)
d1 = (51, 22)
d2 = (52, 51)
d3 = (93, 50)
point_temp = []
while True:
	img = sensor.snapshot()
	if uart.any():
		task = uart.read(1)
		FH = bytearray([0xA1,0x33,0X1A])
		uart.write(FH)
	blobs = img.find_blobs(THRESHOLD_R, pixels_threshold = 1, area_threshold = 1)
	blob = []
	for b in blobs:
		img.draw_rectangle(b.rect(), color = (0, 255, 0), scale = 1, thickness = 1)
	max_size=0
	for b in blobs:
		if b.pixels() > max_size:
			blob = b
			max_size = b.pixels()
	if blob:
		cx = blob[0] + int(blob[2] / 2)
		cy = blob[1] + int(blob[3] / 2)
	if task == b'1':
		img.draw_string(0, 0, "task:1")
		img.draw_cross(point_temp[0][0], point_temp[0][1])
		if blob:
			err_dx = cx - point_temp[0][0]
			err_dy = cy - point_temp[0][1]
			if abs(err_dx) < 1 and abs(err_dy) < 1:
				task = b'0'
				FH = bytearray([0xA1, 0x11, 0x1A])
				uart.write(FH)
				print('1')
		sending_data(err_dx, err_dy)
	elif task == b'2':
		img.draw_string(0, 0, "task:2")
		points = (point_temp[1], point_temp[2], point_temp[3], point_temp[4])
		if blob:
			err_dx = cx - points[goal_task2][0]
			err_dy = cy - points[goal_task2][1]
			if goal_task2 == 1:
				err_dy = 0
			elif goal_task2 == 2:
				err_dx = 0
			elif goal_task2 == 3:
				err_dy = 0
			elif goal_task2 == 0 and flgend == 1:
				err_dx = 0
			print(err_dx, err_dy)
			if abs(err_dx) < 2 and abs(err_dy) < 2:
				goal_task2 = goal_task2 + 1
				if goal_task2 == 4:
					flgend = 1
					goal_task2 = 0
				elif flgend == 1:
					task = b'0'
					goal_task2 = 0
					FH = bytearray([0xA1, 0x11, 0x1A])
					uart.write(FH)
					print("OK")
					flgend = 0
		sending_data(err_dx, err_dy)
	elif task == b'4' or task == b'3':
		if cnt_find_rect > 10 and flgstart == 0:
			flgstart = 1
			corner_cx = int((corner[3][0] + corner[2][0] + corner[1][0] + corner[0][0]) / 4)
			corner_cy = int((corner[3][1] + corner[2][1] + corner[1][1] + corner[0][1]) / 4)
			corner_temp = []
			for i in range(4):
				dx = corner_cx - corner[i][0]
				dy = corner_cy - corner[i][1]
				x = corner[i][0] + int(dx * 0.06)
				y = corner[i][1] + int(dy * 0.08)
				corner_temp.append((x, y))
			corner = corner_temp
			p_array.append(corner[0])
			for i in range(4):
				next = i + 1
				if i == 3:
					next = 0
				dy = (corner[next][1] - corner[i][1]) / DIV_TASK_3
				dx = (corner[next][0] - corner[i][0]) / DIV_TASK_3
				for j in range(DIV_TASK_3):
					pointx = int(corner[i][0] + dx * (j + 1))
					pointy = int(corner[i][1] + dy * (j + 1))
					p_array.append((pointx, pointy))
			print(len(p_array))
			for p in p_array:
				img.draw_circle(p[0], p[1], 1)
		elif flgstart == 0:
			rects = img.find_rects(threshold = 1000)
			for r in rects:
				rect = r.rect()
				if (rect[0] > 15 and rect[1] > 15
					and rect[2] < 60 and rect[3] < 60
					and (rect[0] + rect[2]) < 100
					and (rect[1] + rect[3]) < 100
					and (rect[0] + rect[2]) > 25
					and (rect[1] + rect[3]) > 25):
					cnt_find_rect = cnt_find_rect + 1
					img.draw_rectangle(rect, color = (255, 0, 0))
					corner = r.corners()
					for point in r.corners():
						img.draw_circle(point[0], point[1], 5, color = (0, 255, 0))
					break
		print(len(p_array))
		if blob and flgstart == 1:
			print(goal_task4)
			err_dx = cx - p_array[goal_task4][0]
			err_dy = cy - p_array[goal_task4][1]
			if abs(err_dx) < 2 and abs(err_dy) < 2:
				goal_task4 = goal_task4 + 1
				if goal_task4 == len(p_array):
					task = b'0'
					goal_task4 = 0
					FH = bytearray([0xA1, 0x11, 0x1A])
					uart.write(FH)
					print("OK")
					flgstart = 0
					p_array = []
					cnt_find_rect = 0
		if flgstart == 1:
			sending_data(err_dx, err_dy)
	elif task == b'9':
		point_temp.append((cx, cy))
		img.draw_string(0, 0, "task:5")
		for p in point_temp:
			img.draw_circle(p[0], p[1], 1)
		FH = bytearray([0xA1, 0x44, 0x1A])
		uart.write(FH)
		task = b'0'
	elif task == b'6':
		img.draw_string(0, 0, "task:6")
	lcd.display(img)