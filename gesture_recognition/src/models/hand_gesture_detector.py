import cv2
import mediapipe as mp
import time
import numpy as np
from math import pi

# FINGER INDEXES
THUMB = 0
INDEX = 1
MIDDLE = 2
RING = 3
PINKY = 4

# Landmark index for the wrist and middle finger base
WRIST_INDEX = 0
BASE_MIDDLE_FINGER_INDEX = 9

# Shortcuts for mp classes:
# The main class that runs the hand detection/tracking.
mp_hands = mp.solutions.hands 
# Provides helper functions to draw landmarks and connections on the image
mp_drawing = mp.solutions.drawing_utils

class Point():
    def __init__(self, *args):
        if len(args) == 0:
            self.x, self.y = 0, 0
        elif len(args) == 1:
            self.x, self.y = args[0], args[0]
        elif len(args) == 2:
            self.x, self.y = args
        else:
            raise ValueError("Too many arguments")
		
    def __iter__(self):
        yield self.x
        yield self.y

class HandGestureDetector():
	def __init__(self,camera_address=0,fps = 10 , max_num_hands=1, min_detection_confidence=0.3, min_tracking_confidence=0.6):
		self.hands = mp_hands.Hands(
			static_image_mode=False,
			max_num_hands=max_num_hands,
			min_detection_confidence=min_detection_confidence,
			min_tracking_confidence=min_tracking_confidence
		)
		self.camera_address = camera_address
		self.cap = cv2.VideoCapture(self.camera_address)
		start_time = time.time()
		while True:
			print(f"Waiting for camera {self.camera_address} to open...")
			if self.cap.isOpened():
				print(f"Camera {self.camera_address} opened.")
				break
			if time.time() - start_time > 5:  # wait up to 5 seconds
				self.cap.release()
				raise TimeoutError(f"Failed to open camera source {self.camera_address}")
			else:
				self.cap = cv2.VideoCapture(self.camera_address)
				time.sleep(0.1)
		self.fps = fps
		self.indecies_relative_to_wrist = [Point() for _ in range(21)]
		self.running = False
		self.counted = False
		self.closed = False


	def get_finger_status(self,hand_landmarks):
		# each point on hand have index for example:
		# the wrist index is 0 the thumb finger [1,2,3,4] so it's tip is 4
		# and the pip is what point we want to determain if the 
		# finger is up or down
		fingers = []
		tips = [4, 8, 12, 16, 20] #the tips for each finger 
		pip = [3, 6, 10, 14, 18]
		
		# make hand indeses relative to the hand not the image
		# by multiplying the x and y for each point by rotation matrix
		rotation_angle = pi/2-np.arctan2(hand_landmarks.landmark[BASE_MIDDLE_FINGER_INDEX].y - hand_landmarks.landmark[WRIST_INDEX].y,
									hand_landmarks.landmark[BASE_MIDDLE_FINGER_INDEX].x - hand_landmarks.landmark[WRIST_INDEX].x)

		# initialize list for the relative coordinates with wrist
		
		for i in range(0,21):
			# get the coordinates relative to the wrist
			relative_x = -hand_landmarks.landmark[i].x + hand_landmarks.landmark[WRIST_INDEX].x
			relative_y = hand_landmarks.landmark[i].y - hand_landmarks.landmark[WRIST_INDEX].y
			relative_point = Point(relative_x, relative_y)
			# rotate the coordinates around the wrist
			rotated_coords = self._rotate_2d(np.array([[relative_point.x, relative_point.y]]), -np.degrees(rotation_angle))
			self.indecies_relative_to_wrist[i].x = rotated_coords[0][0]
			self.indecies_relative_to_wrist[i].y = rotated_coords[0][1]

		# to Check if it is the front or the back hand face
		# TODO: make finger detection work on relative coordinates
		if self.indecies_relative_to_wrist[pip[THUMB]].x > self.indecies_relative_to_wrist[tips[PINKY]].x :
		# Thumb
			fingers.append(1 if self.indecies_relative_to_wrist[tips[THUMB]].x > self.indecies_relative_to_wrist[pip[THUMB]].x else 0)
		else:
			fingers.append(1 if self.indecies_relative_to_wrist[tips[THUMB]].x < self.indecies_relative_to_wrist[pip[THUMB]].x else 0)

		# Other fingers
		for i in range(1, 5):
			fingers.append(1 if self.indecies_relative_to_wrist[tips[i]].y > self.indecies_relative_to_wrist[pip[i]].y else 0)

		return fingers
	
	def run(self):
		last_time = 0
		while True:
			ret, frame = self.cap.read()
			
			if not ret:
				start_time = time.time()
				while True:
					if ret:
						print(f"Camera returned.")
						break
					if time.time() - start_time > 5:  # wait up to 5 seconds
						self.cap.release()
						raise TimeoutError(f"camera disconnected")
					else:
						self.cap = cv2.VideoCapture(self.camera_address)
						ret, frame = self.cap.read()
						print(ret)
						print(f"Waiting for camera to return...")
						time.sleep(0.1)

			# Limit FPS
			current_time = time.time()
			if (current_time - last_time) < 1.0 / self.fps:
				continue
			last_time = current_time

			# ✅ Rotate if needed
			#frame = cv2.rotate(frame, -cv2.ROTATE_90_CLOCKWISE)

			# Flip for mirror effect
			#frame = cv2.rotate(frame, 2)

			rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			results = self.hands.process(rgb)

			if results.multi_hand_landmarks:
				for hand_landmarks in results.multi_hand_landmarks:
					mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

					fingers = self.get_finger_status(hand_landmarks)
					self.running = True
					self.gesture = ''.join(str(f) for f in fingers)
					cv2.putText(frame, f"{self.gesture}", (10, 50),
								cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
			else:
				self.running = False	

			cv2.imshow("Finger Gesture Detection", frame)
			key = cv2.waitKey(1) & 0xFF
			if key == ord('a'):  # ESC to exit
				self.counted  = not self.counted

			if key == ord('q'):  # ESC to exit
				break

		self.cap.release()
		cv2.destroyAllWindows()
		self.closed = True

	def _rotate_2d(self,coords, angle_deg):
		angle = np.radians(angle_deg)
		# 2D rotation matrix
		rot_matrix = np.array([
			[np.cos(angle), -np.sin(angle)],
			[np.sin(angle),  np.cos(angle)]
		])
		return coords @ rot_matrix.T  # matrix multiply


if __name__ == "__main__":
	detector = HandGestureDetector(camera_address=0, fps=10)
	detector.run()
