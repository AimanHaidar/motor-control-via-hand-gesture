import cv2
import mediapipe as mp
import time

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

def get_finger_status(hand_landmarks):
	fingers = []
	tips = [4, 8, 12, 16, 20]
	pip = [3, 6, 10, 14, 18]
	front_hand_face = True
	# to Check if the hand is flipped or not
	if hand_landmarks.landmark[0].y > hand_landmarks.landmark[pip[2]].y:
		print(hand_landmarks.landmark[0].y," ",hand_landmarks.landmark[pip[2]].y)
		print("hand not flipped")
		# to Check if it is the front or the back hand face
		if hand_landmarks.landmark[pip[0]].x > hand_landmarks.landmark[tips[4]].x :
			print("front_face")
		# Thumb
			fingers.append(1 if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[pip[0]].x else 0)
		else:
			print("back_face")
			fingers.append(1 if hand_landmarks.landmark[tips[0]].x > hand_landmarks.landmark[pip[0]].x else 0)

		# Other fingers
		for i in range(1, 5):
			fingers.append(1 if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[pip[i]].y else 0)
	else:
		print(hand_landmarks.landmark[0].y," ",hand_landmarks.landmark[pip[2]].y)
		print("hand flipped")
		if hand_landmarks.landmark[pip[0]].x < hand_landmarks.landmark[tips[4]].x :
			print("front_face")
		# Thumb
			fingers.append(1 if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[pip[0]].x else 0)
		else:
			print("back_face")
			fingers.append(1 if hand_landmarks.landmark[tips[0]].x > hand_landmarks.landmark[pip[0]].x else 0)

		# Other fingers
		for i in range(1, 5):
			fingers.append(1 if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[pip[i]].y else 0)
	return fingers

