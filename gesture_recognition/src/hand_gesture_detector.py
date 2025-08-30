import cv2
import mediapipe as mp
import time

# FINGER INDEXES
THUMB = 0
INDEX = 1
MIDDLE = 2
RING = 3
PINKY = 4

# Landmark index for the wrist
WRIST_INDEX = 0

# Shortcuts for mp classes:
# The main class that runs the hand detection/tracking.
mp_hands = mp.solutions.hands 
# Provides helper functions to draw landmarks and connections on the image
mp_drawing = mp.solutions.drawing_utils

def get_finger_status(hand_landmarks):
	# each point on hand have index for example:
	# the wrist index is 0 the thumb finger [1,2,3,4] so it's tip is 4
	# and the pip is what point we want to determain if the 
	# finger is up or down
	fingers = []
	tips = [4, 8, 12, 16, 20] #the tips for each finger 
	pip = [3, 6, 10, 14, 18]
	front_hand_face = True
	# to Check if the hand is flipped or not
	if hand_landmarks.landmark[WRIST_INDEX].y > hand_landmarks.landmark[pip[MIDDLE]].y:
		print(hand_landmarks.landmark[WRIST_INDEX].y," ",hand_landmarks.landmark[pip[MIDDLE]].y)
		print("hand not flipped")
		# to Check if it is the front or the back hand face
		if hand_landmarks.landmark[pip[THUMB]].x > hand_landmarks.landmark[tips[PINKY]].x :
			print("front_face")
		# Thumb
			fingers.append(1 if hand_landmarks.landmark[tips[THUMB]].x < hand_landmarks.landmark[pip[THUMB]].x else 0)
		else:
			print("back_face")
			fingers.append(1 if hand_landmarks.landmark[tips[THUMB]].x > hand_landmarks.landmark[pip[THUMB]].x else 0)

		# Other fingers
		for i in range(1, 5):
			fingers.append(1 if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[pip[i]].y else 0)
	else:
		print(hand_landmarks.landmark[WRIST_INDEX].y," ",hand_landmarks.landmark[pip[MIDDLE]].y)
		print("hand flipped")
		if hand_landmarks.landmark[pip[THUMB]].x < hand_landmarks.landmark[tips[PINKY]].x :
			print("front_face")
		# Thumb
			fingers.append(1 if hand_landmarks.landmark[tips[THUMB]].x > hand_landmarks.landmark[pip[THUMB]].x else 0)
		else:
			print("back_face")
			fingers.append(1 if hand_landmarks.landmark[tips[THUMB]].x < hand_landmarks.landmark[pip[THUMB]].x else 0)

		# Other fingers
		for i in range(1, 5):
			fingers.append(1 if hand_landmarks.landmark[tips[i]].y > hand_landmarks.landmark[pip[i]].y else 0)
	return fingers


# ✅ Connect to phone stream
#cap = cv2.VideoCapture("http://192.168.0.75:8080/video")
cap = cv2.VideoCapture(0)

# Skip frames to lower FPS
TARGET_FPS = 10   # process ~10 frames per second
last_time = 0

# creates a Mediapipe Hands object (a model that detects and tracks hands)
with mp_hands.Hands(
	static_image_mode=False,
	max_num_hands=1,
	min_detection_confidence=0.6, #Minimum confidence for the initial hand detection (before tracking starts).
	min_tracking_confidence=0.6 # Confidence threshold for tracking hand landmarks across frames.
) as hands:
	
	while True:
		ret, frame = cap.read()
		if not ret:
			break

		# Limit FPS
		current_time = time.time()
		if (current_time - last_time) < 1.0 / TARGET_FPS:
			continue
		last_time = current_time

		# ✅ Rotate if needed
		#frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

		# Flip for mirror effect
		#frame = cv2.rotate(frame, 2)

		rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		results = hands.process(rgb)

		if results.multi_hand_landmarks:
			for hand_landmarks in results.multi_hand_landmarks:
				mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

				fingers = get_finger_status(hand_landmarks)
				gesture = ''.join(str(f) for f in fingers)
				cv2.putText(frame, f"{gesture}", (10, 50),
							cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
				
		cv2.imshow("Finger Gesture Detection", frame)

		if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
			break

cap.release()
cv2.destroyAllWindows()