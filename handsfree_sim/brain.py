import cv2 as cv
import mediapipe as mp

""" Importing the hands module """
mp_hands = mp.solutions.hands

""" Creating an object of the hands class """
hands = mp_hands.Hands(
    model_complexity=0,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

def movement(image):
    """ Flipping the image for a selfie-view display """
    image = cv.flip(image, 1) 
    
    """ To improve performance, mark the image as not writeable. """
    image.flags.writeable = False
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    results = hands.process(image)
    
    image.flags.writeable = True
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    
    if results.multi_hand_landmarks not in (None, []):
        for hand_landmark in results.multi_hand_landmarks:
            
            """ Storing the coordinates of the fingers (tips and pips) """
            thumb_tip = hand_landmark.landmark[mp_hands.HandLandmark.THUMB_TIP]
            thumb_mcp = hand_landmark.landmark[mp_hands.HandLandmark.THUMB_MCP]
            index_tip = hand_landmark.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            index_pip = hand_landmark.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
            middle_tip = hand_landmark.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            middle_pip = hand_landmark.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            ring_tip = hand_landmark.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
            ring_pip = hand_landmark.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
            pinky_tip = hand_landmark.landmark[mp_hands.HandLandmark.PINKY_TIP]
            pinky_pip = hand_landmark.landmark[mp_hands.HandLandmark.PINKY_PIP]
            
            """ Condition for all the fingers to be straight """
            if thumb_tip.y < thumb_mcp.y and index_tip.y < index_pip.y and middle_tip.y < middle_pip.y and ring_tip.y < ring_pip.y and pinky_tip.y < pinky_pip.y:
                
                if index_tip.x < index_pip.x: return 'l' # (Turtle turns left)
                
                else: return 'r' # (Turtle turns right)
            
            elif index_tip.y < index_pip.y and middle_tip.y > middle_pip.y and ring_tip.y > ring_pip.y and pinky_tip.y > pinky_pip.y:
                
                """ Condition for only the index finger to be straight """
                if index_tip.x < index_pip.x: return 'ls' # (Turtle goes left on its spot)
                
                else: return 'rs' # (Turtle goes right on its spot)          
            
            else:
                """ Condition for a fist (turtle stops) """
                return 'n'