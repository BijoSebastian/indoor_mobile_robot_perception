import cv2

# Create background subtractor object
background_subtractor = cv2.createBackgroundSubtractorMOG2()

# Capture video from webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply background subtraction
    fg_mask = background_subtractor.apply(frame)

    # Combine the foreground mask with the original frame
    fg_frame = cv2.bitwise_and(frame, frame, mask=fg_mask)

    # Display the frame with foreground
    cv2.imshow('Frame with Foreground', fg_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
