import cv2

# Substitua pelos seus dados
rtsp_url = "rtsp://admin:admin123@192.168.0.111:554/play1.sdp"

cap = cv2.VideoCapture(rtsp_url)

if not cap.isOpened():
    print("Erro ao abrir o stream RTSP.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): # Pressione 'q' para sair
            break

cap.release()
cv2.destroyAllWindows()