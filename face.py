import cv2
face_cascade = cv2.CascadeClassifier(
    'D:/ANDA/Lib/site-packages/cv2/data/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(
    'D:/ANDA/Lib/site-packages/cv2/data/haarcascade_eye.xml')
cap = cv2.VideoCapture(0)
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(
    'M', 'J', 'P', 'G'), 10, (640, 480))
while True:
    ret, image = cap.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 10)
    print("faces:", len(faces))
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
        face_img = image[y:y + h, x:x + w]
        eye = eye_cascade.detectMultiScale(face_img, 1.1, 10)
        for (ex, ey, ew, eh) in eye:
            cv2.rectangle(image, (x + ex, y + ey),
                          (x + ex + ew, y + ey + eh), (255, 0, 0), 2)
    cv2.imshow('Video', image)
    out.write(image)
    k = cv2.waitKey(1)
    if k == 27:
        break
cap.release()
out.release()
cv2.destroyAllWindows()