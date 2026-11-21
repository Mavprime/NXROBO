import cv2

img = cv2.imread('/home/spark/current_view.jpg')
h, w, _ = img.shape
cv2.imwrite('/home/spark/crop1.jpg', img[max(0, 190-60):min(h, 190+60), max(0, 341-60):min(w, 341+60)])
cv2.imwrite('/home/spark/crop2.jpg', img[max(0, 250-40):min(h, 250+40), max(0, 343-55):min(w, 343+55)])
print("OK")
