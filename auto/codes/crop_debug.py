import cv2

img = cv2.imread(r"C:\Users\user1\.gemini\antigravity\brain\02c08ee7-abc0-4fcb-b5ee-bb7961f71adf\current_view.jpg")
h, w, _ = img.shape

# Crop around (341, 190) - size 112x100
x1, y1 = max(0, 341 - 56), max(0, 190 - 50)
x2, y2 = min(w, 341 + 56), min(h, 190 + 50)
crop1 = img[y1:y2, x1:x2]
cv2.imwrite(r"C:\Users\user1\.gemini\antigravity\brain\02c08ee7-abc0-4fcb-b5ee-bb7961f71adf\crop_top.jpg", crop1)

# Crop around (343, 250) - size 109x39
x1, y1 = max(0, 343 - 55), max(0, 250 - 25)
x2, y2 = min(w, 343 + 55), min(h, 250 + 25)
crop2 = img[y1:y2, x1:x2]
cv2.imwrite(r"C:\Users\user1\.gemini\antigravity\brain\02c08ee7-abc0-4fcb-b5ee-bb7961f71adf\crop_pedestal.jpg", crop2)
print("Saved crops.")
