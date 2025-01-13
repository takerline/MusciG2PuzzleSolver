import cv2

cap = cv2.VideoCapture(6)

num = 0

while cap.isOpened():

    succes, img = cap.read()

    k = cv2.waitKey(5)

    if k == ord('f'):
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite(r'img_gorka_' + str(num) + '.png', img)
        print("image saved!")

        num += 1

    # cv2.line(img, (0, 296), (img.shape[1], 296), (0, 255, 0), 2)

    # #Draw vertical line at x=377
    # cv2.line(img, (377, 0), (377, img.shape[0]), (0, 255, 0), 2)

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()


#udevprool la informacion del vendorj