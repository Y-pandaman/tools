import cv2

# video to photo
def video_to_photo(video_path=None, photo_path=None):
    vc = cv2.VideoCapture(video_path)
    c = 1
    if vc.isOpened():
        rval, frame = vc.read()
    else:
        rval = False
    while rval:
        rval, frame = vc.read()
        cv2.imwrite(photo_path + int_2_str(c) + '.jpg', frame)
        c = c + 1
        cv2.waitKey(1)
    vc.release()


# int to str, length 5
def int_2_str(number=0):
    number_str = str(number)
    if len(number_str) < 5:
        number_str = (5 - len(number_str)) * '0' + number_str
    return number_str

if __name__=='__main__':
    video_path = "/home/ubuntu/workspace/data/video/sineva.mp4"
    photo_path = "/home/ubuntu/workspace/data/video/sineva/"
    video_to_photo(video_path, photo_path)
