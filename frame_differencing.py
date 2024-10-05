import cv2
import numpy as np
from PIL import Image
import os
import glob

vid_cap = cv2.VideoCapture(0)


# to make the motion detection frame more exposed
def get_mask(frame1, frame2, kernel=np.array((9,9), dtype=np.uint8)):
    """ Obtains image mask
        Inputs: 
            frame1 - Grayscale frame at time t
            frame2 - Grayscale frame at time t + 1
            kernel - (NxN) array for Morphological Operations
        Outputs: 
            mask - Thresholded mask for moving pixels
        """

    frame_diff = cv2.subtract(frame2, frame1)

    # blur the frame difference
    frame_diff = cv2.medianBlur(frame_diff, 3)
    
    mask = cv2.adaptiveThreshold(frame_diff, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY_INV, 11, 3)

    mask = cv2.medianBlur(mask, 3)

    # morphological operations
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    return mask


# ***************************************** code from src ***************************************************
def remove_contained_bboxes(boxes):
    """ Removes all smaller boxes that are contained within larger boxes.
        Requires bboxes to be sorted by area (score)
        Inputs:
            boxes - array bounding boxes sorted (descending) by area 
                    [[x1,y1,x2,y2]]
        Outputs:
            keep - indexes of bounding boxes that are not entirely contained 
                   in another box
        """
    # bubble sort ??
    check_array = np.array([True, True, False, False])
    keep = list(range(0, len(boxes)))
    for i in keep: # range(0, len(bboxes)):
        for j in range(0, len(boxes)):
            # check if box j is completely contained in box i
            if np.all((np.array(boxes[j]) >= np.array(boxes[i])) == check_array):
                try:
                    keep.remove(j)
                except ValueError:
                    continue
    return keep


def non_max_suppression(boxes, scores, threshold=1e-1):
    """
    Perform non-max suppression on a set of bounding boxes and corresponding scores.
    Inputs:
        boxes: a list of bounding boxes in the format [xmin, ymin, xmax, ymax]
        scores: a list of corresponding scores 
        threshold: the IoU (intersection-over-union) threshold for merging bounding boxes
    Outputs:
        boxes - non-max suppressed boxes
    """
    # Sort the boxes by score in descending order
    boxes = boxes[np.argsort(scores)[::-1]]

    # remove all contained bounding boxes and get ordered index
    order = remove_contained_bboxes(boxes)

    keep = []
    while order:
        i = order.pop(0)
        keep.append(i)
        for j in order:
            # Calculate the IoU between the two boxes
            intersection = max(0, min(boxes[i][2], boxes[j][2]) - max(boxes[i][0], boxes[j][0])) * \
                           max(0, min(boxes[i][3], boxes[j][3]) - max(boxes[i][1], boxes[j][1]))
            union = (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1]) + \
                    (boxes[j][2] - boxes[j][0]) * (boxes[j][3] - boxes[j][1]) - intersection
            iou = intersection / union

            # Remove boxes with IoU greater than the threshold
            if iou > threshold:
                order.remove(j)
                
    return boxes[keep]
# ***************************************** code from src *******************************************************

def motion_contours(mask, thresh=500): # threshold value for pixel based bounding box area around the moving object
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1) # cv2.RETR_TREE

    detects = []
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        bbox_area = w*h
        if bbox_area > thresh:
            detects.append([x,y,x+w, y+h, bbox_area])
        
    return np.array(detects)


    


def object_motion():
    prev_frame = None
    idx=1
    while True:
        ret, frame = vid_cap.read()

        if ret == False:
            return
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if prev_frame is not None:
            # frame_diff = cv2.subtract(gray_frame, prev_frame)
            # compute motion mask
            kernel = np.array((9,9), dtype=np.uint8)
            mask = get_mask(gray_frame, prev_frame, kernel)
            detections = motion_contours(mask)

            if len(detections) > 0 and detections.ndim == 2:
                # separate bboxes and scores
                bboxes = detections[:, :4]
                scores = detections[:, -1]
                # Get Non-Max Suppressed Bounding Boxes
                nms_bboxes = non_max_suppression(bboxes, scores, threshold=0.1)
                if nms_bboxes is not None:
                    for box in nms_bboxes:
                        cv2.rectangle(mask, (box[0], box[1]), (box[2], box[3]), (255,0,0), 1)
            
            cv2.imshow("Frame Differencing", mask)
            cv2.imwrite(f"image_{idx}.png", mask)
            idx+=1


        prev_frame = gray_frame

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or idx==50:
            break
    vid_cap.release()
    cv2.destroyAllWindows()



def create_gif_from_images(save_path : str, image_path : str, ext : str):
    ext = ext.lstrip('.')
    image_paths = sorted(glob.glob(os.path.join(image_path, f'*.{ext}'))) 
    image_paths.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
    pil_images = [Image.open(im_path) for im_path in image_paths]

    pil_images[0].save(save_path, format='GIF', append_images=pil_images, save_all=True, duration=50, loop=0)





def main():
    #object_motion()
    create_gif_from_images('frame_differencing.gif', '', '.png')


if __name__ == "__main__":
    main()