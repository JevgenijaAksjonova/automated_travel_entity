#! /usr/bin/env python
from __future__ import print_function

import pprint
from os import listdir, makedirs, path
from shutil import copy2

import numpy as np

import cv2
import math
import yaml

import random
pp = pprint.PrettyPrinter(indent=4)
#rec = Recognizer()p

MIDDLE_PIXEL = np.r_[640//2,480//2]

#Thresholds in hue for all collors
colors = ["green", "red", "blue", "yellow", "purple", "orange"]

#map of (lower,upper) thresholds in hsv for the respective colors
#HSV values in open cv are given in ranges (0-180,0-255,0-255) respectively

#Used soly for visualisation purposes
color_2_rgb = {
    "green": (0, 255, 0),
    "red": (255, 0, 0),
    "blue": (0, 0, 255),
    "yellow": (255, 255, 0),
    "purple": (128, 0, 128),
    "orange": (255, 165, 0),
    "error": (128, 128, 128),
}

DEBUGGING = False


#Function to extract a widnow from a object in a manner that perserves perspectives
def extract_object_image(middle_point, top_left, bot_right, image):
    #TODO: Fix bug where right side of image is cut of to early
    (x_mid, y_mid) = middle_point
    (x_min, y_min) = top_left
    (x_max, y_max) = bot_right
    #Calculate the width of the window
    window_width = max(x_max - x_min, y_max - y_min)
    alignment = int((window_width // 2) * 2)

    #Create preliminary new window
    new_x_min = x_mid - alignment
    new_x_max = x_mid + alignment
    new_y_min = y_mid - alignment
    new_y_max = y_mid + alignment
    
    adjusted_window = False
    #Check if resulting window is outside of image
    if new_x_max >= image.shape[1]:
        new_x_mid = x_mid - (new_x_max - image.shape[1])
        adjusted_window = True
    elif new_x_min < 0:
        new_x_mid = x_mid - new_x_min
        adjusted_window = True
    else:
        new_x_mid = x_mid

    if new_y_max >= image.shape[0]:
        new_y_mid = y_mid - (new_y_max - image.shape[0])
        adjusted_window = True
    elif new_y_min < 0:
        new_y_mid = y_mid - new_y_min
        adjusted_window = True
    else:
        new_y_mid = y_mid
    distance_from_center = np.linalg.norm(np.r_[new_x_mid,new_y_mid] - MIDDLE_PIXEL)
    #Create new window
    new_x_min = new_x_mid - alignment
    new_x_max = new_x_mid + alignment
    new_y_min = new_y_mid - alignment
    new_y_max = new_y_mid + alignment

    return image[new_y_min:new_y_max, new_x_min:new_x_max, :], adjusted_window, distance_from_center

#An object representic an area classified by the algorithm as a candidate area.
class ObjectCandidate(object):
    def __init__(self, x_min, x_max, y_min, y_max, color,cont_area):

        self.bot_right = (x_max, y_max)
        self.top_left = (x_min, y_min)

        x_mid = (x_max + x_min) // 2
        y_mid = (y_max + y_min) // 2
        self.mid = (x_mid, y_mid)

        center_point_x = x_mid
        center_point_y = (y_max + 2 * y_min) // 3
        self.center_point = (center_point_x, center_point_y)

        self.widht = (x_max - x_min)
        self.height = (y_max - y_min)
        self.area = self.height * self.widht
        self.color = color
        self.contour_area = cont_area
    @property
    def score(self):
        return self.distance_from_center * self.contour_area / (4.0 if self.adjusted else 1.0)

    def find_img(self, full_rgb_img):
        self.img,self.adjusted,self.distance_from_center = extract_object_image(self.mid, self.top_left,
                                        self.bot_right, full_rgb_img)
        (x, y, z) = self.img.shape
        return x * y * z != 0

    def find_depth(self, full_depth_img):
        x_range = slice(self.center_point[1]-10,self.center_point[1]+10)
        y_range = slice(self.center_point[0]-10,self.center_point[0]+10)
        self.z = np.nanmean(full_depth_img[x_range,y_range])
        return self.z is None or math.isnan(self.z)

    def __repr__(self):
        return "contour_area = {contour_area}, area = {area} ,color = {color}, adjusted = {adjusted}, distance from center = {distance_from_center}, score = {score}".format(
            contour_area = self.contour_area,area = self.area, color = self.color, adjusted=self.adjusted,distance_from_center = self.distance_from_center, score = self.score)

default_hsv_thresh = {
    "green": (np.array([40, 110, 80]), np.array([85, 255, 230])),
    "blue": (np.array([18, 110, 40]), np.array([35, 255, 230])),
    "yellow": (np.array([90, 190, 80]), np.array([100, 255, 240])),
    "purple": (np.array([125, 30, 30]), np.array([170, 255, 255])),
    "red": (np.array([110, 110, 80]), np.array([120, 255, 240])),
    "orange": (np.array([107, 150, 80]), np.array([115, 255, 240])),
}


_mask_kernel = np.ones((3, 3), np.uint8)
def compute_mask(image, bounds):
    lower_bound, upper_bound = bounds
    mask = cv2.inRange(image, lower_bound, upper_bound)
    cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel=_mask_kernel,
        dst=mask,
        iterations=10)
    return mask

#Process image :D
def color_segment_image(bgr_image,
                        depth_image = None,
                        return_debug_image=False,
                        apply_checks=True,
                        hsv_thresholds = default_hsv_thresh):
    #IDEA! Remove lense glare detections by removing blue objects detected on top
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    #hsv_image = cv2.medianBlur(hsv_image,11)

    object_candidates = []
    #to_many_object_candidates = False
    if return_debug_image:
        bgr_dbg = bgr_image.copy()

    for color in ["red","green","blue","purple"]:
        #if to_many_object_candidates:
            #break
        mask = compute_mask(hsv_image, hsv_thresholds[color])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            #if to_many_object_candidates:
                #break
            contour = cv2.convexHull(contour)
            cont_area = cv2.contourArea(contour)
            x_min = int(contour[:, 0, 0].min())
            x_max = int(contour[:, 0, 0].max())
            y_min = int(contour[:, 0, 1].min())
            y_max = int(contour[:, 0, 1].max())
            detected_obj = ObjectCandidate(x_min, x_max, y_min, y_max,
                                            color,cont_area)
            
            #resonability checks
            im_width, im_height, im_channels = bgr_image.shape
            im_area = float(im_width * im_height)
            area_too_big = cv2.contourArea(contour) > im_area / 4
            width_too_large = detected_obj.widht > float(im_height) / 1.5
            height_too_large = detected_obj.height > float(im_width) / 1.5
            checks_ok = not (area_too_big or width_too_large or
                                height_too_large) if apply_checks else True

            if not checks_ok:
                print("area_too_big =", area_too_big)
                print("width_too_large =", width_too_large)
                print("height_too_large =", height_too_large)
            img_ok = detected_obj.find_img(bgr_image)
            depth_ok = detected_obj.find_depth(depth_image)
            #print(img_ok and depth_ok and checks_ok)
            #if img_ok and depth_ok and checks_ok:
            object_candidates.append(detected_obj)
            if return_debug_image:
                cv2.drawContours(
                    bgr_dbg, [contour],
                    -1,
                    color=color_2_rgb[color]
                    if checks_ok else color_2_rgb["error"],
                    thickness=-1)
                cv2.rectangle(
                    bgr_dbg,
                    detected_obj.top_left,
                    detected_obj.bot_right,
                    color=(0, 0, 0),
                    thickness=2)
                cv2.circle(
                    bgr_dbg,
                    detected_obj.center_point,
                    radius=5,
                    color=(0, 0, 0),
                    thickness=2)

            #to_many_object_candidates = len(
                #object_candidates) > 8 if apply_checks else False         

    object_candidates, bgr_dbg = (
        object_candidates, bgr_dbg) if True else (#not to_many_object_candidates else (
            [], bgr_image)
    #if to_many_object_candidates:
        #print("Too many object candidates")

    if return_debug_image:
        return object_candidates, bgr_dbg
    else:
        return object_candidates


def save_dataset_object_candidates(dataset_base_dir,
                                   save_dir,
                                   include_debug=False,
                                   do_checks=True):
    #TODO: Deal with images with no detections.
    #TODO: Deal with all colors.
    no_obj_dir = path.join(save_dir, "No Object")
    makedirs(no_obj_dir)
    class_dirs = listdir(dataset_base_dir)
    allowed_extentions = [".jpg", ".jpeg", ".png"]

    if include_debug:
        stats = {class_dir: dict() for class_dir in class_dirs + colors}

    for class_dir in class_dirs:
        
        class_save_dir = path.join(save_dir, class_dir)
        makedirs(class_save_dir)
        class_load_dir = path.join(dataset_base_dir, class_dir)
        if not path.isdir(class_load_dir):
            continue

        if include_debug:
            class_no_obj_dir = path.join(no_obj_dir, class_dir)
            makedirs(class_no_obj_dir)
            n_not_detected = 0


        im_files = listdir(class_load_dir)
        for im_file in im_files:
            im_path = path.join(dataset_base_dir, class_dir, im_file)
            if path.splitext(im_path)[1] not in allowed_extentions:
                continue

            img = cv2.imread(im_path, 3)

            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            object_candidates, dbg_img = color_segment_image(
                img, return_debug_image=True,apply_checks=do_checks)
            if len(object_candidates) > 0:

                if include_debug:
                    image_save_dir = path.join(class_save_dir,
                                               path.splitext(im_file)[0])
                    makedirs(image_save_dir)
                else:
                    image_save_dir = class_save_dir

                for i, oc in enumerate(object_candidates):
                    img = cv2.cvtColor(oc.img, cv2.COLOR_BGR2RGB)
                    if oc.color not in class_dir.lower() and include_debug:
                        if "false_detections" in stats[oc.color]:
                            stats[oc.color]["false_detections"] += 1
                        else:
                            stats[oc.color]["false_detections"] = 1

                    save_fn = path.join(
                        image_save_dir,
                        "detection_{i}_{color}_{rnd}.jpg".format(
                            i=i, color=oc.color, rnd=random.randint(0, 9999)))

                    cv2.imwrite(save_fn, img)

                if include_debug:
                    dbg_img_fn = path.join(image_save_dir, "debug.jpg")
                    dbg_img = cv2.cvtColor(dbg_img, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(dbg_img_fn, dbg_img)
                    orig_fn = path.join(image_save_dir, "original.jpg")
                    copy2(im_path, orig_fn)

            elif include_debug:
                n_not_detected += 1
                save_fn = path.join(
                    class_no_obj_dir, "{fn}.jpg".format(fn=im_file))
                copy2(im_path, save_fn)

        if include_debug:
            n_detected = len(im_files) - n_not_detected
            stats[class_dir] = dict(stats[class_dir], **{
                "total": len(im_files),
                "detected": n_detected,
                "not_detected": n_not_detected,
                "not_detected/detected":
                float(n_not_detected) / float(n_detected)
                if n_detected != 0 else float("inf"),
            })
    if include_debug:
        with open(path.join(save_dir, "stats.yaml"), "w") as stats_file:
            yaml.dump(stats, stats_file, default_flow_style=False)


if __name__ == "__main__":

    objectCandidates = save_dataset_object_candidates(
        "/home/chm/Source/image_dataset_orginal/image_dataset",
        "oc_dataset",
        include_debug=False,
        do_checks=False)
