# Copyright 2022 Agricultural-Robotics-Bonn
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import cv2 as cv

from .geometric import *


def getContourCenter(contours):
    """function to compute the center points of the contours

    Args:
        contours (_type_): contours from image

    Returns:
        _type_: contours centers
    """
    # get center of contours
    contCenterPTS = np.zeros((len(contours), 2))
    for i in range(0, len(contours)):
        # get contour
        c_curr = contours[i]

        # get moments
        M = cv.moments(c_curr)

        # compute center of mass
        if (M['m00'] != 0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            contCenterPTS[i, :] = [cy, cx]

    contCenterPTS = contCenterPTS[~np.all(contCenterPTS == 0, axis=1)]
    return contCenterPTS


def getPlantMasks(binaryMask, min_contour_area, bushy=False, max_contour_height=100):
    """Функция для получения масок растений с разделением высоких контуров
    
    Args:
        binaryMask: бинарная маска изображения
        min_contour_area: минимальная площадь контура
        bushy: флаг для кустистых растений
        max_contour_height: максимальная высота контура перед разделением
        
    Returns:
        Список контуров растений
    """
    contours = cv.findContours(binaryMask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)[0]
    filtered_contours = []
    
    for contour in contours:
        if cv.contourArea(contour) < min_contour_area:
            continue
            
        x, y, w, h = cv.boundingRect(contour)
        
        if h > max_contour_height:
            # Создаем маску для всего изображения
            mask = np.zeros(binaryMask.shape[:2], dtype=np.uint8)
            cv.drawContours(mask, [contour], -1, 255, -1)
            
            num_parts = int(np.ceil(h / max_contour_height))
            part_height = h / num_parts
            
            for i in range(num_parts):
                y_start = y + int(i * part_height)
                y_end = y + int((i + 1) * part_height)
                
                # Вырезаем часть маски
                mask_part = np.zeros_like(mask)
                mask_part[y_start:y_end, :] = mask[y_start:y_end, :]
                
                # Находим контуры в этой части
                part_contours, _ = cv.findContours(mask_part, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
                
                for cnt in part_contours:
                    if cv.contourArea(cnt) > min_contour_area/2:
                        filtered_contours.append(cnt)
        else:
            if bushy:
                sub_contours = splitContours(contour, x, y, w, h)
                filtered_contours.extend([c for c in sub_contours if len(c) > 0])
            else:
                filtered_contours.append(contour)
    
    return filtered_contours

def splitContours(contour, x, y, w, h, max_coutour_height):
    """splits larg contours in smaller regions 

    Args:
        contour (_type_): _description_
        x (_type_): _description_
        y (_type_): _description_
        w (_type_): _description_
        h (_type_): _description_

    Returns:
        _type_: sub polygons (seperated countours)
    """
    sub_polygon_num = h // max_coutour_height
    sub_polys = list()
    subContour = list()
    vtx_idx = list()
    contour = [contour.squeeze().tolist()]
    for subPoly in range(1, sub_polygon_num + 1):
        for vtx in range(len(contour[0])):
            if (subPoly - 1 * max_coutour_height) - 1 <= contour[0][vtx][1] and \
                    (subPoly * max_coutour_height) - 1 >= contour[0][vtx][1] and \
                    vtx not in vtx_idx:
                subContour.append([contour[0][vtx]])
                vtx_idx.append(vtx)

        sub_polys.append(np.array(subContour))
        subContour = list()

    return sub_polys


def sortContours(contours, method="left-to-right"):
    """initialize the reverse flag and sort index

    Args:
        cnts (_type_): _description_
        method (str, optional): _description_. Defaults to "left-to-right".

    Returns:
        _type_: sorted countours, bboxes
    """
    reverse = False
    i = 0
    # handle if we need to sort in reverse
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True
    # handle if we are sorting against the y-coordinate rather than
    # the x-coordinate of the bounding box
    if method == "top-to-bottom" or method == "bottom-to-top":
        i = 1
    # construct the list of bounding boxes and sort them from top to
    # bottom
    boundingBoxes = [cv.boundingRect(c) for c in contours]
    (contours, boundingBoxes) = zip(*sorted(zip(contours, boundingBoxes),
                                            key=lambda b: b[1][i], reverse=reverse))
    # return the list of sorted contours and bounding boxes
    return contours, boundingBoxes


def getContoursInWindow(contourCenters, box):
    """iflters out countours inside a box

    Args:
        contourCenters (_type_): _description_
        box (_type_): _description_

    Returns:
        _type_: contour centers
    """
    points = []
    for cnt in range(len(contourCenters[1])):
        x, y = contourCenters[0][cnt], contourCenters[1][cnt]
        if isInBox(list(box), [x, y]):
            points.append([x, y])
    return points