
import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
cm = 38
marker_size = 7
border_size = 1

def generate_world_marker():
    marker_ids = [[200, 201,],
                  [202, 203,],
                  [204, 205,],]
    markers = np.full((3*(marker_size+2*border_size)*cm, 2*(marker_size+2*border_size)*cm), 255)
    # draw marker
    for i, ids in enumerate(marker_ids):
        for j, id in enumerate(ids):
            marker = cv2.aruco.generateImageMarker(aruco_dict, id, marker_size * cm)
            markers[(border_size+i*(marker_size+2*border_size))*cm:
                    (border_size+i*(marker_size+2*border_size))*cm+marker.shape[0],
                    (border_size+j*(marker_size+2*border_size))*cm:
                    (border_size+j*(marker_size+2*border_size))*cm+marker.shape[1]] = marker
    # draw line
    markers[0, :] = 0
    markers[:, 0] = 0
    for i, ids in enumerate(marker_ids):
        markers[((i+1)*(marker_size+2*border_size))*cm-1, :] = 0
        for j, id in enumerate(ids):
            markers[:, (j*(marker_size+2*border_size))*cm-1] = 0
    cv2.imwrite("world_marker.png", markers)


def generate_robot_marker():
    marker_ids = [[100, 101,],
                  [102, 103,],]
    markers = np.full(((2*(marker_size+border_size)+border_size)*cm,
                       (2*(marker_size+border_size)+border_size)*cm), 255)
    # draw marker
    for i, ids in enumerate(marker_ids):
        for j, id in enumerate(ids):
            marker = cv2.aruco.generateImageMarker(aruco_dict, id, marker_size * cm)
            markers[(border_size+i*(marker_size+border_size))*cm:
                    (border_size+i*(marker_size+border_size))*cm+marker.shape[0],
                    (border_size+j*(marker_size+border_size))*cm:
                    (border_size+j*(marker_size+border_size))*cm+marker.shape[1]] = marker
    # draw line
    markers[0, :] = 0
    markers[:, 0] = 0
    markers[-1, :] = 0
    markers[:, -1] = 0
    cv2.imwrite("robot_marker.png", markers)


def generate_block_marker():
    marker_ids = [[4,],
                  [5,],]
    block_id = 2
    markers = np.full(((2*(marker_size+border_size)+border_size)*cm, (marker_size+2*border_size+1)*cm,), 255)
    # draw marker
    for i, ids in enumerate(marker_ids):
        for j, id in enumerate(ids):
            marker = cv2.aruco.generateImageMarker(aruco_dict, id, marker_size * cm)
            markers[(border_size+i*(marker_size+border_size))*cm:
                    (border_size+i*(marker_size+border_size))*cm+marker.shape[0],
                    (border_size+j*(marker_size+border_size))*cm:
                    (border_size+j*(marker_size+border_size))*cm+marker.shape[1]] = marker
    # draw line
    markers[0, :] = 0
    markers[:, 0] = 0
    markers[-1, :] = 0
    markers[:, -1] = 0
    cv2.imwrite("block_{}_marker.png".format(block_id), markers)


def generate_all_marker():
    for id in range(50):
        # draw merker
        marker = cv2.aruco.generateImageMarker(aruco_dict, id, marker_size * cm)
        marker_with_border = np.full(((marker_size+2*border_size)*cm,
                                      (marker_size+2*border_size)*cm), 255)
        marker_with_border[border_size*cm:(border_size+marker_size)*cm,
                           border_size*cm:(border_size+marker_size)*cm,] = marker
        # draw line
        marker_with_border[0, :] = 0
        marker_with_border[:, 0] = 0
        marker_with_border[-1, :] = 0
        marker_with_border[:, -1] = 0
        cv2.imwrite("markers/marker_{}.png".format(id), marker_with_border)


if __name__ == "__main__":
    generate_all_marker()