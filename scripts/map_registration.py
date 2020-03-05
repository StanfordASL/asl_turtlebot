#!/usr/bin/env python

import argparse
import base64
import json
import socket
import threading

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import redis
import rospy

from sensor_msgs.msg import CompressedImage

class imagereg:

    @staticmethod
    def register(img_0, img_1, plot=False):
        if plot:
            plt.figure()
            idx_subplots = [231, 234]
        else:
            idx_subplots = [None, None]
        img_lp_0 = imagereg.logpolar(img_0, idx_subplots[0])
        img_lp_1 = imagereg.logpolar(img_1, idx_subplots[1])

        idx_th, idx_a = imagereg.phase_correlation(img_lp_0, img_lp_1, plot)
        if abs(idx_a) > 1:
            return None, np.array([0., 0.]), 0., 0.

        th, a = imagereg.compute_angle_scale(img_lp_0.shape, idx_th, 0)
        print("Angle: {} => {}, Scale: {} => {}".format(idx_th, th, idx_a, a))

        img_r_0 = imagereg.rotate_scale(img_0, th, a)
        idx_y, idx_x = imagereg.phase_correlation(img_r_0, img_1, plot)
        x, y = imagereg.compute_translation(img_r_0.shape, idx_x, idx_y)
        print("Translation: ({}, {})".format(x, y))
        img_reg_0 = imagereg.translate(img_r_0, x, y)

        if plot:
            plt.figure()
            plt.subplot(141)
            plt.imshow(img_0, cmap='gray')
            plt.title('Image 0')

            plt.subplot(142)
            plt.imshow(img_r_0, cmap='gray')
            plt.title('Rot 0')

            plt.subplot(143)
            plt.imshow(img_reg_0, cmap='gray')
            plt.title('Trans 0')

            plt.subplot(144)
            plt.imshow(img_1, cmap='gray')
            plt.title('Image 1')

            plt.figure()
            plt.subplot(121)
            plt.imshow(img_reg_0.astype(np.float32) + img_1.astype(np.float32), cmap='gray')
            plt.title('Image sum')
            plt.colorbar()

            plt.subplot(122)
            plt.imshow(img_reg_0.astype(np.float32) - img_1.astype(np.float32), cmap='gray')
            plt.title('Image diff')
            plt.colorbar()

            plt.show()

        return img_reg_0, np.array([x, y]), th, a

    @staticmethod
    def logpolar(img, idx_subplot=None):
        dim = img.shape
        half_dim = tuple((np.array(dim) / 2.).tolist())

        # Compute FFT of image
        img_f = np.fft.fftshift(np.fft.fft2(img))

        # Ensure FFT of real image is symmetric with conjugate (F[i,j] = F*[-i,-j])
        img_f_sym = np.conj(np.fliplr(np.flipud(img_f)))
        img_f = (img_f + img_f_sym) / 2.
        img_m = np.log(np.abs(img_f))

        # img_lp = cv.logPolar(img_f, half_dim, max(half_dim)/np.log(max(dim)), cv.INTER_CUBIC)
        img_lp = cv.linearPolar(img_m, half_dim, max(half_dim), cv.INTER_CUBIC)
        img_lp = cv.resize(img_lp, (991, 360), interpolation=cv.INTER_CUBIC)

        # Plot image, fft, and logpolar
        if idx_subplot is not None:
            plt.subplot(idx_subplot)
            plt.imshow(img, cmap='gray')
            plt.title('Image')
            plt.colorbar()

            plt.subplot(idx_subplot+1)
            plt.imshow(img_m, cmap='plasma')
            plt.title('FFT')
            plt.colorbar()

            plt.subplot(idx_subplot+2)
            plt.imshow(img_lp, cmap='plasma')
            plt.title('Log polar')
            plt.colorbar()

        return img_lp

    @staticmethod
    def phase_correlation(img_0, img_1, plot=False):
        def neighborhood(img, row, col, radius):
            if row - radius >= 0 and row + radius < img.shape[0] and \
               col - radius >= 0 and col + radius < img.shape[1]:
                return img[row-radius:row+radius+1, col-radius:col+radius+1]

            block = np.zeros((2*radius+1, 2*radius+1))
            for i in range(block.shape[0]):
                ii = (i + row - radius) % img.shape[0]
                for j in range(block.shape[1]):
                    jj = (j + col - radius) % img.shape[1]
                    block[i,j] = img[ii,jj]
            return block

        def center_of_mass(img):
            dim = img.shape
            II, JJ = np.meshgrid(np.arange(dim[0]), np.arange(dim[1]), indexing='ij')
            img_sum = img.flatten().sum()
            i = (II * img).flatten().sum() / img_sum
            j = (JJ * img).flatten().sum() / img_sum
            return i, j

        # Take fourier transform of both images
        img_f_0 = np.fft.rfft2(img_0)
        img_f_1 = np.fft.rfft2(img_1)

        # Compute cross correlation in frequency domain
        img_cc_f = img_f_0 * np.conj(img_f_1) / np.abs(img_f_0 * img_f_1)

        # Convert back to spatial domain
        img_cc = np.real(np.fft.irfft2(img_cc_f))

        # Extract precise peak location
        idx_row, idx_col = np.unravel_index(np.argmax(img_cc, axis=None), img_cc.shape)
        print(idx_row, idx_col)
        radius = 1
        N = neighborhood(img_cc, idx_row, idx_col, radius)
        drow, dcol = center_of_mass(N)
        row = idx_row + drow - radius
        col = idx_col + dcol - radius

        # img_cc_flat = img_cc[:,0]
        # row = np.argmax(img_cc_flat)
        # print(row, img_cc_flat[row], row * 180./np.pi)

        if plot:
            plt.figure()
            plt.subplot(231)
            plt.imshow(img_0, cmap='gray')
            plt.title('Image')
            plt.colorbar()

            plt.subplot(232)
            plt.imshow(np.log(np.abs(img_f_0)), cmap='plasma')
            plt.title('FFT')
            plt.colorbar()

            plt.subplot(233)
            plt.imshow(img_cc, cmap='plasma')
            plt.title('Cross correlation')
            plt.colorbar()

            plt.subplot(234)
            plt.imshow(img_1, cmap='gray')
            plt.title('Image')
            plt.colorbar()

            plt.subplot(235)
            plt.imshow(np.log(np.abs(img_f_1)), cmap='plasma')
            plt.title('FFT')
            plt.colorbar()

            ax=plt.subplot(236)
            cax = ax.matshow(N, cmap='plasma')
            plt.title('Neighborhood')
            plt.colorbar(cax)

            plt.figure()
            plt.subplot(211)
            plt.plot(img_cc[:,idx_col])
            plt.title("Cross correlation: col {}".format(idx_col))

            plt.subplot(212)
            plt.plot(img_cc[idx_row,:])
            plt.title("Cross correlation: row {}".format(idx_row))

        return row, col

    @staticmethod
    def compute_angle_scale(dim, idx_th, idx_a):
        th = idx_th / dim[0] * 360.

        half_dim = tuple((np.array(dim) / 2.).tolist())
        log_scale = max(half_dim) / np.log(max(dim))
        log_base = np.exp(1. / log_scale)
        a = np.power(log_base, idx_a)
        return th, a

    @staticmethod
    def compute_translation(dim, idx_x, idx_y):
        x = np.array([idx_x, idx_y])
        dim = np.array(dim)

        x = np.mod(x - dim / 2., dim) - dim / 2.
        return -x[0], -x[1]

    @staticmethod
    def rotate_scale(img, th, a):
        R = cv.getRotationMatrix2D((img.shape[1]/2., img.shape[0]/2.), th, a)
        img_r = cv.warpAffine(img, R, img.shape)
        return img_r

    @staticmethod
    def translate(img, x, y):
        T = np.array([[1., 0., x],
                      [0., 1., y]])
        img_t = cv.warpAffine(img, T, img.shape)
        return img_t

def normalize_image(img):
    img[img==255] = 0
    return img / img.flatten().max()

def json_to_transform(json):
    def json_to_rot(json):
        x, y, z, w = json['x'], json['y'], json['z'], json['w']
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

    def json_to_pos(json):
        x, y, z = json['x'], json['y'], json['z']
        return np.array([x, y, z])

    T = np.eye(3)
    T[:2,:2] = json_to_rot(json['orientation'])[:2,:2]
    T[:2,2] = json_to_pos(json['position'])[:2]
    return T

def transform_to_json(T):
    def rot_to_json(R):
        def sign(x):
            return 1. if x >= 0 else -1.
        x = 1 + R[0,0] - R[1,1] - R[2,2]
        y = 1 - R[0,0] + R[1,1] - R[2,2]
        z = 1 - R[0,0] - R[1,1] + R[2,2]
        w = 1 + R[0,0] + R[1,1] + R[2,2]
        x = 0.5 * np.sqrt(np.maximum(x, 0))
        y = 0.5 * np.sqrt(np.maximum(y, 0))
        z = 0.5 * np.sqrt(np.maximum(z, 0))
        w = 0.5 * np.sqrt(np.maximum(w, 0))
        x *= sign(R[2,1] - R[1,2])
        y *= sign(R[0,2] - R[2,0])
        z *= sign(R[1,0] - R[0,1])
        return {'x': x, 'y': y, 'z': z, 'w': w}

    def pos_to_json(p):
        return {'x': p[0], 'y': p[1], 'z': p[2]}

    R = np.eye(3)
    R[:2,:2] = T[:2,:2]
    p = np.zeros(3)
    p[:2] = T[:2,2]
    return {'orientation': rot_to_json(R), 'position': pos_to_json(p)}

def transform(th, p):
    T = np.eye(3)
    c = np.cos(th)
    s = np.sin(th)
    T[0,0], T[0,1] = c, -s
    T[1,0], T[1,1] = s, c
    T[:2,2] = p
    return T

def transform_inverse(T):
    T_inv = np.eye(3)
    T_inv[:2,:2] = T[:2,:2].T
    T_inv[:2,2] = -T_inv[:2,:2].dot(T[:2,2])
    return T_inv

def get_maps(redis_client, agents):
    maps = {}
    for agent in agents:
        map_info = json.loads(redis_client.get("{}::map".format(agent)))
        str_map = redis_client.get("{}::map::data".format(agent))
        arr_map = np.fromstring(base64.decodestring(str_map), dtype=np.int8).astype(np.uint8)
        arr_map = arr_map.reshape((map_info['info']['height'], map_info['info']['width']))
        arr_map = normalize_image(arr_map)

        map_info['data'] = arr_map
        maps[agent] = map_info
    return maps

def get_poses(redis_client, agents):
    T_poses = {}
    headers = {}
    for agent in agents:
        str_pose = redis_client.get("{}::tf".format(agent))
        if str_pose is None:
            continue
        posestamped = json.loads(str_pose)
        T_poses[agent] = json_to_transform(posestamped['pose'])
        headers[agent] = posestamped['header']
    return T_poses, headers

def check_resolution(maps):
    res = None
    for agent in maps:
        if res is None:
            res = maps[agent]['info']['resolution']
        assert res == maps[agent]['info']['resolution']
    return res

def combine_maps(map_0, map_1):
    map_01 = np.zeros((map_0.shape[0], map_0.shape[1], 3), dtype=np.uint8)
    map_01[:,:,1][map_1>0] = 255
    map_01[:,:,2][map_0>0] = 255
    return map_01

def register_maps(agents, maps):
    res = check_resolution(maps)

    T_maps = {agent: {} for agent in agents}
    registered_maps = {agent: {} for agent in agents}
    for i, agent_i in enumerate(agents):
        for j, agent_j in enumerate(agents[i+1:]):
            map_reg_i, p, th, a = imagereg.register(maps[agent_i]['data'], maps[agent_j]['data'], plot=False)
            if map_reg_i is None:
                continue
            registered_map = combine_maps(map_reg_i, maps[agent_j]['data'])

            T_origin = np.eye(3)
            T_origin[:2,2] = np.array([-res * maps[agent_i]['info']['origin']['position']['x'],
                                       -res * maps[agent_i]['info']['origin']['position']['y']])

            T_reg = transform(-np.pi/180. * th, res * p)
            T_i_to_j = T_reg.dot(T_origin)

            T_maps[agent_j][agent_i] = T_i_to_j
            T_maps[agent_i][agent_j] = transform_inverse(T_i_to_j)
            registered_maps[agent_j][agent_i] = registered_map
            registered_maps[agent_i][agent_j] = registered_map[:,:,[0,2,1]]
    return T_maps, registered_maps

def publish_poses(redis_client, agents, T_poses, T_maps, seq):
    for agent in agents:
        T_adversaries = {}
        for adversary in T_maps[agent]:
            if adversary not in T_poses:
                continue
            T_from_adversary = T_maps[agent][adversary]
            T_adversary = T_from_adversary.dot(T_poses[adversary])
            T_adversaries[adversary] = {
                'pose': transform_to_json(T_adversary),
                'header': {
                    'stamp': {'secs': 0, 'nsecs': 0},
                    'frame_id': adversary,
                    'seq': seq
                }
            }
        print("{}: {}".format(agent, T_adversaries))
        redis_client.set("{}::adversaries::tf".format(agent), json.dumps(T_adversaries))

def maps_updated(maps, maps_prev):
    if maps_prev is None:
        return True
    for agent in maps:
        if agent not in maps_prev:
            return True
        map_curr = maps[agent]
        map_prev = maps_prev[agent]
        if map_curr['header']['seq'] != map_prev['header']['seq']:
            return True
    return False

def pose_headers_updated(headers, headers_prev):
    for agent in headers:
        if agent not in headers_prev:
            return True
        header = headers[agent]
        header_prev = headers_prev[agent]
        if header['seq'] != header_prev['seq']:
            return True
    return False

T_maps = None
T_maps_lock = threading.Lock()

def update_maps(agents, redis_hostname):
    global T_maps, T_maps_lock

    pub = {agent: {} for agent in agents}
    for agent_i in agents:
        for agent_j in agents:
            if agent_i == agent_j:
                continue
            pub[agent_i][agent_j] = rospy.Publisher("/map_{}_{}/image_raw/compressed".format(agent_i.replace('.','_'),
                                                                                    agent_j.replace('.','_')),
                                                    CompressedImage, queue_size=10)

    redis_client = redis.Redis(host=redis_hostname)
    maps_prev = None
    while not rospy.is_shutdown():
        maps = get_maps(redis_client, agents)
        if not maps_updated(maps, maps_prev):
            continue

        T_maps_new, registered_maps = register_maps(agents, maps)

        T_maps_lock.acquire()
        T_maps = T_maps_new
        T_maps_lock.release()

        maps_prev = maps

        for agent_i in registered_maps:
            for agent_j in registered_maps[agent_i]:
                img = registered_maps[agent_i][agent_j]
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv.imencode('.jpg', img)[1]).tostring()
                pub[agent_i][agent_j].publish(msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--redis', default="{}.local".format(socket.gethostname()), help="Hostname of Redis master")
    parser.add_argument('--robots', nargs='+', help="Names of turtlebots", required=True)
    args = parser.parse_args()
    # img_0 = normalize_image(cv.imread('map0.png')[:,:,0])
    # img_1 = normalize_image(cv.imread('map1.png')[:,:,0])

    rospy.init_node('map_registration')

    agents = args.robots

    t = threading.Thread(target=update_maps, args=(agents, args.redis))
    t.start()

    redis_client = redis.Redis(host=args.redis)

    pose_headers_prev = None
    seq = 0
    while not rospy.is_shutdown():
        T_maps_lock.acquire()
        if T_maps is None:
            T_maps_lock.release()
            continue
        T_maps_lock.release()

        T_poses, pose_headers = get_poses(redis_client, agents)

        if pose_headers_prev is None or pose_headers_updated(pose_headers, pose_headers_prev):
            T_maps_lock.acquire()
            T_maps_local = T_maps.copy()
            T_maps_lock.release()

            publish_poses(redis_client, agents, T_poses, T_maps_local, seq)
            pose_headers_prev = pose_headers
            seq += 1

    t.join()
