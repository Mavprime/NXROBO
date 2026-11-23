#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline diagnosis: run BOTH YOLO models on freshly captured camera frames
and dump every raw box (model, class name, score, box) so the physical source
of a false Cake detection can be identified."""

import os
import pathlib
import platform
import time

import cv2
import rospy
import yolov5
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

if platform.system() != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

FRAMES = int(os.environ.get('PROBE_FRAMES', '6'))
OUTDIR = os.path.expanduser('~/logs/probe_frames')
FACTOR = 1.0

bridge = CvBridge()
images = []
frames = []


def cb(msg):
    if len(images) >= FRAMES:
        return
    try:
        cv = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception as exc:  # noqa: BLE001
        print('convert failed: %s' % exc)
        return
    images.append(cv.copy())
    frames.append(time.time())


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    rospy.init_node('raw_model_probe', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, cb, queue_size=1,
                     buff_size=2 ** 24)
    deadline = time.time() + 20.0
    while len(images) < FRAMES and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)
    print('[PROBE] captured %d frames' % len(images))

    # items_config: class name -> item id mapping
    item_file = os.path.expanduser('~/spark_noetic/src/3rd_app/auto_match_25/'
                                   'auto_match/config/items_config.yaml')
    with open(item_file) as handle:
        items = yaml.safe_load(handle)['items']
    print('[PROBE] items_config: %s' % sorted(items.items()))

    models = []
    for tag, path in (('auto.pt', '~/auto.pt'), ('banana.pt', '~/banana.pt')):
        model = yolov5.load(os.path.expanduser(path))
        model.conf = 0.03
        models.append((tag, model))
        print('[PROBE] loaded %s names=%s' % (tag, list(model.model.names.values())))

    for index, image in enumerate(images):
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        canvas = image.copy()
        print('\n===== frame %d =====' % index)
        for tag, model in models:
            try:
                results = model(rgb, augment=False)
                predictions = results.xyxy[0]
            except Exception as exc:  # noqa: BLE001
                print('  %s inference failed: %s' % (tag, exc))
                continue
            for prediction in predictions:
                x1, y1, x2, y2, conf, cls = [float(v) for v in prediction[:6]]
                name = model.model.names[int(cls)]
                item_id = items.get(name)
                print('  %-9s %-14s score=%.3f id=%-3s box=(%d,%d)-(%d,%d) '
                      'center=(%d,%d) size=%dx%d'
                      % (tag, name, conf, item_id, x1, y1, x2, y2,
                         (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1))
                color = (0, 255, 0) if item_id != 2 else (0, 0, 255)
                cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)),
                              color, 2)
                label = '%s %s %.2f' % (tag[0], name, conf)
                cv2.putText(canvas, label, (int(x1), max(12, int(y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3,
                            cv2.LINE_AA)
                cv2.putText(canvas, label, (int(x1), max(12, int(y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                            cv2.LINE_AA)
        raw_path = os.path.join(OUTDIR, 'frame_%02d_raw.jpg' % index)
        ann_path = os.path.join(OUTDIR, 'frame_%02d_boxes.jpg' % index)
        cv2.imwrite(raw_path, image, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        cv2.imwrite(ann_path, canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    print('\n[PROBE] frames written to %s' % OUTDIR)


if __name__ == '__main__':
    main()
