#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run both YOLO models on saved images and dump every raw box.

Usage: img_model_probe.py IMAGE [IMAGE ...]
Prints one block per image with all boxes sorted by vertical centre, so a
specific region (e.g. the tier-2 cake site) can be inspected.
"""
import os
import pathlib
import platform
import sys

import cv2
import yolov5
import yaml

if platform.system() != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

ITEM_FILE = os.path.expanduser(
    '~/spark_noetic/src/3rd_app/auto_match_25/auto_match/config/items_config.yaml')

with open(ITEM_FILE) as handle:
    ITEMS = yaml.safe_load(handle)['items']

MODELS = []
for tag, path in (('auto', '~/auto.pt'), ('banana', '~/banana.pt')):
    model = yolov5.load(os.path.expanduser(path))
    model.conf = 0.03
    MODELS.append((tag, model))


def main():
    for image_path in sys.argv[1:]:
        image = cv2.imread(image_path)
        if image is None:
            print('cannot read %s' % image_path)
            continue
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        canvas = image.copy()
        rows = []
        for tag, model in MODELS:
            try:
                results = model(rgb, augment=False)
                predictions = results.xyxy[0]
            except Exception as exc:  # noqa: BLE001
                print('%s inference failed: %s' % (tag, exc))
                continue
            for prediction in predictions:
                x1, y1, x2, y2, conf, cls = [float(v) for v in prediction[:6]]
                name = model.model.names[int(cls)]
                item_id = ITEMS.get(name)
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                rows.append((cy, cx, tag, name, item_id, conf,
                             int(x1), int(y1), int(x2), int(y2)))
        rows.sort()
        print('===== %s ===== %d boxes' % (os.path.basename(image_path), len(rows)))
        for cy, cx, tag, name, item_id, conf, x1, y1, x2, y2 in rows:
            print('  v=%3d u=%3d  %-9s %-13s id=%-4s score=%.3f  box=(%d,%d)-(%d,%d)'
                  % (cy, cx, tag, name, item_id, conf, x1, y1, x2, y2))
            color = (0, 0, 255) if item_id in (2, 20) else (0, 200, 0)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            label = '%s %s %.2f' % (tag[0], name, conf)
            cv2.putText(canvas, label, (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, label, (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        out_path = image_path.replace('.jpg', '_probe.jpg')
        cv2.imwrite(out_path, canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        print('  -> annotated copy: %s' % out_path)


if __name__ == '__main__':
    main()
