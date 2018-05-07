import numpy as np

LABELS = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

CONFIG_DICT = {
    'H': 416,
    'W': 416,
    'GRID_H': 13,
    'GRID_W': 13,
    'BOXES': 5,
    'NO_CLASSES': len(LABELS),
    'CLASS_WEIGHTS': np.ones(len(LABELS), dtype='float32'),
    'ANCHORS': [0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828],
    'BATCH_SIZE': 16,
    'COORD_SCALE': 1.0,
    'OBJECT_SCALE': 5.0,
    'NO_OBJECT_SCALE': 1.0,
    'CLASS_SCALE': 1.0,
    'WARM_UP_BATCHES': 0,
    'GT_BOX_BUFFER': 50,
    'WEIGHTS': 'data/yolo.weights',
    'TRAIN_DATA': 'data/train2014/',
    'VAL_DATA': 'data/val2014/',
    'TRAIN_ANNOTATIONS': 'coco-data/train/',
    'VAL_ANNOTATIONS': 'coco-data/val/'
}
