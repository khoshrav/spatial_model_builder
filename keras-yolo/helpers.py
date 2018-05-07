import tensorflow as tf
import datetime
import os
from keras.callbacks import EarlyStopping, ModelCheckpoint, TensorBoard
from preprocessing import parse_annotation, BatchGenerator
from config import LABELS, CONFIG_DICT
import cPickle as pickle

def space_to_depth_transform(x):
    return tf.space_to_depth(x, block_size=2)

def early_stopping():
    return EarlyStopping(monitor='val_loss', min_delta=0.001,
            patience=3, mode='min', verbose=1)

def model_checkpoint():
    return ModelCheckpoint('weights_coco_yolo_v2.h5', monitor='val_loss',
            verbose=1, save_best_only=True, mode='min', period=1)

def tensor_board():
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    tensorboard = TensorBoard(log_dir=os.getcwd() + 'coco_' + current_time,
            histogram_freq=0, write_graph=True, write_images=False)
    return tensorboard

def normalize(image):
    return image / 255.

def get_data():
    image_configs = {
    'IMAGE_H': CONFIG_DICT['H'],
    'IMAGE_W': CONFIG_DICT['W'],
    'GRID_H': CONFIG_DICT['GRID_H'],
    'GRID_W': CONFIG_DICT['GRID_W'],
    'BOX': CONFIG_DICT['BOXES'],
    'LABELS': LABELS,
    'CLASS': CONFIG_DICT['NO_CLASSES'],
    'ANCHORS': CONFIG_DICT['ANCHORS'],
    'BATCH_SIZE': CONFIG_DICT['BATCH_SIZE'],
    'TRUE_BOX_BUFFER': CONFIG_DICT['GT_BOX_BUFFER']
}
    train_data = CONFIG_DICT['TRAIN_DATA']
    train_annotations = CONFIG_DICT['TRAIN_ANNOTATIONS']
    val_data = CONFIG_DICT['VAL_DATA']
    val_annotations = CONFIG_DICT['VAL_ANNOTATIONS']

    # train_imgs, seen_train_labels = parse_annotation(train_annotations, train_data, labels=LABELS)
    # Pickle Dump train images
    # with open('train_imgs', 'wb') as fp:
        # pickle.dump(train_imgs, fp)
    with open ('train_imgs', 'rb') as fp:
        train_imgs = pickle.load(fp)
    train_batch = BatchGenerator(train_imgs, image_configs, norm=normalize)
    # val_imgs, seen_valid_labels = parse_annotation(val_annotations, val_data, labels=LABELS)
    # Pickle Dump train images
    # with open('val_imgs', 'wb') as fp:
        # pickle.dump(val_imgs, fp)
    with open ('val_imgs', 'rb') as fp:
        val_imgs = pickle.load(fp)
    val_batch = BatchGenerator(val_imgs, image_configs, norm=normalize, jitter=False)
    return [train_batch, val_batch]

