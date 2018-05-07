from keras.models import Model
from keras.layers import Reshape, Conv2D, Input, MaxPooling2D, BatchNormalization, Lambda
from keras.layers.advanced_activations import LeakyReLU
from keras.optimizers import Adam
import tensorflow as tf
import numpy as np
from keras.layers.merge import concatenate
from helpers import space_to_depth_transform
from config import CONFIG_DICT
from utils import WeightReader

W = CONFIG_DICT['W']
H = CONFIG_DICT['H']
NO_CLASSES = CONFIG_DICT['NO_CLASSES']
GRID_H = CONFIG_DICT['GRID_H']
GRID_W = CONFIG_DICT['GRID_W']
ANCHORS = CONFIG_DICT['ANCHORS']
GT_BOX_BUFFER = CONFIG_DICT['GT_BOX_BUFFER']
BOXES = CONFIG_DICT['BOXES']
CLASS_WEIGHTS = CONFIG_DICT['CLASS_WEIGHTS']
BATCH_SIZE = CONFIG_DICT['BATCH_SIZE']
OBJECT_SCALE = CONFIG_DICT['OBJECT_SCALE']
NO_OBJECT_SCALE = CONFIG_DICT['NO_OBJECT_SCALE']
CLASS_SCALE = CONFIG_DICT['CLASS_SCALE']
COORD_SCALE = CONFIG_DICT['COORD_SCALE']
WARM_UP_BATCHES = CONFIG_DICT['WARM_UP_BATCHES']
WEIGHT_PATH = CONFIG_DICT['WEIGHTS']

input_image = Input(shape=(H, W, 3))
gt_boxes  = Input(shape=(1, 1, 1, GT_BOX_BUFFER , 4))

# Conv Layer: 1
x = Conv2D(32, (3, 3), strides=(1,1), padding='same', name='conv_1', use_bias=False)(input_image)
x = BatchNormalization(name='norm_1')(x)
x = LeakyReLU(alpha=0.1)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

# Conv Layer: 2
x = Conv2D(64, (3, 3), strides=(1, 1),  padding='same', name='conv_2', use_bias=False)(x)
x = BatchNormalization(name='norm_2')(x)
x = LeakyReLU(alpha=0.1)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

# Conv Layer: 3-5
x = Conv2D(128, (3, 3), strides=(1, 1), padding='same', name='conv_3', use_bias=False)(x)
x = BatchNormalization(name='norm_3')(x)
x = LeakyReLU(alpha=0.1)(x)
x = Conv2D(64, (1, 1), strides=(1, 1), padding='same', name='conv_4', use_bias=False)(x)
x = BatchNormalization(name='norm_4')(x)
x = LeakyReLU(alpha=0.1)(x)
x = Conv2D(128, (3, 3), strides=(1, 1), padding='same', name='conv_5', use_bias=False)(x)
x = BatchNormalization(name='norm_5')(x)
x = LeakyReLU(alpha=0.1)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

# Conv Layer: 6-7
x = Conv2D(256, (3, 3), strides=(1, 1), padding='same', name='conv_6', use_bias=False)(x)
x = BatchNormalization(name='norm_6')(x)
x = LeakyReLU(alpha=0.1)(x)
x = Conv2D(128, (1, 1), strides=(1, 1), padding='same', name='conv_7', use_bias=False)(x)
x = BatchNormalization(name='norm_7')(x)
x = LeakyReLU(alpha=0.1)(x)

# Conv Layer: 8
x = Conv2D(256, (3, 3), strides=(1, 1), padding='same', name='conv_8', use_bias=False)(x)
x = BatchNormalization(name='norm_8')(x)
x = LeakyReLU(alpha=0.1)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

# Conv Layer: 9-12
no = 9
for _ in xrange(2):
    x = Conv2D(512, (3, 3), strides=(1, 1), padding='same', name='conv_{}'.format(no), use_bias=False)(x)
    x = BatchNormalization(name='norm_{}'.format(no))(x)
    x = LeakyReLU(alpha=0.1)(x)
    no += 1
    x = Conv2D(256, (1, 1), strides=(1, 1), padding='same', name='conv_{}'.format(no), use_bias=False)(x)
    x = BatchNormalization(name='norm_{}'.format(no))(x)
    x = LeakyReLU(alpha=0.1)(x)
    no += 1

# Conv Layer: 13
x = Conv2D(512, (3, 3), strides=(1, 1), padding='same', name='conv_13', use_bias=False)(x)
x = BatchNormalization(name='norm_13')(x)
x = LeakyReLU(alpha=0.1)(x)

divergence = x

x = MaxPooling2D(pool_size=(2, 2))(x)

# Conv Layer: 14-17
no = 14
for _ in xrange(2):
    x = Conv2D(1024, (3, 3), strides=(1, 1), padding='same', name='conv_{}'.format(no), use_bias=False)(x)
    x = BatchNormalization(name='norm_{}'.format(no))(x)
    x = LeakyReLU(alpha=0.1)(x)
    no += 1
    x = Conv2D(512, (1, 1), strides=(1, 1), padding='same', name='conv_{}'.format(no), use_bias=False)(x)
    x = BatchNormalization(name='norm_{}'.format(no))(x)
    x = LeakyReLU(alpha=0.1)(x)
    no += 1

# Conv Layer: 18-20
no = 18
for _ in xrange(3):
    x = Conv2D(1024, (3, 3), strides=(1, 1), padding='same', name='conv_{}'.format(no), use_bias=False)(x)
    x = BatchNormalization(name='norm_{}'.format(no))(x)
    x = LeakyReLU(alpha=0.1)(x)
    no += 1

# Conv Layer: 21 (This deals with the divergence from layer 13)
divergence = Conv2D(64, (1, 1), strides=(1, 1), padding='same', name='conv_21', use_bias=False)(divergence)
divergence = BatchNormalization(name='norm_21')(divergence)
divergence = LeakyReLU(alpha=0.1)(divergence)
divergence = Lambda(space_to_depth_transform)(divergence)

# Merge both the models
x = concatenate([divergence, x])

# Conv Layer: 22
x = Conv2D(1024, (3, 3), strides=(1, 1), padding='same', name='conv_22', use_bias=False)(x)
x = BatchNormalization(name='norm_22')(x)
x = LeakyReLU(alpha=0.1)(x)

# Conv Layer: 23
x = Conv2D(BOXES * (5 + NO_CLASSES), (1,1), strides=(1,1), padding='same', name='conv_23')(x)
x = Reshape((GRID_H, GRID_W, BOXES, 5 + NO_CLASSES))(x)

output = Lambda(lambda args: args[0])([x, gt_boxes])
model = Model([input_image, gt_boxes], output, name='yolo_v2')

def bbox_loss(y_true, y_pred):
    """
        This function defines the custom bounding box loss. Heavily inspired by
        https://github.com/allanzelener/YAD2K/blob/master/yad2k/models/keras_yolo.py
    """
    mask_shape = tf.shape(y_true)[:4]
    cell_x = tf.to_float(tf.reshape(tf.tile(tf.range(GRID_W), [GRID_H]), (1, GRID_H, GRID_W, 1, 1)))
    cell_y = tf.transpose(cell_x, (0,2,1,3,4))
    cell_grid = tf.tile(tf.concat([cell_x,cell_y], -1), [BATCH_SIZE, 1, 1, 5, 1])
    coord_mask = tf.zeros(mask_shape)
    conf_mask = tf.zeros(mask_shape)
    class_mask = tf.zeros(mask_shape)

    seen = tf.Variable(0.)

    # adjust x and y
    pred_box_xy = tf.sigmoid(y_pred[..., :2]) + cell_grid
    true_box_xy = y_true[..., 0:2]

    # adjust w and h
    pred_box_wh = tf.exp(y_pred[..., 2:4]) * np.reshape(ANCHORS, [1, 1, 1, BOXES, 2])
    true_box_wh = y_true[..., 2:4]

    # adjust class probabilities
    pred_box_class = y_pred[..., 5:]

    # adjust confidence
    pred_box_conf = tf.sigmoid(y_pred[..., 4])
    true_wh_half = true_box_wh / 2.
    true_mins = true_box_xy - true_wh_half
    true_maxes = true_box_xy + true_wh_half

    pred_wh_half = pred_box_wh / 2.
    pred_mins = pred_box_xy - pred_wh_half
    pred_maxes = pred_box_xy + pred_wh_half

    intersect_mins = tf.maximum(pred_mins, true_mins)
    intersect_maxes = tf.minimum(pred_maxes, true_maxes)
    intersect_wh = tf.maximum(intersect_maxes - intersect_mins, 0.)
    intersect_areas = intersect_wh[..., 0] * intersect_wh[..., 1]

    true_areas = true_box_wh[..., 0] * true_box_wh[..., 1]
    pred_areas = pred_box_wh[..., 0] * pred_box_wh[..., 1]

    union_areas = pred_areas + true_areas - intersect_areas
    iou_scores = tf.truediv(intersect_areas, union_areas)

    true_box_conf = iou_scores * y_true[..., 4]

    # adjust class probabilities
    true_box_class = tf.argmax(y_true[..., 5:], -1)

    # coordinate mask: simply the position of the ground truth boxes (the predictors)
    coord_mask = tf.expand_dims(y_true[..., 4], axis=-1) * COORD_SCALE

    # confidence mask: penelize predictors + penalize boxes with low IOU
    # penalize the confidence of the boxes, which have IOU with some ground truth box < 0.6
    true_xy = gt_boxes[..., 0:2]
    true_wh = gt_boxes[..., 2:4]

    true_wh_half = true_wh / 2.
    true_mins = true_xy - true_wh_half
    true_maxes = true_xy + true_wh_half

    pred_xy = tf.expand_dims(pred_box_xy, 4)
    pred_wh = tf.expand_dims(pred_box_wh, 4)

    pred_wh_half = pred_wh / 2.
    pred_mins = pred_xy - pred_wh_half
    pred_maxes = pred_xy + pred_wh_half

    intersect_mins = tf.maximum(pred_mins,  true_mins)
    intersect_maxes = tf.minimum(pred_maxes, true_maxes)
    intersect_wh = tf.maximum(intersect_maxes - intersect_mins, 0.)
    intersect_areas = intersect_wh[..., 0] * intersect_wh[..., 1]

    true_areas = true_wh[..., 0] * true_wh[..., 1]
    pred_areas = pred_wh[..., 0] * pred_wh[..., 1]

    union_areas = pred_areas + true_areas - intersect_areas
    iou_scores = tf.truediv(intersect_areas, union_areas)

    best_ious = tf.reduce_max(iou_scores, axis=4)
    conf_mask = conf_mask + tf.to_float(best_ious < 0.6) * (1 - y_true[..., 4]) * NO_OBJECT_SCALE

    # penalize the confidence of the boxes, which are reponsible for corresponding ground truth box
    conf_mask = conf_mask + y_true[..., 4] * OBJECT_SCALE

    # class mask: simply the position of the ground truth boxes (the predictors)
    class_mask = y_true[..., 4] * tf.gather(CLASS_WEIGHTS, true_box_class) * CLASS_SCALE
    no_boxes_mask = tf.to_float(coord_mask < COORD_SCALE/2.)
    seen = tf.assign_add(seen, 1.)

    true_box_xy, true_box_wh, coord_mask = tf.cond(tf.less(seen, WARM_UP_BATCHES),
                          lambda: [true_box_xy + (0.5 + cell_grid) * no_boxes_mask,
                                   true_box_wh + tf.ones_like(true_box_wh) * np.reshape(ANCHORS, [1, 1, 1, BOXES, 2]) * no_boxes_mask,
                                   tf.ones_like(coord_mask)],
                          lambda: [true_box_xy,
                                   true_box_wh,
                                   coord_mask])

    nb_coord_box = tf.reduce_sum(tf.to_float(coord_mask > 0.0))
    nb_conf_box = tf.reduce_sum(tf.to_float(conf_mask  > 0.0))
    nb_class_box = tf.reduce_sum(tf.to_float(class_mask > 0.0))

    loss_xy = tf.reduce_sum(tf.square(true_box_xy-pred_box_xy) * coord_mask)/(nb_coord_box + 1e-6)/2.
    loss_wh = tf.reduce_sum(tf.square(true_box_wh-pred_box_wh) * coord_mask)/(nb_coord_box + 1e-6)/2.
    loss_conf = tf.reduce_sum(tf.square(true_box_conf-pred_box_conf) * conf_mask)/(nb_conf_box + 1e-6)/2.
    loss_class = tf.nn.sparse_softmax_cross_entropy_with_logits(labels=true_box_class, logits=pred_box_class)
    loss_class = tf.reduce_sum(loss_class * class_mask)/(nb_class_box + 1e-6)

    loss = loss_xy + loss_wh + loss_conf + loss_class
    return loss

def load_weights():
    """
        This method loads the pretrained weights from YOLO
    """
    weight_reader = WeightReader(WEIGHT_PATH)
    weight_reader.reset()
    nb_conv = 23
    for i in range(1, nb_conv+1):
        conv_layer = model.get_layer('conv_' + str(i))
        if i < nb_conv:
            norm_layer = model.get_layer('norm_' + str(i))
            size = np.prod(norm_layer.get_weights()[0].shape)
            beta = weight_reader.read_bytes(size)
            gamma = weight_reader.read_bytes(size)
            mean = weight_reader.read_bytes(size)
            var = weight_reader.read_bytes(size)
            weights = norm_layer.set_weights([gamma, beta, mean, var])

        if len(conv_layer.get_weights()) > 1:
            bias   = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[1].shape))
            kernel = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[0].shape))
            kernel = kernel.reshape(list(reversed(conv_layer.get_weights()[0].shape)))
            kernel = kernel.transpose([2,3,1,0])
            conv_layer.set_weights([kernel, bias])
        else:
            kernel = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[0].shape))
            kernel = kernel.reshape(list(reversed(conv_layer.get_weights()[0].shape)))
            kernel = kernel.transpose([2,3,1,0])
            conv_layer.set_weights([kernel])
    # Randomize the weights of the last layer
    # We would only be training the last layer again
    layer = model.layers[-4]
    weights = layer.get_weights()
    new_kernel = np.random.normal(size=weights[0].shape)/(GRID_H*GRID_W)
    new_bias = np.random.normal(size=weights[1].shape)/(GRID_H*GRID_W)
    layer.set_weights([new_kernel, new_bias])

def compile_model():
    """
        This method compiles the Keras Model and returns it for training
    """
    print(model.summary())
    load_weights()
    adam = Adam(lr=0.5e-4, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.)
    model.compile(loss=bbox_loss, optimizer=adam)
    return model
