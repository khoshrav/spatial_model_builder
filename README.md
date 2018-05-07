## Keras Training for YOLO ##

This project contains code that performs YOLO training on Keras, using the v2 model as the architecture. The final weights are then used to perform forward pass to build a spatial model of an environment. Since training the model from scratch is unfeasible, we try to retrain the last layer to simulate training of an end to end object recognition model. To learn more on how YOLO works and how to use it, visit this awesome [website](https://pjreddie.com).

The Keras model also has the following callbacks while training:

1. tensorboard logs, which can then be visualized after training is completed.

2. early stopping which stops training if the validation error does not decrease by 0.0001 every epoch for 3 consecutive epochs.

3. save best model weights after every epoch if the validation loss was the best one found yet.

### Steps for training the model ###

1. Install the requirements by `pip install -r requirements.txt`.

2. Download the COCO 2014 training and validation dataset from [here](http://cocodataset.org/#home) and store it in a folder called `data/`.

3. Download the contents from [here](https://gist.github.com/chicham/6ed3842d0d2014987186#file-coco2pascal-py) to convert the COCO annotations to Pascal VOC which is necessary for the training.

4. Run the command `python coco2pascal.py create_annotations . train ../coco_data/train` and `python coco2pascal.py create_annotations . val ../coco_data/train`.

5. Download the weights from [here](https://pjreddie.com/media/files/yolo.weights) and store them at the same folder as the model.

6. Check the configuration file `config.py` and check if everything matches.

7. Training can be started by running `python train.py`.

#### Final weights ####

Our final weights after training the model has been uploaded as a h5 file to Google Drive. This can be used to run further training or even to perform forward passes for prediction. This file can be found [here](https://drive.google.com/open?id=1SPMbAM7fKu8ZjID55pVUiNv3kk-iT0S7)

#### Credits ####

Being a difficult problem, and with lots of data to be handled, along with interconversion between C++ and Python backends for robotic eye integration and Model training, we had to take some help for preprocessing and common utilities for loading and converting data into the proper format. The [preprocessing.py](https://github.com/experiencor/keras-yolo2/blob/master/preprocessing.py) and [utils.py](https://github.com/allanzelener/YAD2K/blob/master/yad2k/utils/utils.py) were used to preprocess the COCO data into a format that could be understood with the pretrained weights, which was decoded properly using the `utils.py` methods.
