#! /usr/bin/env python
from __future__ import print_function

#general imports
import numpy as np
import yaml
from os import path

#Keras imports
from keras.preprocessing import image as keras_image
from keras.models import load_model
from keras.utils.generic_utils import CustomObjectScope
from keras.applications.mobilenet import relu6, DepthwiseConv2D, preprocess_input


class Recognizer:
    def __init__(self,model_dir):
        self.model_dir = model_dir
        with CustomObjectScope({
                'relu6': relu6,
                'DepthwiseConv2D': DepthwiseConv2D
        }):
            self.model_path = path.join(self.model_dir,"model.spec")
            self.class_indices_path = path.join(self.model_dir,"class_indices.yaml")
            self.model = load_model(self.model_path)
            with open(self.class_indices_path,"r") as class_indices_file:
                self.class_indices = yaml.load(class_indices_file)
            self.index_classes = {v: k for k, v in self.class_indices.items()}
            self.model._make_predict_function(
            )  #Quickfix for bug #2397 in keras

    def preprocess(self, image):
        input_shape = self.model.input_shape[1:3]
        image = keras_image.array_to_img(image).resize(input_shape)
        image = keras_image.img_to_array(image)
        image = preprocess_input(image)
        image = np.expand_dims(image, 0)

        return image

    def predict(self, image):
        image = self.preprocess(image)
        res = self.model.predict(image, steps=1)[0]
        probable_class_idx = np.argmax(res)
        return probable_class_idx, self.index_classes[probable_class_idx], res[
            probable_class_idx]